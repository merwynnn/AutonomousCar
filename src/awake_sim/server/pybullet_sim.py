"""PyBullet-backed simulation manager used by the live dashboard."""

from __future__ import annotations

import asyncio
import contextlib
import json
import math
import time
from pathlib import Path
from typing import Any

import numpy as np

from awake_sim.config import (
    ScenarioConfig,
    SensorsConfig,
    VehicleConfig,
    load_scenario,
    load_sensors,
    load_vehicle,
)
from awake_sim.control.gap_follow import FurthestGapController
from awake_sim.control.pure_pursuit import PurePursuitController
from awake_sim.control.racing_line import build_racing_line
from awake_sim.control.wall_follow import WallFollowController
from awake_sim.sensors.lidar import LidarModel, LidarScan
from awake_sim.sensors.ultrasonic import UltrasonicModel, UltrasonicReading
from awake_sim.sim.odometry import OdometryModel
from awake_sim.sim.vehicle import Actuation
from awake_sim.sim.world import PyBulletWorld
from awake_sim.slam import HAVE_CPP
from awake_sim.slam.pipeline import SlamPipeline
from awake_sim.tracks.geometry import TrackGeometry, build_test_track, project_to_polyline

ROOT = Path(__file__).resolve().parents[3]


class PyBulletSimulation:
    """Fixed-step PyBullet simulation with realistic sensors and a race manager."""

    def __init__(
        self,
        scenario_path: str | Path | None = None,
        enable_lidar: bool = True,
        enable_slam: bool = True,
        lidar_subrays: int = 1,
    ) -> None:
        scenario_file = (
            Path(scenario_path)
            if scenario_path
            else ROOT / "configs" / "scenarios" / "test_track_baseline.yaml"
        )
        self.scenario: ScenarioConfig = load_scenario(scenario_file)
        self.vehicle: VehicleConfig = load_vehicle(ROOT / self.scenario.vehicle)
        self.sensors: SensorsConfig = load_sensors(ROOT / self.scenario.sensors)
        self.track: TrackGeometry = build_test_track(width_m=0.9)
        self.world = PyBulletWorld(self.track, self.vehicle)
        self.enable_lidar = enable_lidar
        self.enable_slam = enable_slam and HAVE_CPP and enable_lidar
        # Live mode uses one sub-ray per beam to keep the dashboard real-time.
        # High-fidelity/batch runs can restore the configured 3 sub-rays.
        live_lidar = self.sensors.lidar.model_copy(deep=True)
        live_lidar.divergence["subrays"] = max(1, lidar_subrays)
        live_lidar.divergence["live_mode"] = True
        self.lidar = LidarModel(live_lidar, self.world, seed=self.scenario.seed)
        self.ultrasonic = UltrasonicModel(
            self.sensors.ultrasonic, self.world, seed=self.scenario.seed
        )
        self.controller = WallFollowController(base_speed=2.4)
        self.gap_follow = FurthestGapController()
        self.racing_line = build_racing_line(
            self.track, self.vehicle, lateral_margin=0.25
        )
        self.speed_limit = 2.4
        self.pure_pursuit = PurePursuitController(
            self.racing_line,
            wheelbase_m=self.vehicle.wheelbase_m,
            speed_limit=self.speed_limit,
        )
        self.auto_controller = "wall_follow"
        self.localization_mode = "gt"

        mount = self.sensors.lidar.mount.get("position_m", [0.10, 0.0, 0.14])
        self.slam = (
            SlamPipeline(
                resolution_m=0.05,
                width_m=15.0,
                height_m=8.0,
                lidar_offset_x=float(mount[0]),
                lidar_offset_y=float(mount[1]),
                max_range_m=self.sensors.lidar.max_range_m,
                scan_period_s=1.0 / self.sensors.lidar.rate_hz,
            )
            if self.enable_slam
            else None
        )
        self.odometry = OdometryModel(self.scenario.odometry, seed=self.scenario.seed)
        self.odometry.reset(self.track.sample(self.track.start_s))
        self.slam_mcl_enabled = False
        self.slam_loop_enabled = False
        self.slam_global_relocalization_enabled = False
        if self.slam is not None:
            self.slam.set_mcl_enabled(self.slam_mcl_enabled)
            self.slam.set_loop_closure_enabled(self.slam_loop_enabled)
            self.slam.set_global_relocalization_enabled(
                self.slam_global_relocalization_enabled
            )
        self.slam_pose: tuple[float, float, float] | None = None
        self.slam_ate = 0.0
        self.slam_scan_ms = 0.0
        self.slam_match_score = 0.0
        self.slam_inliers = 0
        self.slam_scan_count = 0
        self.slam_loop_closures = 0
        self.slam_graph_nodes = 0
        self.slam_graph_edges = 0
        self.slam_loop_event = False
        self.slam_loop_candidate = -1
        self.slam_loop_match: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.slam_mcl_pose: tuple[float, float, float] | None = None
        self.slam_mcl_ate = 0.0
        self.slam_mcl_ess = 0.0
        self.slam_mcl_spread = 0.0
        self.slam_mcl_particles: list[tuple[float, float, float]] = []
        self.slam_global_relocalizations = 0
        self.slam_relocalized = False
        self.slam_map_payload: dict[str, Any] | None = None
        self._slam_dirty = False
        self.odom_drift_m = 0.0

        self.physics_dt = 1.0 / self.scenario.simulation.physics_hz
        self.control_every = 4  # 60 Hz
        self.sonar_every = 12  # 20 Hz
        self.scan_every = 24  # 10 Hz
        self.tick_steps = 8  # 30 Hz telemetry ticks
        self.realism_scale = {"ideal": 0.0, "realistic": 1.0, "harsh": 2.0}[
            self.scenario.realism
        ]

        self.paused = False
        self.finished = False
        self.sim_time = 0.0
        self.step_count = 0
        self.lap = 1
        self.lap_start_time = 0.0
        self.last_lap_time: float | None = None
        self.lap_times: list[float] = []
        self.penalties = 0
        self.progress_m = 0.0
        self.lateral_m = 0.0
        self.last_progress = 0.0
        self.lap_armed = False
        self.lap_distance = 0.0
        self.control_mode = "auto"
        self.manual = Actuation()
        self.manual_time = -10.0
        self.actuation = Actuation()
        self.stuck_time = 0.0
        self.recovery_time = 0.0
        self.recovery_steer = 0.0
        self.control_pose: tuple[float, float, float] | None = None

        self._latest_scan: LidarScan | None = None
        self._latest_sonars: list[UltrasonicReading] = []
        self._scan_seq = 0
        self._scan_dirty = False
        self._queues: set[asyncio.Queue[dict[str, Any]]] = set()
        self._task: asyncio.Task[None] | None = None

        self._sample_scan()
        self._sample_sonars()

    # ------------------------------------------------------------------ state
    def reset(self, keep_map: bool = False) -> None:
        start_pose = self.track.sample(self.track.start_s)
        self.world.reset(start_pose)
        self.odometry.reset(start_pose)
        self.controller.reset()
        self.gap_follow.reset()
        self.pure_pursuit.reset()
        self.paused = True
        self.finished = False
        self.sim_time = 0.0
        self.step_count = 0
        self.lap = 1
        self.lap_start_time = 0.0
        self.last_lap_time = None
        self.lap_times = []
        self.penalties = 0
        self.progress_m = 0.0
        self.lateral_m = 0.0
        self.last_progress = 0.0
        self.lap_armed = False
        self.lap_distance = 0.0
        self.control_mode = "auto"
        self.manual = Actuation()
        self.actuation = Actuation()
        self.stuck_time = 0.0
        self.recovery_time = 0.0
        self.recovery_steer = 0.0
        self.control_pose = None
        self._latest_scan = None
        self._latest_sonars = []
        self._scan_seq = 0
        if self.slam is not None:
            if keep_map:
                self.slam.reset_tracking()
                self.slam.set_map_frozen(True)
                self.slam.set_mcl_enabled(self.slam_mcl_enabled)
            else:
                self.slam.reset()
                self.slam.set_map_frozen(False)
        self.slam_pose = None
        self.slam_ate = 0.0
        self.slam_scan_ms = 0.0
        self.slam_match_score = 0.0
        self.slam_inliers = 0
        self.slam_scan_count = 0
        self.slam_loop_closures = 0
        self.slam_graph_nodes = 0
        self.slam_graph_edges = 0
        self.slam_loop_event = False
        self.slam_loop_candidate = -1
        self.slam_loop_match = (0.0, 0.0, 0.0)
        self.slam_mcl_pose = None
        self.slam_mcl_ate = 0.0
        self.slam_mcl_ess = 0.0
        self.slam_mcl_spread = 0.0
        self.slam_mcl_particles = []
        self.slam_global_relocalizations = 0
        self.slam_relocalized = False
        if not keep_map:
            self.slam_map_payload = None
        self._slam_dirty = False
        self.odom_drift_m = 0.0
        self._sample_scan()
        self._sample_sonars()
        self._scan_dirty = True

    def _sample_scan(self) -> None:
        if not self.enable_lidar:
            return
        state = self.world.state()
        self._latest_scan = self.lidar.scan(state, self.sim_time, self.realism_scale)
        self._scan_seq += 1
        self._scan_dirty = True
        if self.slam is not None:
            odom = self.odometry.pose
            self.odom_drift_m = float(np.hypot(odom.x - state.x, odom.y - state.y))
            started = time.perf_counter()
            update = self.slam.process(
                self._latest_scan,
                odom.x,
                odom.y,
                odom.theta,
                state.speed,
                state.yaw_rate,
            )
            self.slam_scan_ms = (time.perf_counter() - started) * 1000.0
            self._apply_slam_update(update, state)

    def _apply_slam_update(self, update: Any, state: Any) -> None:
        self.slam_pose = (update.x, update.y, update.theta)
        self.slam_ate = float(np.hypot(update.x - state.x, update.y - state.y))
        self.slam_match_score = update.match_score
        self.slam_inliers = update.inliers
        self.slam_scan_count = update.scan_count
        self.slam_loop_closures = update.loop_closures
        self.slam_graph_nodes = update.graph_nodes
        self.slam_graph_edges = update.graph_edges
        self.slam_loop_event = update.loop_closure_this_scan
        self.slam_loop_candidate = update.loop_candidate
        self.slam_loop_match = (
            update.loop_match_x,
            update.loop_match_y,
            update.loop_match_theta,
        )
        self.slam_mcl_pose = (
            (update.mcl_x, update.mcl_y, update.mcl_theta)
            if update.mcl_initialized
            else None
        )
        self.slam_mcl_ate = (
            float(np.hypot(update.mcl_x - state.x, update.mcl_y - state.y))
            if update.mcl_initialized
            else 0.0
        )
        self.slam_mcl_ess = update.mcl_ess
        self.slam_mcl_spread = update.mcl_spread
        self.slam_global_relocalizations = update.global_relocalizations
        self.slam_relocalized = update.relocalized_this_scan
        if update.relocalized_this_scan:
            self.control_pose = None
        if self.slam is not None and update.mcl_initialized:
            particles = self.slam.mcl_particles()
            self.slam_mcl_particles = particles[::2]
        else:
            self.slam_mcl_particles = []
        if (
            self.slam_scan_count == 1
            or self.slam_scan_count % 10 == 0
            or self.slam_loop_event
        ):
            self.slam_map_payload = self.slam.map_payload() if self.slam is not None else None
        self._slam_dirty = True

    def _apply_recovery(self, command: Actuation, true_speed: float) -> None:
        """Back out and reorient if the car is pinned against a wall.

        This is a sensor/actuator safety layer, not localization cheating. It uses
        the measured speed and the controller command only.
        """
        dt = self.control_every * self.physics_dt
        if self.recovery_time > 0.0:
            self.recovery_time -= dt
            self.actuation = Actuation(
                throttle=-0.45,
                steering=self.recovery_steer,
            )
            return
        if abs(true_speed) < 0.05:
            self.stuck_time += dt
        else:
            self.stuck_time = 0.0
        if self.stuck_time > 0.8:
            self.recovery_time = 1.2
            self.stuck_time = 0.0
            self.recovery_steer = 0.0
            if self.slam is not None and self.slam_global_relocalization_enabled:
                self.slam.request_global_relocalization()
                self.control_pose = None
            self.actuation = Actuation(throttle=-0.65, steering=0.0)
            return
        self.actuation = command

    def _sample_sonars(self) -> None:
        self._latest_sonars = self.ultrasonic.sample(
            self.world.state(), self.sim_time, self.realism_scale
        )

    def _update_control(self) -> None:
        if self.control_mode == "manual":
            if self.sim_time - self.manual_time > 0.45:
                self.manual = Actuation(throttle=0.0, steering=self.manual.steering * 0.9)
            self.actuation = self.manual
            return
        if self.auto_controller == "pure_pursuit":
            state = self.world.state()
            if self.localization_mode == "gt":
                pose = (state.x, state.y, state.yaw)
                speed = state.speed
                self.control_pose = None
            else:
                if self.localization_mode == "odom":
                    odom = self.odometry.pose
                    raw_pose = (odom.x, odom.y, odom.theta)
                else:  # mcl
                    if self.slam_mcl_pose is None:
                        self.actuation = Actuation(throttle=0.0, steering=0.0)
                        return
                    raw_pose = self.slam_mcl_pose
                speed = self.odometry.last_speed
                if self.control_pose is None:
                    pose = raw_pose
                else:
                    alpha = 0.35
                    dtheta = (raw_pose[2] - self.control_pose[2] + math.pi) % (
                        2.0 * math.pi
                    ) - math.pi
                    pose = (
                        self.control_pose[0] + alpha * (raw_pose[0] - self.control_pose[0]),
                        self.control_pose[1] + alpha * (raw_pose[1] - self.control_pose[1]),
                        self.control_pose[2] + alpha * dtheta,
                    )
                self.control_pose = pose
            command = self.pure_pursuit.update(pose[0], pose[1], pose[2], speed)
            self._apply_recovery(command, state.speed)
            return
        if self.auto_controller == "gap_follow":
            state = self.world.state()
            scan = self._latest_scan
            if scan is None:
                self.actuation = Actuation()
                return
            if self.sim_time < scan.t_available:
                # Lidar-only controller: hold the last command until the scan
                # has actually arrived (no future data).
                return
            command = self.gap_follow.update(
                scan, self.control_every * self.physics_dt, self.speed_limit
            )
            self._apply_recovery(command, state.speed)
            return
        scan = self._latest_scan
        if scan is None:
            self.actuation = Actuation()
            return
        if self.sim_time < scan.t_available:
            # Real controllers hold their last command while a scan is in flight.
            return
        self.actuation = self.controller.update(scan, self.control_every * self.physics_dt)

    def _update_progress(self) -> None:
        state = self.world.state()
        s, lateral, _ = project_to_polyline(self.track.centerline, (state.x, state.y))
        progress = (s - self.track.start_s) % self.track.length
        if progress > self.track.length - 0.2:
            progress = 0.0
        self.progress_m = progress
        self.lateral_m = lateral
        # Arm lap counting only after the car has actually travelled half a lap.
        # A small roll at the start line must not count as a completed lap.
        self.lap_distance += max(0.0, state.speed) * self.physics_dt
        if self.lap_distance > 0.5 * self.track.length:
            self.lap_armed = True
        if (
            self.lap_armed
            and self.last_progress > 0.8 * self.track.length
            and self.progress_m < 0.2 * self.track.length
        ):
            lap_time = self.sim_time - self.lap_start_time
            self.lap_times.append(lap_time)
            self.last_lap_time = lap_time
            self.lap_start_time = self.sim_time
            self.lap += 1
            self.lap_armed = False
            self.lap_distance = 0.0
            if self.lap > self.scenario.race.laps:
                self.finished = True
                self.paused = True
        self.last_progress = self.progress_m

    def step_once(self) -> None:
        if self.enable_lidar and self.step_count % self.scan_every == 0:
            self._sample_scan()
        if self.step_count % self.sonar_every == 0:
            self._sample_sonars()
        if self.step_count % self.control_every == 0:
            self._update_control()
        self.world.set_actuation(self.actuation)
        self.world.step()
        state = self.world.state()
        self.odometry.update(state.speed, state.yaw_rate, self.physics_dt)
        self.sim_time += self.physics_dt
        self.step_count += 1
        self._update_progress()

    # --------------------------------------------------------------- messages
    def hello(self) -> dict[str, Any]:
        mount_height = self.sensors.lidar.mount.get("position_m", [0, 0, 0])[2]
        warnings: list[str] = []
        if mount_height >= self.sensors.lidar.wall_height_warning_m:
            warnings.append(
                f"lidar mount {mount_height:.2f} m is at/above the "
                f"{self.sensors.lidar.wall_height_warning_m:.2f} m wall height"
            )
        return {
            "type": "hello",
            "protocol": 1,
            "mode": "pybullet",
            "scenario": self.scenario.model_dump(mode="json"),
            "vehicle": {
                "name": self.vehicle.name,
                "length_m": self.vehicle.dimensions.length_m,
                "width_m": self.vehicle.dimensions.width_m,
                "wheelbase_m": self.vehicle.wheelbase_m,
                "drive": self.vehicle.drive,
            },
            "sensors": {
                "lidar": {
                    "beams": self.sensors.lidar.beams,
                    "rate_hz": self.sensors.lidar.rate_hz,
                    "max_range_m": self.sensors.lidar.max_range_m,
                    "mount": self.sensors.lidar.mount,
                },
                "ultrasonic": [s.name for s in self.sensors.ultrasonic.sensors],
                "imu_rate_hz": self.sensors.imu.rate_hz,
            },
            "track": {
                "name": "test_track",
                "width_m": self.track.width_m,
                "bounds": list(self.track.bounds),
                "centerline": [
                    [round(float(x), 4), round(float(y), 4)] for x, y in self.track.centerline
                ],
                "inner": [
                    [round(float(x), 4), round(float(y), 4)] for x, y in self.track.inner
                ],
                "outer": [
                    [round(float(x), 4), round(float(y), 4)] for x, y in self.track.outer
                ],
                "wall_height_m": 0.16,
                "wall_radius_m": 0.08,
                "start_distance_m": self.track.start_s,
                "track_length_m": round(self.track.length, 3),
                "obstacles": [
                    {
                        "name": obstacle.name,
                        "kind": obstacle.kind,
                        "x": obstacle.x,
                        "y": obstacle.y,
                        "yaw": obstacle.yaw,
                        "length": obstacle.length,
                        "width": obstacle.width,
                        "height": obstacle.height,
                        "radius": obstacle.radius,
                    }
                    for obstacle in self.track.obstacles
                ],
            },
            "race": {"laps": self.scenario.race.laps},
            "slam_cpp": HAVE_CPP,
            "warnings": warnings,
            "capabilities": {
                "manual_driving": True,
                "slam": self.enable_slam,
                "replay": False,
                "batch": False,
            },
        }

    def state_message(self) -> dict[str, Any]:
        state = self.world.state()
        return {
            "type": "state",
            "mode": "pybullet",
            "t": round(self.sim_time, 4),
            "paused": self.paused,
            "finished": self.finished,
            "car": {
                "x": round(state.x, 4),
                "y": round(state.y, 4),
                "theta": round(state.yaw, 4),
                "speed": round(state.speed, 3),
                "steering": round(state.steering, 4),
                "yaw_rate": round(state.yaw_rate, 4),
            },
            "race": {
                "lap": min(self.lap, self.scenario.race.laps),
                "laps": self.scenario.race.laps,
                "lap_time": round(max(0.0, self.sim_time - self.lap_start_time), 3),
                "last_lap": None if self.last_lap_time is None else round(self.last_lap_time, 3),
                "best_lap": round(min(self.lap_times), 3) if self.lap_times else None,
                "total_time": round(sum(self.lap_times), 3),
                "penalties": self.penalties,
                "progress_m": round(self.progress_m, 2),
                "lateral_m": round(self.lateral_m, 3),
            },
            "sensors": {
                "lidar_rate_hz": self.sensors.lidar.rate_hz,
                "lidar_beams": self.sensors.lidar.beams,
                "realism": self.scenario.realism,
                "scan_seq": self._scan_seq,
                "sonars": [
                    {
                        "name": reading.name,
                        "range_m": None
                        if reading.range_m is None
                        else round(reading.range_m, 3),
                        "valid": reading.valid,
                    }
                    for reading in self._latest_sonars
                ],
            },
            "control_mode": self.control_mode,
            "auto_controller": self.auto_controller,
            "localization_mode": self.localization_mode,
            "speed_limit": self.speed_limit,
            "gap": {
                "valid": (
                    self.auto_controller == "gap_follow"
                    and self.gap_follow.debug.valid
                ),
                "angle_rad": round(self.gap_follow.debug.angle_rad, 4),
                "steering_rad": round(self.gap_follow.debug.steering_rad, 4),
                "range_m": round(self.gap_follow.debug.range_m, 3),
                "clearance_m": round(self.gap_follow.debug.clearance_m, 3),
                "width_rad": round(self.gap_follow.debug.gap_width_rad, 4),
            },
        }

    def scan_message(self) -> dict[str, Any]:
        scan = self._latest_scan
        if scan is None:
            self._sample_scan()
            scan = self._latest_scan
        if scan is None:
            return {
                "type": "scan",
                "t": round(self.sim_time, 4),
                "seq": self._scan_seq,
                "frame_id": "lidar",
                "angles": [],
                "ranges": [],
                "intensities": [],
                "realism": self.scenario.realism,
            }
        ranges = [
            None if not scan.valid[i] else round(float(scan.ranges[i]), 3)
            for i in range(scan.beam_count)
        ]
        intensities = [round(float(v), 1) for v in scan.intensities]
        return {
            "type": "scan",
            "t": round(scan.t_measure, 4),
            "seq": self._scan_seq,
            "frame_id": scan.frame_id,
            "angles": [round(float(a), 5) for a in scan.angles],
            "ranges": ranges,
            "intensities": intensities,
            "realism": self.scenario.realism,
        }

    def slam_message(self, include_map: bool = False) -> dict[str, Any]:
        message: dict[str, Any] = {
            "type": "slam",
            "available": self.slam_pose is not None,
            "t": round(self.sim_time, 4),
            "pose": None
            if self.slam_pose is None
            else {
                "x": round(self.slam_pose[0], 4),
                "y": round(self.slam_pose[1], 4),
                "theta": round(self.slam_pose[2], 4),
            },
            "ate_m": round(self.slam_ate, 4),
            "scan_ms": round(self.slam_scan_ms, 2),
            "match_score": round(self.slam_match_score, 4),
            "inliers": self.slam_inliers,
            "scan_count": self.slam_scan_count,
            "loop_closures": self.slam_loop_closures,
            "loop_event": self.slam_loop_event,
            "graph_nodes": self.slam_graph_nodes,
            "graph_edges": self.slam_graph_edges,
            "loop_candidate": self.slam_loop_candidate,
            "loop_match": {
                "x": round(self.slam_loop_match[0], 4),
                "y": round(self.slam_loop_match[1], 4),
                "theta": round(self.slam_loop_match[2], 4),
            },
            "odom_drift_m": round(self.odom_drift_m, 4),
            "odometry": {
                "x": round(self.odometry.x, 4),
                "y": round(self.odometry.y, 4),
                "theta": round(self.odometry.theta, 4),
            },
            "options": {
                "mcl_enabled": self.slam_mcl_enabled,
                "loop_closure_enabled": self.slam_loop_enabled,
                "global_relocalization_enabled": self.slam_global_relocalization_enabled,
            },
            "mcl": {
                "initialized": self.slam_mcl_pose is not None,
                "pose": None
                if self.slam_mcl_pose is None
                else {
                    "x": round(self.slam_mcl_pose[0], 4),
                    "y": round(self.slam_mcl_pose[1], 4),
                    "theta": round(self.slam_mcl_pose[2], 4),
                },
                "ate_m": round(self.slam_mcl_ate, 4),
                "ess": round(self.slam_mcl_ess, 1),
                "spread_m": round(self.slam_mcl_spread, 4),
                "global_relocalizations": self.slam_global_relocalizations,
                "relocalized": self.slam_relocalized,
                "particles": [
                    [round(x, 3), round(y, 3)] for x, y, _theta in self.slam_mcl_particles
                ],
            },
        }
        if include_map and self.slam_map_payload is not None:
            message["map"] = self.slam_map_payload
        return message

    def save_map(self, path: str | Path, metadata: dict[str, Any] | None = None) -> Path:
        if self.slam is None:
            raise RuntimeError("SLAM is not enabled")
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "map": self.slam.map_payload(),
            "seed": self.scenario.seed,
            "track": "test_track",
            "start_pose": {
                "x": self.track.sample(self.track.start_s).x,
                "y": self.track.sample(self.track.start_s).y,
                "theta": self.track.sample(self.track.start_s).theta,
            },
            "metadata": metadata or {},
        }
        target.write_text(json.dumps(payload), encoding="utf-8")
        return target

    def status(self) -> dict[str, Any]:
        state = self.world.state()
        return {
            "mode": "pybullet",
            "scenario": self.scenario.name,
            "paused": self.paused,
            "finished": self.finished,
            "sim_time": round(self.sim_time, 3),
            "lap": self.lap,
            "laps": self.scenario.race.laps,
            "track_length_m": round(self.track.length, 3),
            "controller": self.control_mode if self.control_mode == "manual" else self.auto_controller,
            "localization": self.localization_mode,
            "speed_limit": self.speed_limit,
            "speed_mps": round(state.speed, 3),
            "slam": self.enable_slam,
            "slam_scan_ms": round(self.slam_scan_ms, 2),
            "slam_ate_m": round(self.slam_ate, 4),
            "mcl_enabled": self.slam_mcl_enabled,
            "loop_closure_enabled": self.slam_loop_enabled,
        }

    # --------------------------------------------------------------- commands
    def handle_command(self, msg: dict[str, Any]) -> None:
        kind = msg.get("type")
        if kind == "run.start":
            if self.finished:
                self.reset()
            self.paused = False
        elif kind == "run.pause":
            self.paused = True
        elif kind == "run.reset":
            self.reset()
        elif kind == "run.step":
            self.paused = True
            for _ in range(self.tick_steps):
                self.step_once()
        elif kind == "run.set_speed":
            self.speed_limit = max(0.5, min(6.0, float(msg.get("speed", 3.0))))
            self.controller.base_speed = self.speed_limit
            self.pure_pursuit.speed_limit = self.speed_limit
        elif kind == "control.mode":
            mode = str(msg.get("mode", "auto"))
            self.control_mode = mode if mode in {"auto", "manual"} else "auto"
            self.manual = Actuation()
        elif kind == "control.controller":
            controller = str(msg.get("controller", "wall_follow"))
            self.auto_controller = (
                controller
                if controller in {"wall_follow", "pure_pursuit", "gap_follow"}
                else "wall_follow"
            )
            self.gap_follow.reset()
            self.actuation = Actuation()
        elif kind == "localization.mode":
            mode = str(msg.get("mode", "gt"))
            self.localization_mode = mode if mode in {"gt", "odom", "mcl"} else "gt"
            self.control_pose = None
            if self.localization_mode == "mcl" and self.slam is not None:
                self.slam_mcl_enabled = True
                self.slam.set_mcl_enabled(True)
                self._slam_dirty = True
        elif kind == "slam.set_options":
            if self.slam is not None:
                if "mcl_enabled" in msg:
                    self.slam_mcl_enabled = bool(msg["mcl_enabled"])
                    self.slam.set_mcl_enabled(self.slam_mcl_enabled)
                    if not self.slam_mcl_enabled:
                        self.slam_mcl_pose = None
                        self.slam_mcl_particles = []
                if "loop_closure_enabled" in msg:
                    self.slam_loop_enabled = bool(msg["loop_closure_enabled"])
                    self.slam.set_loop_closure_enabled(self.slam_loop_enabled)
                if "global_relocalization_enabled" in msg:
                    self.slam_global_relocalization_enabled = bool(
                        msg["global_relocalization_enabled"]
                    )
                    self.slam.set_global_relocalization_enabled(
                        self.slam_global_relocalization_enabled
                    )
                self._slam_dirty = True
        elif kind == "slam.relocalize":
            if self.slam is not None:
                self.slam.request_global_relocalization()
                self._slam_dirty = True
        elif kind == "slam.reset_mcl":
            if self.slam is not None:
                self.slam.set_mcl_enabled(False)
                self.slam.set_mcl_enabled(self.slam_mcl_enabled)
                self.slam_mcl_pose = None
                self.slam_mcl_particles = []
                self._slam_dirty = True
        elif kind == "drive.command":
            self.control_mode = "manual"
            self.manual = Actuation(
                throttle=max(-1.0, min(1.0, float(msg.get("throttle", 0.0)))),
                steering=max(-0.55, min(0.55, float(msg.get("steering", 0.0)))),
            )
            self.manual_time = self.sim_time
        elif kind == "ping":
            pass

    # ------------------------------------------------------------- broadcast
    def subscribe(self) -> asyncio.Queue[dict[str, Any]]:
        queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=64)
        self._queues.add(queue)
        return queue

    def unsubscribe(self, queue: asyncio.Queue[dict[str, Any]]) -> None:
        self._queues.discard(queue)

    def _broadcast(self, message: dict[str, Any]) -> None:
        for queue in list(self._queues):
            try:
                queue.put_nowait(message)
            except asyncio.QueueFull:
                with contextlib.suppress(asyncio.QueueEmpty):
                    queue.get_nowait()
                with contextlib.suppress(asyncio.QueueFull):
                    queue.put_nowait(message)

    async def _run(self) -> None:
        period = 1.0 / max(1.0, self.scenario.simulation.telemetry_hz)
        loop = asyncio.get_running_loop()
        while True:
            tick_start = loop.time()
            if not self.paused and not self.finished:
                # Fixed number of physics steps per tick keeps runs deterministic.
                for _ in range(self.tick_steps):
                    self.step_once()
            self._broadcast(self.state_message())
            if self._scan_dirty:
                self._broadcast(self.scan_message())
                self._scan_dirty = False
            if self._slam_dirty:
                include_map = (
                    self.slam_scan_count == 1 or self.slam_scan_count % 10 == 0
                )
                self._broadcast(self.slam_message(include_map=include_map))
                self._slam_dirty = False
            # Sleep only the remaining part of the tick so the sim stays real-time.
            work = loop.time() - tick_start
            await asyncio.sleep(max(0.0, period - work))

    def start(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self._run())

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
            self._task = None
