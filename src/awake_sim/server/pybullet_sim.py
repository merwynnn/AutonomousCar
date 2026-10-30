"""PyBullet-backed simulation manager used by the live dashboard."""

from __future__ import annotations

import asyncio
import contextlib
from pathlib import Path
from typing import Any

from awake_sim.config import (
    ScenarioConfig,
    SensorsConfig,
    VehicleConfig,
    load_scenario,
    load_sensors,
    load_vehicle,
)
from awake_sim.control.pure_pursuit import PurePursuitController
from awake_sim.control.racing_line import build_racing_line
from awake_sim.control.wall_follow import WallFollowController
from awake_sim.sensors.lidar import LidarModel, LidarScan
from awake_sim.sensors.ultrasonic import UltrasonicModel, UltrasonicReading
from awake_sim.sim.vehicle import Actuation
from awake_sim.sim.world import PyBulletWorld
from awake_sim.slam import HAVE_CPP
from awake_sim.tracks.geometry import TrackGeometry, build_test_track, project_to_polyline

ROOT = Path(__file__).resolve().parents[3]


class PyBulletSimulation:
    """Fixed-step PyBullet simulation with realistic sensors and a race manager."""

    def __init__(
        self,
        scenario_path: str | Path | None = None,
        enable_lidar: bool = True,
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
        # Live mode uses one sub-ray per beam to keep the dashboard real-time.
        # High-fidelity/batch runs can restore the configured 3 sub-rays.
        live_lidar = self.sensors.lidar.model_copy(deep=True)
        live_lidar.divergence["subrays"] = 1
        live_lidar.divergence["live_mode"] = True
        self.lidar = LidarModel(live_lidar, self.world, seed=self.scenario.seed)
        self.ultrasonic = UltrasonicModel(
            self.sensors.ultrasonic, self.world, seed=self.scenario.seed
        )
        self.controller = WallFollowController(base_speed=2.4)
        self.racing_line = build_racing_line(self.track, self.vehicle)
        self.pure_pursuit = PurePursuitController(
            self.racing_line,
            wheelbase_m=self.vehicle.wheelbase_m,
        )
        self.auto_controller = "wall_follow"

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

        self._latest_scan: LidarScan | None = None
        self._latest_sonars: list[UltrasonicReading] = []
        self._scan_seq = 0
        self._scan_dirty = False
        self._queues: set[asyncio.Queue[dict[str, Any]]] = set()
        self._task: asyncio.Task[None] | None = None

        self._sample_scan()
        self._sample_sonars()

    # ------------------------------------------------------------------ state
    def reset(self) -> None:
        self.world.reset(self.track.sample(self.track.start_s))
        self.controller.reset()
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
        self._latest_scan = None
        self._latest_sonars = []
        self._scan_seq = 0
        self._sample_scan()
        self._sample_sonars()
        self._scan_dirty = True

    def _sample_scan(self) -> None:
        if not self.enable_lidar:
            return
        self._latest_scan = self.lidar.scan(
            self.world.state(), self.sim_time, self.realism_scale
        )
        self._scan_seq += 1
        self._scan_dirty = True

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
            # M2 upper bound: pure pursuit uses the ground-truth pose. The SLAM/MCL
            # localization path will replace this in M3.
            state = self.world.state()
            self.actuation = self.pure_pursuit.update(
                state.x, state.y, state.yaw, state.speed
            )
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
            },
            "race": {"laps": self.scenario.race.laps},
            "slam_cpp": HAVE_CPP,
            "warnings": warnings,
            "capabilities": {
                "manual_driving": True,
                "slam": False,
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
            "speed_mps": round(state.speed, 3),
            "slam": False,
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
            self.controller.base_speed = max(0.5, min(6.0, float(msg.get("speed", 2.2))))
        elif kind == "control.mode":
            mode = str(msg.get("mode", "auto"))
            self.control_mode = mode if mode in {"auto", "manual"} else "auto"
            self.manual = Actuation()
        elif kind == "control.controller":
            controller = str(msg.get("controller", "wall_follow"))
            self.auto_controller = (
                controller if controller in {"wall_follow", "pure_pursuit"} else "wall_follow"
            )
            self.actuation = Actuation()
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
