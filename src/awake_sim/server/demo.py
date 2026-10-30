"""M0 demo simulation.

This is a deliberately small placeholder that drives a car around the PDF-like
test track and produces a synthetic lidar scan. It exists to exercise the
dashboard, WebSocket protocol and track geometry before the PyBullet simulation
lands in M1. It is labelled ``mode: demo`` in every message.
"""

from __future__ import annotations

import asyncio
import contextlib
import math
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
from awake_sim.slam import HAVE_CPP
from awake_sim.tracks.geometry import Pose2, TrackGeometry, build_test_track, ramp_scan

ROOT = Path(__file__).resolve().parents[3]


class DemoSimulation:
    """A tiny centreline follower used by the M0 dashboard."""

    def __init__(self, scenario_path: str | Path | None = None) -> None:
        scenario_file = (
            Path(scenario_path)
            if scenario_path
            else ROOT / "configs" / "scenarios" / "test_track_baseline.yaml"
        )
        self.scenario: ScenarioConfig = load_scenario(scenario_file)
        self.vehicle: VehicleConfig = load_vehicle(ROOT / self.scenario.vehicle)
        self.sensors: SensorsConfig = load_sensors(ROOT / self.scenario.sensors)
        self.track: TrackGeometry = build_test_track(width_m=0.9)
        self.rng = np.random.default_rng(self.scenario.seed)

        self.paused = False
        self.finished = False
        self.started = False
        self.sim_time = 0.0
        self.distance = 0.0
        self.speed = 2.0
        self.lap = 1
        self.lap_start_time = 0.0
        self.last_lap_time: float | None = None
        self.lap_times: list[float] = []
        self.penalties = 0

        self._queues: set[asyncio.Queue[dict[str, Any]]] = set()
        self._task: asyncio.Task[None] | None = None
        self._next_scan = 0.0
        self._scan_seq = 0

        self.realism_scale = {
            "ideal": 0.0,
            "realistic": 1.0,
            "harsh": 2.0,
        }.get(self.scenario.realism, 1.0)

    # ------------------------------------------------------------------ state
    @property
    def pose(self) -> Pose2:
        return self.track.sample(self.track.start_s + self.distance)

    def reset(self) -> None:
        self.paused = True
        self.finished = False
        self.started = False
        self.sim_time = 0.0
        self.distance = 0.0
        self.lap = 1
        self.lap_start_time = 0.0
        self.last_lap_time = None
        self.lap_times = []
        self.penalties = 0
        self._next_scan = 0.0
        self._scan_seq = 0
        self.rng = np.random.default_rng(self.scenario.seed)

    def step(self, dt: float) -> None:
        if self.paused or self.finished:
            return
        self.started = True
        self.sim_time += dt
        self.distance += self.speed * dt
        if self.distance >= self.track.length:
            self.distance -= self.track.length
            lap_time = self.sim_time - self.lap_start_time
            self.lap_times.append(lap_time)
            self.last_lap_time = lap_time
            self.lap_start_time = self.sim_time
            self.lap += 1
            if self.lap > self.scenario.race.laps:
                self.finished = True
                self.paused = True

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
            "mode": "demo",
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
                "ultrasonic": [
                    s.name for s in self.sensors.ultrasonic.sensors
                ],
                "imu_rate_hz": self.sensors.imu.rate_hz,
            },
            "track": {
                "name": "test_track",
                "width_m": self.track.width_m,
                "bounds": list(self.track.bounds),
                "centerline": [[round(float(x), 4), round(float(y), 4)] for x, y in self.track.centerline],
                "inner": [[round(float(x), 4), round(float(y), 4)] for x, y in self.track.inner],
                "outer": [[round(float(x), 4), round(float(y), 4)] for x, y in self.track.outer],
                "wall_height_m": 0.16,
                "wall_radius_m": 0.08,
                "start_distance_m": self.track.start_s,
            },
            "race": {"laps": self.scenario.race.laps},
            "slam_cpp": HAVE_CPP,
            "warnings": warnings,
            "capabilities": {
                "manual_driving": False,
                "slam": False,
                "replay": False,
                "batch": False,
            },
        }

    def state_message(self) -> dict[str, Any]:
        pose = self.pose
        return {
            "type": "state",
            "mode": "demo",
            "t": round(self.sim_time, 4),
            "paused": self.paused,
            "finished": self.finished,
            "car": {
                "x": round(pose.x, 4),
                "y": round(pose.y, 4),
                "theta": round(pose.theta, 4),
                "speed": round(0.0 if self.paused else self.speed, 3),
            },
            "race": {
                "lap": min(self.lap, self.scenario.race.laps),
                "laps": self.scenario.race.laps,
                "lap_time": round(
                    0.0
                    if self.lap_start_time is None
                    else max(0.0, self.sim_time - self.lap_start_time),
                    3,
                ),
                "last_lap": None if self.last_lap_time is None else round(self.last_lap_time, 3),
                "best_lap": round(min(self.lap_times), 3) if self.lap_times else None,
                "total_time": round(sum(self.lap_times), 3),
                "penalties": self.penalties,
            },
            "sensors": {
                "lidar_rate_hz": self.sensors.lidar.rate_hz,
                "lidar_beams": self.sensors.lidar.beams,
                "realism": self.scenario.realism,
            },
        }

    def scan_message(self) -> dict[str, Any]:
        if self.realism_scale == 0.0:
            sigma = 0.0
            dropout = 0.0
        else:
            sigma = self.sensors.lidar.noise.range_sigma_m * self.realism_scale
            dropout = (
                self.sensors.lidar.dropout.get("base_probability", 0.005)
                * self.realism_scale
            )
        scan = ramp_scan(
            self.track,
            self.pose,
            beams=self.sensors.lidar.beams,
            max_range=self.sensors.lidar.max_range_m,
            rng=self.rng,
            noise_sigma_m=sigma,
            dropout_probability=min(0.5, dropout),
        )
        angles = [round(float(a), 5) for a in scan[:, 0]]
        ranges = [None if math.isnan(r) else round(float(r), 3) for r in scan[:, 1]]
        self._scan_seq += 1
        return {
            "type": "scan",
            "t": round(self.sim_time, 4),
            "seq": self._scan_seq,
            "frame_id": "lidar",
            "angles": angles,
            "ranges": ranges,
            "realism": self.scenario.realism,
        }

    def status(self) -> dict[str, Any]:
        return {
            "mode": "demo",
            "scenario": self.scenario.name,
            "paused": self.paused,
            "finished": self.finished,
            "sim_time": self.sim_time,
            "lap": self.lap,
            "laps": self.scenario.race.laps,
            "track_length_m": round(self.track.length, 3),
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
            self.step(float(msg.get("dt", 1.0 / 30.0)))
        elif kind == "run.set_speed":
            self.speed = max(0.2, min(8.0, float(msg.get("speed", self.speed))))
        elif kind == "ping":
            pass

    # ------------------------------------------------------------- broadcast
    def subscribe(self) -> asyncio.Queue[dict[str, Any]]:
        queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=32)
        self._queues.add(queue)
        return queue

    def unsubscribe(self, queue: asyncio.Queue[dict[str, Any]]) -> None:
        self._queues.discard(queue)

    def _broadcast(self, message: dict[str, Any]) -> None:
        for queue in list(self._queues):
            try:
                queue.put_nowait(message)
            except asyncio.QueueFull:
                # Drop old telemetry rather than slowing the simulation.
                with contextlib.suppress(asyncio.QueueEmpty):
                    queue.get_nowait()
                with contextlib.suppress(asyncio.QueueFull):
                    queue.put_nowait(message)

    async def _run(self) -> None:
        loop = asyncio.get_running_loop()
        last = loop.time()
        period = 1.0 / max(1.0, self.scenario.simulation.telemetry_hz)
        while True:
            await asyncio.sleep(period)
            now = loop.time()
            dt = min(now - last, 0.1)
            last = now
            if not self.paused and not self.finished:
                self.step(dt)
            self._broadcast(self.state_message())
            if self.sim_time >= self._next_scan:
                self._broadcast(self.scan_message())
                self._next_scan = self.sim_time + 1.0 / self.sensors.lidar.rate_hz

    def start(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self._run())

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
            self._task = None
