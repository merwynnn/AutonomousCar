"""FastAPI backend for the AWAKE simulator dashboard.

Run from the repository root:

    uv run python -m dashboard.server

The simulator core remains independent: this module only adapts it to HTTP and
WebSocket, and derives every car payload from ``simulator.config``.
"""

from __future__ import annotations

import asyncio
import contextlib
import os
import time
from collections.abc import AsyncIterator, MutableMapping
from pathlib import Path
from typing import Any, Literal

from algorithms.gap_follow import FurthestGapController, GapDebug
from algorithms.interface import Actuation, Observation
from algorithms.wall_follow import WallFollowController
from fastapi import FastAPI, Response, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import ValidationError
from ruamel.yaml import YAML
from simulator.car_urdf import build_urdf
from simulator.config import SensorsConfig, VehicleConfig, load_scenario
from simulator.mjcf import build_mjcf
from simulator.random_track import build_random_track
from simulator.simulation import DEFAULT_SCENARIO, Simulation
from simulator.track import build_test_track
from simulator.vehicle import VehicleState

from dashboard.spec import sensor_payload, track_payload, vehicle_payload

WEB_DIST = Path(__file__).resolve().parent / "web" / "dist"
REFERENCE_SCENARIO = (
    Path(__file__).resolve().parents[1] / "simulator" / "configs" / "awake_2027.yaml"
)


class DashboardSimulation:
    """Owns one simulator instance and turns it into dashboard telemetry."""

    def __init__(self, scenario_path: str | Path | None = None, backend: str | None = None) -> None:
        default_scenario = (
            REFERENCE_SCENARIO if REFERENCE_SCENARIO.exists() else DEFAULT_SCENARIO
        )
        self.scenario_path = Path(
            scenario_path or os.environ.get("AWAKE_SCENARIO") or default_scenario
        )
        backend_name = backend or os.environ.get("AWAKE_BACKEND", "mujoco")
        self.backend: Literal["pybullet", "mujoco"] = (
            "mujoco" if backend_name == "mujoco" else "pybullet"
        )
        self.scenario = load_scenario(self.scenario_path)
        self.track = build_test_track(width_m=0.9)
        self.track_mode: Literal["default", "random"] = "default"
        self.track_seed = 2027
        self.track_density = 0.25
        self.track_width_m = 0.9
        self.sim = self._build_sim()
        self.paused = True
        self.finished = False
        self.control_mode: str = "auto"
        self.auto_controller: str = "gap_follow"
        self.speed_limit = 1.5
        self.manual = Actuation()
        self.manual_time = -10.0
        self._scan_key: tuple[float, float] | None = None
        self._queues: set[asyncio.Queue[dict[str, Any]]] = set()
        self._task: asyncio.Task[None] | None = None
        self._reload_pending = False
        self._pending_config: dict[str, Any] | None = None
        self._pending_track: dict[str, Any] | None = None
        self.gap_follow = FurthestGapController()
        self.wall_follow = WallFollowController(base_speed=self.speed_limit)

    # ------------------------------------------------------------------ setup
    def _build_sim(self) -> Simulation:
        # One lidar sub-ray keeps live dashboard frame rates healthy; batch and
        # CLI runs still use the configured fidelity.
        return Simulation(
            self.scenario_path,
            lidar_subrays=1,
            backend=self.backend,
            track=self.track,
        )

    def reload_config(self) -> None:
        with contextlib.suppress(Exception):
            self.sim.close()
        self.scenario = load_scenario(self.scenario_path)
        self.sim = self._build_sim()
        self.paused = True
        self.finished = False
        self.manual = Actuation()
        self._scan_key = None
        self.gap_follow.reset()
        self.wall_follow.reset()
        self.wall_follow.base_speed = self.speed_limit
        self._broadcast(self.hello_message())

    @staticmethod
    def _merge_yaml(target: MutableMapping[str, Any], source: dict[str, Any]) -> None:
        for key in list(target.keys()):
            if key not in source:
                del target[key]
        for key, value in source.items():
            current = target.get(key)
            if isinstance(current, MutableMapping) and isinstance(value, dict):
                DashboardSimulation._merge_yaml(current, value)
            else:
                target[key] = value

    def _write_yaml(self, path: Path, data: dict[str, Any]) -> None:
        """Write values while preserving existing comments where possible."""

        path.parent.mkdir(parents=True, exist_ok=True)
        yaml = YAML()
        yaml.preserve_quotes = True
        yaml.indent(mapping=2, sequence=4, offset=2)
        if path.exists():
            with path.open("r", encoding="utf-8") as handle:
                document = yaml.load(handle)
            if document is None:
                document = data
            else:
                self._merge_yaml(document, data)
        else:
            document = data
        with path.open("w", encoding="utf-8") as handle:
            yaml.dump(document, handle)

    def _apply_config(self, message: dict[str, Any]) -> None:
        vehicle_data = message.get("vehicle")
        sensor_data = message.get("sensors")
        save = bool(message.get("save", False))
        if not isinstance(vehicle_data, dict) or not isinstance(sensor_data, dict):
            self._broadcast(
                {
                    "type": "config.result",
                    "ok": False,
                    "saved": False,
                    "message": "Vehicle and sensor configs are required.",
                }
            )
            return
        try:
            vehicle = VehicleConfig.model_validate(vehicle_data)
            sensors = SensorsConfig.model_validate(sensor_data)
        except ValidationError as exc:
            self._broadcast(
                {
                    "type": "config.result",
                    "ok": False,
                    "saved": False,
                    "message": exc.errors()[0].get("msg", str(exc)),
                    "detail": exc.errors(),
                }
            )
            return

        try:
            new_sim = Simulation(
                self.scenario_path,
                lidar_subrays=1,
                backend=self.backend,
                vehicle=vehicle,
                sensors=sensors,
                track=self.track,
            )
        except Exception as exc:  # noqa: BLE001
            self._broadcast(
                {
                    "type": "config.result",
                    "ok": False,
                    "saved": False,
                    "message": f"Could not build the car model: {exc}",
                }
            )
            return

        if save:
            root = self.scenario_path.parent
            try:
                self._write_yaml(root / self.scenario.vehicle, vehicle.model_dump(mode="json"))
                self._write_yaml(root / self.scenario.sensors, sensors.model_dump(mode="json"))
            except Exception as exc:  # noqa: BLE001
                new_sim.close()
                self._broadcast(
                    {
                        "type": "config.result",
                        "ok": False,
                        "saved": False,
                        "message": f"Could not save YAML: {exc}",
                    }
                )
                return

        with contextlib.suppress(Exception):
            self.sim.close()
        self.sim = new_sim
        self.scenario = load_scenario(self.scenario_path)
        self.paused = True
        self.finished = False
        self.manual = Actuation()
        self._scan_key = None
        self.gap_follow.reset()
        self.wall_follow.reset()
        self.wall_follow.base_speed = self.speed_limit
        self._broadcast(self.hello_message())
        self._broadcast(self.state_message())
        self._broadcast(self.scan_message())
        self._broadcast(
            {
                "type": "config.result",
                "ok": True,
                "saved": save,
                "message": "Saved to YAML and rebuilt." if save else "Applied live.",
            }
        )

    def reset_run(self) -> None:
        self.sim.reset()
        self.paused = True
        self.finished = False
        self.manual = Actuation()
        self._scan_key = None
        self.gap_follow.reset()
        self.wall_follow.reset()

    # --------------------------------------------------------------- messages
    def hello_message(self) -> dict[str, Any]:
        vehicle = self.sim.vehicle
        sensors = self.sim.sensors
        warnings: list[str] = []
        lidar_mount = sensors.lidar.mount
        lidar_height = vehicle.ride_height_m + lidar_mount.position_m[2]
        if lidar_height >= sensors.lidar.wall_height_warning_m:
            warnings.append(
                f"lidar at {lidar_height:.2f} m is at/above the "
                f"{sensors.lidar.wall_height_warning_m:.2f} m wall height"
            )
        return {
            "type": "hello",
            "protocol": 1,
            "mode": self.backend,
            "backend": self.backend,
            "scenario": {
                "name": self.scenario.name,
                "description": self.scenario.description,
                "realism": self.scenario.realism,
                "laps": self.scenario.race.laps,
                "seed": self.scenario.seed,
            },
            "vehicle": vehicle_payload(vehicle),
            "sensors": sensor_payload(sensors, vehicle),
            "config": {
                "vehicle": vehicle.model_dump(mode="json"),
                "sensors": sensors.model_dump(mode="json"),
            },
            "track": {
                **track_payload(self.sim.track),
                "name": (
                    "random_track" if self.track_mode == "random" else "test_track"
                ),
            },
            "track_meta": {
                "mode": self.track_mode,
                "seed": self.track_seed,
                "object_density": round(self.track_density, 3),
                "width_m": round(self.track_width_m, 3),
            },
            "race": {"laps": self.scenario.race.laps},
            "warnings": warnings,
            "paths": {
                "scenario": str(self.scenario_path),
                "vehicle": str(
                    (self.scenario_path.parent / self.scenario.vehicle).resolve()
                ),
                "sensors": str(
                    (self.scenario_path.parent / self.scenario.sensors).resolve()
                ),
            },
            "capabilities": {
                "manual_driving": True,
                "live_reload": True,
                "car_studio": True,
            },
        }

    def state_message(self) -> dict[str, Any]:
        state = self.sim.true_state()
        odom = self.sim.observation.odometry
        lap_times = list(self.sim.lap_times)
        return {
            "type": "state",
            "mode": self.backend,
            "backend": self.backend,
            "t": round(self.sim.sim_time, 4),
            "paused": self.paused,
            "finished": self.finished,
            "car": self._car_message(state),
            "odometry": {
                "x": round(odom.x, 4),
                "y": round(odom.y, 4),
                "theta": round(odom.theta, 4),
            },
            "race": {
                "lap": min(self.sim.lap, self.scenario.race.laps),
                "laps": self.scenario.race.laps,
                "lap_time": round(
                    max(0.0, self.sim.sim_time - self.sim.lap_start_time), 3
                ),
                "last_lap": None if not lap_times else round(lap_times[-1], 3),
                "best_lap": None if not lap_times else round(min(lap_times), 3),
                "total_time": round(sum(lap_times), 3),
                "lap_times": [round(value, 3) for value in lap_times],
                "progress_m": round(self.sim.progress_m, 2),
                "lateral_m": round(self.sim.lateral_m, 3),
            },
            "sensors": {
                "lidar_rate_hz": self.sim.sensors.lidar.rate_hz,
                "lidar_beams": self.sim.sensors.lidar.beams,
                "realism": self.scenario.realism,
                "sonars": [
                    {
                        "name": reading.name,
                        "range_m": None
                        if reading.range_m is None
                        else round(reading.range_m, 3),
                        "valid": reading.valid,
                    }
                    for reading in self.sim.observation.ultrasonic
                ],
            },
            "control_mode": self.control_mode,
            "auto_controller": self.auto_controller,
            "speed_limit": round(self.speed_limit, 3),
            "gap": self._gap_message(),
        }

    def scan_message(self) -> dict[str, Any]:
        scan = self.sim._available_scan
        if scan is None:
            return {
                "type": "scan",
                "t": round(self.sim.sim_time, 4),
                "angles": [],
                "ranges": [],
                "intensities": [],
            }
        ranges = [
            None if not scan.valid[i] else round(float(scan.ranges[i]), 3)
            for i in range(scan.beam_count)
        ]
        return {
            "type": "scan",
            "t": round(scan.t_measure, 4),
            "t_available": round(scan.t_available, 4),
            "frame_id": scan.frame_id,
            "angles": [round(float(value), 5) for value in scan.angles],
            "ranges": ranges,
            "intensities": [round(float(value), 1) for value in scan.intensities],
        }

    def _car_message(self, state: VehicleState) -> dict[str, Any]:
        return {
            "x": round(state.x, 4),
            "y": round(state.y, 4),
            "theta": round(state.yaw, 4),
            "speed": round(state.speed, 3),
            "steering": round(state.steering, 4),
            "yaw_rate": round(state.yaw_rate, 4),
            "wheel_speeds": [round(value, 2) for value in state.wheel_speeds],
        }

    def _gap_message(self) -> dict[str, Any]:
        debug: GapDebug = self.gap_follow.debug
        return {
            "valid": self.auto_controller == "gap_follow" and debug.valid,
            "angle_rad": round(debug.angle_rad, 4),
            "steering_rad": round(debug.steering_rad, 4),
            "range_m": round(debug.range_m, 3),
            "clearance_m": round(debug.clearance_m, 3),
            "width_rad": round(debug.gap_width_rad, 4),
            "gap_left_rad": round(debug.gap_left_rad, 4),
            "gap_right_rad": round(debug.gap_right_rad, 4),
            "front_sector_rad": round(debug.front_sector_rad, 4),
            "throttle": round(debug.throttle, 3),
            "min_clearance_m": round(debug.min_clearance_m, 3),
            "profile": [
                [round(float(angle), 4), round(float(clearance), 3)]
                for angle, clearance in debug.profile
            ],
        }

    # ---------------------------------------------------------------- track
    def _swap_track(self, track: Any, mode: Literal["default", "random"], message: str) -> None:
        try:
            new_sim = Simulation(
                self.scenario_path,
                lidar_subrays=1,
                backend=self.backend,
                vehicle=self.sim.vehicle,
                sensors=self.sim.sensors,
                track=track,
            )
        except Exception as exc:  # noqa: BLE001
            self._broadcast(
                {
                    "type": "track.result",
                    "ok": False,
                    "message": f"Could not build track: {exc}",
                }
            )
            return
        with contextlib.suppress(Exception):
            self.sim.close()
        self.sim = new_sim
        self.track = track
        self.track_mode = mode
        self.paused = True
        self.finished = False
        self.manual = Actuation()
        self._scan_key = None
        self.gap_follow.reset()
        self.wall_follow.reset()
        self.wall_follow.base_speed = self.speed_limit
        self._broadcast(self.hello_message())
        self._broadcast(self.state_message())
        self._broadcast(self.scan_message())
        self._broadcast(
            {"type": "track.result", "ok": True, "message": message}
        )

    def _generate_track(self, message: dict[str, Any]) -> None:
        seed = int(message.get("seed", 2027))
        density = float(message.get("object_density", 0.25))
        width = float(message.get("width_m", 0.9))
        track = build_random_track(
            seed=seed,
            width_m=width,
            object_density=density,
        )
        self.track_seed = seed
        self.track_density = max(0.0, min(1.0, density))
        self.track_width_m = max(0.8, min(1.0, width))
        self._swap_track(track, "random", f"Random track generated (seed {seed}).")

    def _default_track(self) -> None:
        track = build_test_track(width_m=0.9)
        self.track_seed = 2027
        self.track_density = 0.25
        self.track_width_m = 0.9
        self._swap_track(track, "default", "Default test track restored.")

    # ---------------------------------------------------------------- control
    def _auto_command(self, observation: Observation) -> Actuation:
        if self.auto_controller == "wall_follow":
            return self.wall_follow.update(observation)
        return self.gap_follow.update(observation, self.speed_limit)

    def _command(self) -> Actuation:
        if self.control_mode == "manual":
            if time.monotonic() - self.manual_time > 0.45:
                self.manual = Actuation(
                    throttle=0.0, steering=self.manual.steering * 0.90
                )
            return self.manual
        return self._auto_command(self.sim.observation)

    def step_once(self) -> None:
        if self.finished:
            return
        observation = self.sim.step(self._command())
        if observation.t >= self.sim.scenario.simulation.max_time_s:
            self.finished = True
        if self.sim.finished:
            self.finished = True
            self.paused = True

    # --------------------------------------------------------------- commands
    def handle_command(self, message: dict[str, Any]) -> None:
        kind = message.get("type")
        if kind == "run.start":
            if self.finished:
                self.reset_run()
            self.paused = False
        elif kind == "run.pause":
            self.paused = True
        elif kind == "run.reset":
            self.reset_run()
        elif kind == "run.step":
            self.paused = True
            for _ in range(6):
                self.step_once()
        elif kind == "run.set_speed":
            self.speed_limit = max(0.5, min(6.0, float(message.get("speed", 2.4))))
            self.wall_follow.base_speed = self.speed_limit
        elif kind == "control.mode":
            mode = str(message.get("mode", "auto"))
            self.control_mode = mode if mode in {"auto", "manual"} else "auto"
            self.manual = Actuation()
        elif kind == "control.controller":
            controller = str(message.get("controller", "gap_follow"))
            self.auto_controller = (
                controller
                if controller in {"gap_follow", "wall_follow"}
                else "gap_follow"
            )
            self.gap_follow.reset()
            self.wall_follow.reset()
        elif kind == "drive.command":
            self.control_mode = "manual"
            self.manual = Actuation(
                throttle=max(-1.0, min(1.0, float(message.get("throttle", 0.0)))),
                steering=max(-0.55, min(0.55, float(message.get("steering", 0.0)))),
            )
            self.manual_time = time.monotonic()
        elif kind == "config.reload":
            self._reload_pending = True
        elif kind == "config.apply":
            self._pending_config = message
        elif kind == "track.generate":
            self._pending_track = message
        elif kind == "track.default":
            self._pending_track = {"type": "track.default"}
        elif kind == "ping":
            pass

    # ------------------------------------------------------------- transport
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

    async def run_loop(self) -> None:
        tick_period = 1.0 / self.scenario.simulation.control_hz
        loop = asyncio.get_running_loop()
        next_tick = loop.time()
        telemetry_count = 0
        while True:
            next_tick += tick_period
            if self._pending_config is not None:
                pending = self._pending_config
                self._pending_config = None
                self._apply_config(pending)
            if self._pending_track is not None:
                pending_track = self._pending_track
                self._pending_track = None
                if pending_track.get("type") == "track.default":
                    self._default_track()
                else:
                    self._generate_track(pending_track)
            if self._reload_pending:
                self._reload_pending = False
                with contextlib.suppress(Exception):
                    self.reload_config()
            if not self.paused and not self.finished:
                self.step_once()
            telemetry_count += 1
            if telemetry_count % 2 == 0:
                self._broadcast(self.state_message())
                scan = self.sim._available_scan
                if scan is not None:
                    key = (scan.t_measure, scan.t_available)
                    if key != self._scan_key:
                        self._scan_key = key
                        self._broadcast(self.scan_message())
            delay = next_tick - loop.time()
            if delay < 0:
                next_tick = loop.time()
                await asyncio.sleep(0)
            else:
                await asyncio.sleep(delay)

    def status(self) -> dict[str, Any]:
        return {
            "mode": self.backend,
            "backend": self.backend,
            "scenario": self.scenario.name,
            "paused": self.paused,
            "finished": self.finished,
            "sim_time": round(self.sim.sim_time, 3),
            "vehicle": self.sim.vehicle.name,
            "controller": (
                self.control_mode
                if self.control_mode == "manual"
                else self.auto_controller
            ),
            "speed_limit": self.speed_limit,
            "lidar_beams": self.sim.sensors.lidar.beams,
            "slam": False,
        }

    def start(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self.run_loop())

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
            self._task = None
        self.sim.close()


manager = DashboardSimulation()


@contextlib.asynccontextmanager
async def lifespan(_app: FastAPI) -> AsyncIterator[None]:
    manager.start()
    try:
        yield
    finally:
        await manager.stop()


app = FastAPI(title="AWAKE 2027 Car Lab", version="0.2.0", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/health")
def health() -> dict[str, Any]:
    return {
        "status": "ok",
        "mode": manager.backend,
        "backend": manager.backend,
        "vehicle": manager.sim.vehicle.name,
    }


@app.get("/health")
def health_alias() -> dict[str, Any]:
    return health()


@app.get("/api/status")
def status() -> dict[str, Any]:
    return manager.status()


@app.get("/api/car")
def car() -> dict[str, Any]:
    return {
        "vehicle": vehicle_payload(manager.sim.vehicle),
        "sensors": sensor_payload(manager.sim.sensors, manager.sim.vehicle),
        "paths": manager.hello_message()["paths"],
    }


@app.get("/api/config")
def config() -> dict[str, Any]:
    return {
        "vehicle": manager.sim.vehicle.model_dump(mode="json"),
        "sensors": manager.sim.sensors.model_dump(mode="json"),
        "paths": manager.hello_message()["paths"],
    }


@app.get("/api/car/urdf")
def car_urdf() -> Response:
    return Response(
        content=build_urdf(manager.sim.vehicle),
        media_type="application/xml",
        headers={"Content-Disposition": 'attachment; filename="car.urdf"'},
    )


@app.get("/api/car/mjcf")
def car_mjcf() -> Response:
    return Response(
        content=build_mjcf(
            manager.sim.vehicle,
            manager.sim.track,
            timestep_s=1.0 / manager.sim.physics_dt,
        ),
        media_type="application/xml",
        headers={"Content-Disposition": 'attachment; filename="car.xml"'},
    )


@app.get("/api/track")
def track() -> dict[str, Any]:
    return track_payload(manager.sim.track)


@app.websocket("/ws/telemetry")
async def telemetry(websocket: WebSocket) -> None:
    await websocket.accept()
    queue = manager.subscribe()
    try:
        await websocket.send_json(manager.hello_message())
        await websocket.send_json(manager.state_message())
        await websocket.send_json(manager.scan_message())

        async def receive_commands() -> None:
            while True:
                message = await websocket.receive_json()
                if isinstance(message, dict):
                    manager.handle_command(message)

        receiver = asyncio.create_task(receive_commands())
        try:
            while True:
                await websocket.send_json(await queue.get())
        finally:
            receiver.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await receiver
    except WebSocketDisconnect:
        pass
    finally:
        manager.unsubscribe(queue)


if WEB_DIST.exists():
    app.mount("/", StaticFiles(directory=WEB_DIST, html=True), name="dashboard")


def main() -> None:
    import uvicorn

    uvicorn.run("dashboard.server:app", host="0.0.0.0", port=8000, reload=False)


if __name__ == "__main__":
    main()
