"""One-lap rollout harness for CMA-ES policy search.

The harness deliberately evaluates a single lap: it is fast, gives the search
more independent samples per hour, and matches the project's current
optimisation target. Contact detection runs on every physics substep so a
brief wall touch cannot hide between control ticks.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import mujoco
import numpy as np
from algorithms.learned_gap import (
    B2_SCALARS,
    BeamScorerPolicy,
    StructuredGapPolicy,
    b2_layout,
)
from simulator.random_track import build_random_track
from simulator.simulation import Simulation
from simulator.track import TrackGeometry


@dataclass(frozen=True)
class TrackSpec:
    """Deterministic description of one random training track."""

    seed: int
    width_m: float = 0.9
    object_density: float = 0.25
    object_scale: float = 1.25
    object_scale_jitter: float = 0.30

    def build(self) -> TrackGeometry:
        return build_random_track(
            seed=self.seed,
            width_m=self.width_m,
            object_density=self.object_density,
            object_scale=self.object_scale,
            object_scale_jitter=self.object_scale_jitter,
        )


@dataclass(frozen=True)
class RolloutConfig:
    policy_type: str = "b1"
    hidden: int = 0
    subrays: int = 1
    seed: int = 42
    max_time_s: float = 32.0
    collect_trace: bool = False
    terminate_on_contact: bool = True


@dataclass
class RolloutResult:
    policy_type: str
    fitness: float
    score: float
    lap_time_s: float | None
    finished: bool
    progress_m: float
    contacts: int
    contact_steps: int
    max_impulse: float
    top_speed_mps: float
    mean_speed_mps: float
    min_wall_distance_m: float
    sim_time_s: float
    steps: int
    trace: dict[str, list[float]] = field(default_factory=dict)
    meta: dict[str, Any] = field(default_factory=dict)


class ContactMonitor:
    """Count robot-vs-wall/obstacle contacts at physics-step resolution."""

    def __init__(self) -> None:
        self.names: list[str] = []
        self.obstacle: np.ndarray | None = None
        self.floor: np.ndarray | None = None
        self.contact_steps = 0
        self.contacts = 0
        self.max_impulse = 0.0
        self.first_contact_t: float | None = None
        self._force = np.zeros(6, dtype=float)

    def bind(self, world: Any) -> None:
        model = world.model
        self.names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
            for i in range(model.ngeom)
        ]
        self.obstacle = np.array(
            [name.startswith(("wall", "obstacle")) for name in self.names], dtype=bool
        )
        self.floor = np.array([name == "floor" for name in self.names], dtype=bool)

    def reset(self) -> None:
        self.contact_steps = 0
        self.contacts = 0
        self.max_impulse = 0.0
        self.first_contact_t = None

    def update(self, world: Any, sim_time: float | None = None) -> None:
        if self.obstacle is None:
            self.bind(world)
        assert self.obstacle is not None and self.floor is not None
        data = world.data
        model = world.model
        for i in range(data.ncon):
            contact = data.contact[i]
            g1, g2 = int(contact.geom1), int(contact.geom2)
            obs1, obs2 = bool(self.obstacle[g1]), bool(self.obstacle[g2])
            if obs1 == obs2:
                continue
            other = g2 if obs1 else g1
            if self.floor[other]:
                continue
            self.contact_steps += 1
            if self.first_contact_t is None:
                self.first_contact_t = sim_time
            mujoco.mj_contactForce(model, data, i, self._force)
            self.max_impulse = max(self.max_impulse, float(abs(self._force[0])))
        if self.contact_steps and not self.contacts:
            self.contacts = 1


def _make_policy(
    policy_type: str,
    vector: np.ndarray,
    hidden: int,
) -> StructuredGapPolicy | BeamScorerPolicy:
    if policy_type == "b2":
        return BeamScorerPolicy(vector, hidden=hidden)
    return StructuredGapPolicy(vector)


def score_rollout(result: RolloutResult, max_time_s: float) -> float:
    """Lexicographic score: clean beats dirty, finished beats unfinished."""

    smooth = result.meta.get("smoothness_penalty", 0.0)
    wall = result.meta.get("wall_penalty", 0.0)
    if result.contacts:
        return (
            result.progress_m
            - 1000.0
            - 10.0 * min(result.contact_steps, 2000)
            - 5.0 * min(result.max_impulse, 200.0)
            - 50.0 * wall
            - smooth
        )
    if result.finished and result.lap_time_s is not None:
        return (
            1_000_000.0
            + (max_time_s - result.lap_time_s)
            - 50.0 * wall
            - smooth
        )
    return result.progress_m - 50.0 * wall - smooth


def run_vector(
    vector: np.ndarray,
    config: RolloutConfig,
    start_s: float | None = None,
    track_spec: TrackSpec | None = None,
) -> RolloutResult:
    """Run one candidate in a fresh simulation (used by tests and single runs)."""

    monitor = ContactMonitor()
    track = track_spec.build() if track_spec is not None else None
    sim = Simulation(
        "configs/sim/awake_2027.yaml",
        lidar_subrays=config.subrays,
        seed=config.seed,
        track=track,
        physics_step_hook=lambda world: monitor.update(world, sim.sim_time),
    )
    sim.scenario.race.laps = 1
    monitor.bind(sim.world)
    return evaluate_with_simulation(sim, monitor, vector, config, start_s)


def evaluate_with_simulation(
    sim: Simulation,
    monitor: ContactMonitor,
    vector: np.ndarray,
    config: RolloutConfig,
    start_s: float | None = None,
) -> RolloutResult:
    if start_s is not None:
        sim.track.start_s = float(start_s)
    monitor.reset()
    observation = sim.reset()
    policy = _make_policy(config.policy_type, np.asarray(vector, dtype=float), config.hidden)
    policy.reset()

    steps = 0
    last_steer = 0.0
    last_throttle = 0.0
    steer_delta = 0.0
    throttle_delta = 0.0
    wall_penalty = 0.0
    speed_sum = 0.0
    top_speed = 0.0
    min_wall = float("inf")
    trace: dict[str, list[float]] = {
        "t": [],
        "x": [],
        "y": [],
        "yaw": [],
        "speed": [],
        "steering": [],
        "throttle": [],
        "lateral": [],
    }

    while not sim.finished and sim.sim_time < config.max_time_s:
        command = policy.act(observation)
        observation = sim.step(command)
        steps += 1
        state = sim.true_state()
        speed = float(state.speed)
        speed_sum += speed
        top_speed = max(top_speed, abs(speed))
        wall_distance = max(0.0, sim.track.width_m / 2.0 - abs(float(sim.lateral_m)))
        min_wall = min(min_wall, wall_distance)
        wall_penalty += max(0.0, 0.075 - wall_distance) ** 2
        steer_delta += (command.steering - last_steer) ** 2
        throttle_delta += (command.throttle - last_throttle) ** 2
        last_steer = command.steering
        last_throttle = command.throttle
        if config.collect_trace:
            trace["t"].append(float(sim.sim_time))
            trace["x"].append(float(state.x))
            trace["y"].append(float(state.y))
            trace["yaw"].append(float(state.yaw))
            trace["speed"].append(speed)
            trace["steering"].append(float(command.steering))
            trace["throttle"].append(float(command.throttle))
            trace["lateral"].append(float(sim.lateral_m))
        if config.terminate_on_contact and monitor.contact_steps > 0:
            break
        if sim.sim_time > 8.0 and sim.progress_m < 0.5 and abs(speed) < 0.12:
            break

    status = sim.status()
    lap_time = status.lap_times[-1] if status.lap_times else None
    finished = bool(sim.finished and lap_time is not None)
    mean_speed = speed_sum / max(steps, 1)
    smoothness = 0.2 * (steer_delta + 0.5 * throttle_delta) / max(steps, 1)
    result = RolloutResult(
        policy_type=config.policy_type,
        fitness=0.0,
        score=0.0,
        lap_time_s=lap_time,
        finished=finished,
        progress_m=float(sim.progress_m),
        contacts=monitor.contacts,
        contact_steps=monitor.contact_steps,
        max_impulse=monitor.max_impulse,
        top_speed_mps=top_speed,
        mean_speed_mps=mean_speed,
        min_wall_distance_m=min_wall if np.isfinite(min_wall) else 0.0,
        sim_time_s=float(sim.sim_time),
        steps=steps,
        trace=trace if config.collect_trace else {},
        meta={
            "wall_penalty": wall_penalty,
            "smoothness_penalty": smoothness,
            "steer_delta": steer_delta,
            "throttle_delta": throttle_delta,
            "lateral_m": float(sim.lateral_m),
            "estimator_drift_m": float(
                np.hypot(
                    status.true_pose.x - status.odometry_pose.x,
                    status.true_pose.y - status.odometry_pose.y,
                )
            ),
        },
    )
    result.score = score_rollout(result, config.max_time_s)
    result.fitness = -result.score
    return result


def b2_scalar_slice(hidden: int) -> slice:
    layout = b2_layout(hidden)
    return slice(layout["scalars"], layout["scalars"] + len(B2_SCALARS))


__all__ = [
    "ContactMonitor",
    "RolloutConfig",
    "RolloutResult",
    "TrackSpec",
    "b2_scalar_slice",
    "evaluate_with_simulation",
    "run_vector",
    "score_rollout",
]
