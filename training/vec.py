"""Multiprocessing rollout pool for CMA-ES.

Each worker owns one ``Simulation`` and reuses it across candidates. Sensor
models are re-created by ``Simulation.reset`` with the same seed, which gives
every candidate in a generation common random numbers.

The simulation is rebuilt only when the requested track changes. A generation
normally shares one track, so the rebuild cost is paid once per worker per
track rather than once per candidate.
"""

from __future__ import annotations

import multiprocessing as mp
from collections.abc import Iterable
from typing import Any

import numpy as np
from simulator.simulation import Simulation

from training.env import (
    ContactMonitor,
    RolloutConfig,
    RolloutResult,
    TrackSpec,
    evaluate_with_simulation,
)

_WORKER_SIM: Simulation | None = None
_WORKER_MONITOR: ContactMonitor | None = None
_WORKER_CONFIG: RolloutConfig | None = None
_WORKER_DEFAULT_START: float = 0.0
_WORKER_KEY: tuple[TrackSpec | None, int] | None = None


def _build_simulation(
    config: RolloutConfig,
    track_spec: TrackSpec | None,
    seed: int | None = None,
) -> tuple[Simulation, ContactMonitor, float]:
    monitor = ContactMonitor()
    track = track_spec.build() if track_spec is not None else None
    sim = Simulation(
        "configs/sim/awake_2027.yaml",
        lidar_subrays=config.subrays,
        seed=config.seed if seed is None else int(seed),
        track=track,
        physics_step_hook=lambda world: monitor.update(world, sim.sim_time),
    )
    sim.scenario.race.laps = 1
    monitor.bind(sim.world)
    return sim, monitor, float(sim.track.start_s)


def init_worker(config: RolloutConfig) -> None:
    global _WORKER_SIM, _WORKER_MONITOR, _WORKER_CONFIG
    global _WORKER_DEFAULT_START, _WORKER_KEY
    sim, monitor, start = _build_simulation(config, None)
    _WORKER_SIM = sim
    _WORKER_MONITOR = monitor
    _WORKER_CONFIG = config
    _WORKER_DEFAULT_START = start
    _WORKER_KEY = None


def _ensure_track(track_spec: TrackSpec | None, seed: int | None) -> None:
    global _WORKER_SIM, _WORKER_MONITOR, _WORKER_DEFAULT_START, _WORKER_KEY
    assert _WORKER_CONFIG is not None
    key = (track_spec, _WORKER_CONFIG.seed if seed is None else int(seed))
    if _WORKER_SIM is not None and key == _WORKER_KEY:
        return
    if _WORKER_SIM is not None:
        _WORKER_SIM.close()
    sim, monitor, start = _build_simulation(_WORKER_CONFIG, track_spec, seed)
    _WORKER_SIM = sim
    _WORKER_MONITOR = monitor
    _WORKER_DEFAULT_START = start
    _WORKER_KEY = key


def _evaluate_task(
    task: tuple[list[float], float | None, TrackSpec | None, int | None],
) -> RolloutResult:
    vector, start_s, track_spec, seed = task
    _ensure_track(track_spec, seed)
    assert _WORKER_SIM is not None
    assert _WORKER_MONITOR is not None
    assert _WORKER_CONFIG is not None
    start = _WORKER_DEFAULT_START if start_s is None else float(start_s)
    return evaluate_with_simulation(
        _WORKER_SIM,
        _WORKER_MONITOR,
        np.asarray(vector, dtype=float),
        _WORKER_CONFIG,
        start_s=start,
    )


class RolloutPool:
    """A small wrapper around ``multiprocessing.Pool`` with a typed API."""

    def __init__(self, config: RolloutConfig, workers: int) -> None:
        self.config = config
        self.workers = max(1, int(workers))
        self._pool = mp.Pool(
            processes=self.workers,
            initializer=init_worker,
            initargs=(config,),
        )

    def evaluate(
        self,
        vectors: Iterable[np.ndarray],
        start_s: float | None = None,
        track_spec: TrackSpec | None = None,
        seed: int | None = None,
    ) -> list[RolloutResult]:
        tasks: list[tuple[list[float], float | None, TrackSpec | None, int | None]] = [
            (np.asarray(vector, dtype=float).tolist(), start_s, track_spec, seed)
            for vector in vectors
        ]
        return list(self._pool.map(_evaluate_task, tasks))

    def close(self) -> None:
        self._pool.close()
        self._pool.join()

    def __enter__(self) -> RolloutPool:
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.close()


__all__ = ["RolloutPool", "init_worker"]
