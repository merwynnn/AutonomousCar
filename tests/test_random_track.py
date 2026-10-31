from __future__ import annotations

import numpy as np
from algorithms.gap_follow import FurthestGapController
from simulator.random_track import build_random_track
from simulator.simulation import Simulation
from simulator.track import project_to_polyline


def test_random_track_density_and_bounds() -> None:
    empty = build_random_track(seed=1, width_m=0.9, object_density=0.0)
    full = build_random_track(seed=2, width_m=0.9, object_density=1.0)
    assert len(empty.obstacles) == 0
    assert len(full.obstacles) == 12
    assert empty.width_m == 0.9
    assert empty.length > 10.0
    for track in (empty, full):
        walls = np.vstack([track.inner, track.outer])
        assert walls[:, 0].min() >= 0.0
        assert walls[:, 0].max() <= 15.0
        assert walls[:, 1].min() >= 0.0
        assert walls[:, 1].max() <= 8.0
        for obstacle in track.obstacles:
            _, lateral, _ = project_to_polyline(
                track.centerline, (obstacle.x, obstacle.y)
            )
            assert abs(lateral) < track.width_m / 2.0


def test_random_track_obstacles_are_on_track() -> None:
    track = build_random_track(seed=3, width_m=0.9, object_density=0.5)
    assert len(track.obstacles) == 6
    for obstacle in track.obstacles:
        _, lateral, _ = project_to_polyline(track.centerline, (obstacle.x, obstacle.y))
        assert abs(lateral) <= track.width_m / 2.0


def test_simulation_accepts_random_track() -> None:
    track = build_random_track(seed=4, width_m=0.9, object_density=0.25)
    simulation = Simulation(backend="mujoco", lidar_subrays=1, realism="ideal", track=track)
    try:
        observation = simulation.reset()
        assert observation.lidar is not None
        assert simulation.track is track
    finally:
        simulation.close()


def test_gap_debug_exposes_clearance_profile() -> None:
    simulation = Simulation(backend="mujoco", lidar_subrays=1, realism="ideal")
    try:
        observation = simulation.reset()
        controller = FurthestGapController()
        for _ in range(3):
            observation = simulation.step(controller.update(observation, 1.2))
        debug = controller.debug
        assert debug.valid
        assert debug.profile.shape[1] == 2
        assert debug.profile.shape[0] > 10
        assert debug.gap_left_rad <= debug.gap_right_rad
        assert debug.front_sector_rad > 0.0
    finally:
        simulation.close()
