from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from awake_sim.config import load_sensors, load_vehicle
from awake_sim.sensors.lidar import LidarModel
from awake_sim.sim.vehicle import Actuation
from awake_sim.sim.world import PyBulletWorld
from awake_sim.slam import HAVE_CPP
from awake_sim.slam.pipeline import SlamPipeline, deskew_scan
from awake_sim.tracks.geometry import build_test_track

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.cpp
def test_slam_pipeline_tracks_a_moving_scan() -> None:
    if not HAVE_CPP:
        pytest.skip("C++ SLAM extension not built")
    vehicle = load_vehicle(ROOT / "configs" / "vehicles" / "generic_1_10_4wd.yaml")
    sensors = load_sensors(ROOT / "configs" / "sensors" / "ld06.yaml")
    track = build_test_track()
    world = PyBulletWorld(track, vehicle)
    lidar = LidarModel(sensors.lidar, world, seed=5)
    pipeline = SlamPipeline()
    pipeline.set_mcl_enabled(True)
    rng = np.random.default_rng(11)
    errors = []
    mcl_errors = []
    for index in range(8):
        state = world.state()
        scan = lidar.scan(state, index * 0.1, realism_scale=1.0)
        update = pipeline.process(
            scan,
            state.x + rng.normal(0.0, 0.01),
            state.y + rng.normal(0.0, 0.01),
            state.yaw + rng.normal(0.0, 0.003),
            state.speed,
            state.yaw_rate,
        )
        errors.append(np.hypot(update.x - state.x, update.y - state.y))
        assert update.mcl_initialized
        assert update.mcl_particle_count >= 100
        assert 0.0 < update.mcl_ess <= update.mcl_particle_count
        mcl_errors.append(np.hypot(update.mcl_x - state.x, update.mcl_y - state.y))
        world.set_actuation(Actuation(throttle=0.15, steering=0.0))
        for _ in range(24):
            world.step()
    world.close()
    assert max(errors) < 0.25
    assert max(mcl_errors) < 0.5
    occupancy = pipeline.map_array()
    assert np.count_nonzero(occupancy > 0.5) > 100
    payload = pipeline.map_payload()
    assert payload["width"] > 0
    assert len(payload["data"]) > 100


def test_deskew_zero_speed_is_identity() -> None:
    from awake_sim.sensors.lidar import LidarScan

    angles = np.array([-0.5, 0.0, 0.5])
    ranges = np.array([1.0, 2.0, 3.0])
    scan = LidarScan(
        angles=angles,
        ranges=ranges,
        intensities=np.zeros(3),
        valid=np.ones(3, dtype=bool),
        t_measure=0.0,
        t_available=0.0,
    )
    new_angles, new_ranges = deskew_scan(scan, 0.0, 0.0)
    assert np.allclose(new_angles, angles)
    assert np.allclose(new_ranges, ranges)
