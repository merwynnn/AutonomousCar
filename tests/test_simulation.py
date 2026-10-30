from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from awake_sim.config import load_sensors, load_vehicle
from awake_sim.sensors.lidar import LidarModel
from awake_sim.sensors.ultrasonic import UltrasonicModel
from awake_sim.sim.vehicle import Actuation
from awake_sim.sim.world import PyBulletWorld
from awake_sim.slam import HAVE_CPP, core
from awake_sim.tracks.geometry import build_test_track, project_to_polyline

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def rig():
    vehicle = load_vehicle(ROOT / "configs" / "vehicles" / "generic_1_10_4wd.yaml")
    sensors = load_sensors(ROOT / "configs" / "sensors" / "ld06.yaml")
    track = build_test_track()
    world = PyBulletWorld(track, vehicle)
    yield vehicle, sensors, track, world
    world.close()


def test_car_drives_forward(rig) -> None:
    _vehicle, _sensors, _track, world = rig
    world.reset(world.track.sample(world.track.start_s))
    world.set_actuation(Actuation(throttle=0.25, steering=0.0))
    start = world.state()
    for _ in range(120):
        world.step()
    end = world.state()
    assert end.speed > 0.2
    assert end.x > start.x + 0.1


def test_lidar_is_non_ideal_and_wall_height_aware(rig) -> None:
    _vehicle, sensors, _track, world = rig
    state = world.state()
    model = LidarModel(sensors.lidar, world, seed=7)
    scan = model.scan(state, 0.0, realism_scale=1.0)
    assert scan.beam_count == sensors.lidar.beams
    assert scan.valid_count() < scan.beam_count  # dropouts / dead beams exist
    assert scan.valid_count() > scan.beam_count * 0.5
    assert scan.t_available > scan.t_measure
    assert np.nanmin(scan.ranges[scan.valid]) >= sensors.lidar.min_range_m
    assert np.nanmax(scan.ranges[scan.valid]) <= sensors.lidar.max_range_m

    ideal = model.scan(state, 0.0, realism_scale=0.0)
    assert ideal.t_available == pytest.approx(0.0)
    assert ideal.valid_count() >= scan.valid_count()

    high = sensors.lidar.model_copy(deep=True)
    high.mount = {"position_m": [0.10, 0.0, 0.60]}
    high_model = LidarModel(high, world, seed=7)
    high_scan = high_model.scan(state, 0.0, realism_scale=0.0)
    # Above the walls and obstacles, a horizontal scan hits nothing.
    assert high_scan.valid_count() < 10


def test_ultrasonic_has_three_realistic_sensors(rig) -> None:
    _vehicle, sensors, _track, world = rig
    model = UltrasonicModel(sensors.ultrasonic, world, seed=3)
    readings = model.sample(world.state(), 0.0, realism_scale=1.0)
    assert len(readings) == 3
    assert {r.name for r in readings} == {"front_left", "front_right", "rear_center"}
    for reading in readings:
        assert reading.t_available > reading.t_measure
        if reading.range_m is not None:
            assert sensors.ultrasonic.min_range_m <= reading.range_m <= sensors.ultrasonic.max_range_m


def test_project_to_polyline() -> None:
    square = np.array([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)])
    s, lateral, _ = project_to_polyline(square, (0.5, 0.1))
    assert 0.0 <= s <= 4.0
    assert abs(lateral) < 0.2


@pytest.mark.cpp
def test_cpp_slam_binding_roundtrip() -> None:
    if not HAVE_CPP:
        pytest.skip("C++ SLAM extension not built")
    assert core is not None
    slam = core.SlamCore(0.05)
    scan = core.Scan([1.0, 2.0, 3.0], [0.0, 0.1, 0.2], 0.0)
    result = slam.process(scan, core.Pose2(1.0, 2.0, 0.3))
    assert result.initialized
    assert result.scan_count == 1
    assert result.pose.x == pytest.approx(1.0)
    assert result.pose.theta == pytest.approx(0.3)
