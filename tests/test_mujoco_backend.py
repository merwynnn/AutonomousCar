from __future__ import annotations

from pathlib import Path

import numpy as np
from algorithms.gap_follow import FurthestGapController
from simulator.config import load_vehicle
from simulator.mjcf import build_mjcf
from simulator.mujoco_world import MuJoCoWorld
from simulator.simulation import Simulation
from simulator.track import build_test_track
from simulator.vehicle import Actuation

ROOT = Path(__file__).resolve().parents[1]


def test_mjcf_compiles_with_expected_mechanism() -> None:
    import mujoco

    vehicle = load_vehicle(ROOT / "configs" / "vehicle_awake2027.yaml")
    track = build_test_track()
    model = mujoco.MjModel.from_xml_string(build_mjcf(vehicle, track))
    joints = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        for index in range(model.njnt)
    }
    assert "chassis_free" in joints
    assert "fl_steer" in joints
    assert "fl_wheel_spin" in joints
    assert model.nu == 6


def test_mujoco_car_drives_forward() -> None:
    vehicle = load_vehicle(ROOT / "configs" / "vehicle_awake2027.yaml")
    track = build_test_track()
    world = MuJoCoWorld(track, vehicle, physics_hz=240.0)
    try:
        world.reset(track.sample(track.start_s))
        start = world.state()
        world.set_actuation(Actuation(throttle=0.25, steering=0.0))
        for _ in range(120):
            world.step()
        end = world.state()
        assert end.speed > 0.5
        assert end.x > start.x + 0.1
    finally:
        world.close()


def test_mujoco_backend_keeps_sensor_boundary() -> None:
    simulation = Simulation(backend="mujoco", lidar_subrays=1, realism="ideal")
    try:
        observation = simulation.reset()
        assert observation.lidar is not None
        assert observation.lidar.beam_count > 0
        assert "true_pose" not in observation.__dataclass_fields__
        controller = FurthestGapController()
        for _ in range(60):
            observation = simulation.step(controller.update(observation, 1.2))
        assert np.isfinite(observation.odometry.x)
        assert observation.sensors.imu is not None
        assert observation.sensors.encoders is not None
        assert observation.sensors.steering is not None
    finally:
        simulation.close()


def test_mujoco_lidar_sees_track_walls() -> None:
    simulation = Simulation(backend="mujoco", lidar_subrays=1, realism="ideal")
    try:
        observation = simulation.reset()
        scan = observation.lidar
        assert scan is not None
        valid = scan.valid & np.isfinite(scan.ranges_m)
        assert int(valid.sum()) > scan.beam_count * 0.5
        assert float(np.nanmin(scan.ranges_m[valid])) >= simulation.sensors.lidar.min_range_m
        assert float(np.nanmax(scan.ranges_m[valid])) <= simulation.sensors.lidar.max_range_m
    finally:
        simulation.close()
