from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from algorithms.gap_follow import FurthestGapController
from simulator.config import load_vehicle, mount_position, mount_rpy_deg
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
    simulation = Simulation(lidar_subrays=1, realism="ideal")
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
    simulation = Simulation(lidar_subrays=1, realism="ideal")
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


def test_simulation_exposes_an_odometry_compensated_scan() -> None:
    simulation = Simulation(lidar_subrays=1, realism="ideal")
    try:
        observation = simulation.reset()
        for _ in range(10):
            observation = simulation.step(Actuation())
        raw = observation.sensors.lidar
        compensated = observation.compensated_lidar
        assert raw is not None
        assert compensated is not None
        assert compensated is not raw
        assert compensated.motion_compensated is True
        assert compensated.t_compensated == pytest.approx(observation.t)
        assert compensated.t_measure == pytest.approx(raw.t_measure)
        assert observation.lidar_current is compensated
    finally:
        simulation.close()


def test_lidar_scan_carries_its_measurement_pose() -> None:
    """A scan must be anchored at the lidar pose of its sweep start."""

    simulation = Simulation(lidar_subrays=1, realism="ideal")
    try:
        simulation.reset()
        scan = simulation._latest_scan
        assert scan is not None
        state = simulation.world.state()
        mount = simulation.sensors.lidar.mount
        mount_xyz = np.asarray(mount_position(mount), dtype=float)
        yaw = state.yaw
        expected_origin = np.array([state.x, state.y, state.z]) + np.array(
            [
                math.cos(yaw) * mount_xyz[0] - math.sin(yaw) * mount_xyz[1],
                math.sin(yaw) * mount_xyz[0] + math.cos(yaw) * mount_xyz[1],
                mount_xyz[2],
            ]
        )
        assert scan.sensor_origin_world_m == pytest.approx(tuple(expected_origin))
        assert scan.sensor_yaw_world_rad == pytest.approx(
            yaw + math.radians(mount_rpy_deg(mount)[2])
        )

        # Reconstructing the returns with the scan's own pose must put most
        # points back on the wall pipes. The wall pipes have a 0.08 m radius;
        # using the car centre or the live yaw instead leaves a visible bias.
        valid = scan.valid & np.isfinite(scan.ranges)
        angles = scan.angles[valid]
        ranges = scan.ranges[valid]
        c, s = math.cos(scan.sensor_yaw_world_rad), math.sin(scan.sensor_yaw_world_rad)
        local = np.column_stack([np.cos(angles) * ranges, np.sin(angles) * ranges])
        points = np.column_stack(
            [
                scan.sensor_origin_world_m[0] + c * local[:, 0] - s * local[:, 1],
                scan.sensor_origin_world_m[1] + s * local[:, 0] + c * local[:, 1],
            ]
        )
        wall_points = np.vstack([simulation.track.inner, simulation.track.outer])
        distances = np.min(
            np.linalg.norm(points[:, None, :] - wall_points[None, :, :], axis=2),
            axis=1,
        )
        assert float(np.median(distances)) < 0.09
        assert float(np.percentile(distances, 90)) < 0.12
    finally:
        simulation.close()
