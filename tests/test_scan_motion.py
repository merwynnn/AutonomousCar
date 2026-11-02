from __future__ import annotations

import math

import numpy as np
import pytest
from onboard.api import (
    LidarObservation,
    Observation,
    Pose2,
    SensorFrame,
    StateEstimate,
)
from onboard.scan_motion import ScanMotionCompensator


def _estimate(t: float, x: float, y: float, theta: float) -> StateEstimate:
    return StateEstimate(
        t=t,
        pose=Pose2(x, y, theta),
        speed_mps=0.0,
        yaw_rate_rad_s=0.0,
        covariance=np.eye(3),
        valid=True,
    )


def _scan(angle_rad: float, range_m: float, t: float = 0.0) -> LidarObservation:
    return LidarObservation(
        angles_rad=np.array([angle_rad], dtype=float),
        ranges_m=np.array([range_m], dtype=float),
        intensities=np.array([100.0], dtype=float),
        valid=np.array([True]),
        t_measure=t,
        t_available=t + 0.02,
    )


def test_compensation_removes_forward_motion_from_the_scan() -> None:
    compensator = ScanMotionCompensator((0.0, 0.0, 0.0), 0.0)
    start = _estimate(0.0, 0.0, 0.0, 0.0)
    now = _estimate(0.1, 0.5, 0.0, 0.0)
    compensator.update(start)
    compensator.update(now)

    scan = _scan(0.0, 2.0, t=0.0)
    compensated = compensator.compensate(scan, now)
    assert compensated is not None
    assert compensated.motion_compensated is True
    assert compensated.t_compensated == pytest.approx(0.1)
    assert compensated.age_s == pytest.approx(0.1)
    assert compensated.angles_rad[0] == pytest.approx(0.0)
    assert compensated.ranges_m[0] == pytest.approx(1.5)


def test_compensation_rotates_a_point_that_is_now_beside_the_car() -> None:
    compensator = ScanMotionCompensator((0.0, 0.0, 0.0), 0.0)
    start = _estimate(0.0, 0.0, 0.0, 0.0)
    now = _estimate(0.1, 0.0, 0.0, math.pi / 2.0)
    compensator.update(start)
    compensator.update(now)

    compensated = compensator.compensate(_scan(0.0, 2.0), now)
    assert compensated is not None
    # A wall that was straight ahead is now to the right of the car.
    assert compensated.angles_rad[0] == pytest.approx(-math.pi / 2.0)
    assert compensated.ranges_m[0] == pytest.approx(2.0)


def test_compensation_accounts_for_the_lidar_mount_lever_arm() -> None:
    compensator = ScanMotionCompensator((0.1, 0.0, 0.0), 0.0)
    compensator.update(_estimate(0.0, 0.0, 0.0, 0.0))
    now = _estimate(0.1, 0.0, 0.0, math.pi / 2.0)
    compensator.update(now)

    compensated = compensator.compensate(_scan(0.0, 2.0), now)
    assert compensated is not None
    assert compensated.angles_rad[0] == pytest.approx(
        math.atan2(-2.1, -0.1), abs=1e-9
    )
    assert compensated.ranges_m[0] == pytest.approx(math.hypot(0.1, 2.1))


def test_compensation_rejects_scans_without_covered_pose_history() -> None:
    compensator = ScanMotionCompensator((0.0, 0.0, 0.0), 0.0)
    now = _estimate(0.1, 0.0, 0.0, 0.0)
    compensator.update(now)
    assert compensator.compensate(_scan(0.0, 2.0, t=0.0), now) is None

    compensator = ScanMotionCompensator(
        (0.0, 0.0, 0.0), 0.0, max_scan_age_s=0.2
    )
    compensator.update(_estimate(0.0, 0.0, 0.0, 0.0))
    old_now = _estimate(0.5, 0.1, 0.0, 0.0)
    compensator.update(old_now)
    assert compensator.compensate(_scan(0.0, 2.0, t=0.0), old_now) is None


def test_invalid_beams_keep_their_nominal_angle() -> None:
    compensator = ScanMotionCompensator((0.0, 0.0, 0.0), 0.0)
    compensator.update(_estimate(0.0, 0.0, 0.0, 0.0))
    now = _estimate(0.1, 0.2, 0.0, 0.0)
    compensator.update(now)
    scan = LidarObservation(
        angles_rad=np.array([0.4, 1.0]),
        ranges_m=np.array([2.0, np.nan]),
        intensities=np.array([100.0, 0.0]),
        valid=np.array([True, False]),
        t_measure=0.0,
        t_available=0.02,
    )
    compensated = compensator.compensate(scan, now)
    assert compensated is not None
    assert not compensated.valid[1]
    assert compensated.angles_rad[1] == pytest.approx(1.0)
    assert np.isnan(compensated.ranges_m[1])


def test_lidar_current_prefers_the_compensated_scan() -> None:
    raw = _scan(0.0, 2.0, t=0.0)
    compensated = _scan(0.1, 1.8, t=0.0)
    compensated = LidarObservation(
        angles_rad=compensated.angles_rad,
        ranges_m=compensated.ranges_m,
        intensities=compensated.intensities,
        valid=compensated.valid,
        t_measure=compensated.t_measure,
        t_available=compensated.t_available,
        motion_compensated=True,
        t_compensated=0.1,
    )
    state = _estimate(0.1, 0.0, 0.0, 0.0)
    sensors = SensorFrame(lidar=raw)
    observation = Observation(
        t=0.1,
        dt=0.01,
        sensors=sensors,
        state=state,
        compensated_lidar=compensated,
    )
    assert observation.lidar_current is compensated
    assert observation.lidar is raw

    raw_only = Observation(t=0.1, dt=0.01, sensors=sensors, state=state)
    assert raw_only.lidar_current is raw
