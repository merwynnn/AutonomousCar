from __future__ import annotations

import re
from pathlib import Path

import numpy as np
from algorithms import interface as interface
from onboard import api
from onboard.api import (
    EncoderSample,
    ImuSample,
    Observation,
    Pose2,
    SensorFrame,
    StateEstimate,
    SteeringSample,
    UltrasonicObservation,
)

ROOT = Path(__file__).resolve().parents[1]
PORTABLE_PACKAGES = ("onboard", "algorithms")
FORBIDDEN_ROOTS = ("simulator", "dashboard", "pybullet", "mujoco", "fastapi")


def _imports(path: Path) -> list[str]:
    source = path.read_text(encoding="utf-8")
    return re.findall(r"^\s*(?:from|import)\s+([A-Za-z_][\w.]*)", source, re.MULTILINE)


def test_portable_packages_do_not_import_host_code() -> None:
    for package in PORTABLE_PACKAGES:
        for path in (ROOT / package).rglob("*.py"):
            for module in _imports(path):
                root = module.split(".", 1)[0]
                assert root not in FORBIDDEN_ROOTS, f"{path} imports {module}"


def test_interface_reexports_the_portable_contract() -> None:
    assert interface.Observation is api.Observation
    assert interface.Actuation is api.Actuation
    assert interface.RobotAPI is api.RobotAPI
    assert interface.SensorFrame is api.SensorFrame
    assert interface.StateEstimate is api.StateEstimate


def test_observation_exposes_raw_and_fused_data() -> None:
    imu = ImuSample((0.0, 0.0, 0.1), (0.0, 0.0, 9.81), 1.0, 1.01)
    encoders = EncoderSample((0, 0, 10, 10), None, 1.0, 1.01, 0.02)
    steering = SteeringSample(0.05, 1.0, 1.01)
    sonar = UltrasonicObservation("front", 0.8, True, 1.0, 1.02)
    sensors = SensorFrame(
        imu=imu,
        encoders=encoders,
        steering=steering,
        ultrasonic=(sonar,),
    )
    estimate = StateEstimate(
        t=1.0,
        pose=Pose2(0.2, 0.1, 0.05),
        speed_mps=0.7,
        yaw_rate_rad_s=0.1,
        covariance=np.eye(3),
        valid=True,
    )
    observation = Observation(t=1.0, dt=0.01, sensors=sensors, state=estimate)
    assert observation.imu is imu
    assert observation.encoders is encoders
    assert observation.steering is steering
    assert observation.ultrasonic == (sonar,)
    assert observation.odometry == estimate.pose
    assert observation.speed_mps == estimate.speed_mps
