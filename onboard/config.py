"""Portable onboard configuration loading.

The estimator tuning lives in ``configs/estimator.yaml``. Vehicle geometry and
encoder resolution stay in the shared car/sensor YAML so the simulator and the
Raspberry Pi read exactly the same numbers. This module intentionally uses only
PyYAML and dataclasses; it does not import pydantic or any simulator code.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from onboard.actuation import HardwareConfig


@dataclass(frozen=True)
class EstimatorTuning:
    """Gains and thresholds for the shared wheel + IMU estimator."""

    name: str = "wheel_imu_default"
    gyro_weight: float = 0.85
    gyro_bias_gain: float = 0.5
    gyro_bias_tracking_gain: float = 0.02
    max_gyro_bias_rad_s: float = 0.2
    slip_yaw_threshold_rad_s: float = 0.9
    slip_speed_threshold_mps: float = 0.12
    zero_velocity_speed_mps: float = 0.03
    zero_velocity_yaw_rate_rad_s: float = 0.08
    position_noise_mps: float = 0.01
    yaw_noise_rad_s: float = 0.005

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> EstimatorTuning:
        return cls(
            name=str(data.get("name", cls.name)),
            gyro_weight=float(data.get("gyro_weight", cls.gyro_weight)),
            gyro_bias_gain=float(data.get("gyro_bias_gain", cls.gyro_bias_gain)),
            gyro_bias_tracking_gain=float(
                data.get("gyro_bias_tracking_gain", cls.gyro_bias_tracking_gain)
            ),
            max_gyro_bias_rad_s=float(
                data.get("max_gyro_bias_rad_s", cls.max_gyro_bias_rad_s)
            ),
            slip_yaw_threshold_rad_s=float(
                data.get("slip_yaw_threshold_rad_s", cls.slip_yaw_threshold_rad_s)
            ),
            slip_speed_threshold_mps=float(
                data.get("slip_speed_threshold_mps", cls.slip_speed_threshold_mps)
            ),
            zero_velocity_speed_mps=float(
                data.get("zero_velocity_speed_mps", cls.zero_velocity_speed_mps)
            ),
            zero_velocity_yaw_rate_rad_s=float(
                data.get(
                    "zero_velocity_yaw_rate_rad_s",
                    cls.zero_velocity_yaw_rate_rad_s,
                )
            ),
            position_noise_mps=float(
                data.get("position_noise_mps", cls.position_noise_mps)
            ),
            yaw_noise_rad_s=float(data.get("yaw_noise_rad_s", cls.yaw_noise_rad_s)),
        )


@dataclass(frozen=True)
class OnboardConfig:
    """Everything the portable runtime needs before constructing an estimator."""

    vehicle_name: str
    wheel_radius_m: float
    track_width_m: float
    wheelbase_m: float
    max_steering_rad: float
    max_steering_rate_rad_s: float
    ticks_per_rev: int
    encoder_rate_hz: float
    tuning: EstimatorTuning
    lidar_mount_position_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    lidar_mount_yaw_rad: float = 0.0
    hardware: HardwareConfig = field(default_factory=HardwareConfig)


def load_yaml_mapping(path: str | Path) -> dict[str, Any]:
    """Load a YAML mapping without importing simulator config models."""

    with Path(path).open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a YAML mapping at the top level")
    return data


def load_estimator_tuning(path: str | Path) -> EstimatorTuning:
    return EstimatorTuning.from_mapping(load_yaml_mapping(path))


def load_hardware_config(path: str | Path) -> HardwareConfig:
    return HardwareConfig.from_mapping(load_yaml_mapping(path))


def _require_mapping(data: dict[str, Any], key: str, source: Path) -> dict[str, Any]:
    value = data.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"{source}: expected a '{key}' mapping")
    return value


def _require_number(data: dict[str, Any], key: str, source: Path) -> float:
    value = data.get(key)
    if not isinstance(value, int | float):
        raise ValueError(f"{source}: missing numeric '{key}'")
    return float(value)


def _require_vector3(
    data: dict[str, Any], key: str, source: Path
) -> tuple[float, float, float]:
    value = data.get(key, [0.0, 0.0, 0.0])
    if (
        not isinstance(value, list)
        or len(value) != 3
        or not all(isinstance(item, int | float) for item in value)
    ):
        raise ValueError(f"{source}: expected a 3-number '{key}' list")
    return (float(value[0]), float(value[1]), float(value[2]))


def _require_mount(
    sensors: dict[str, Any], source: Path
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    lidar = _require_mapping(sensors, "lidar", source)
    mount = lidar.get("mount", {})
    if not isinstance(mount, dict):
        raise ValueError(f"{source}: expected a 'lidar.mount' mapping")
    return (
        _require_vector3(mount, "position_m", source),
        _require_vector3(mount, "rpy_deg", source),
    )


def load_onboard_config(
    vehicle_path: str | Path,
    sensors_path: str | Path,
    estimator_path: str | Path,
    hardware_path: str | Path | None = None,
) -> OnboardConfig:
    """Resolve shared car/sensor geometry, estimator tuning and actuators."""

    vehicle_source = Path(vehicle_path)
    sensors_source = Path(sensors_path)
    hardware_source = (
        Path(hardware_path)
        if hardware_path is not None
        else vehicle_source.parent / "hardware.yaml"
    )
    vehicle = load_yaml_mapping(vehicle_source)
    sensors = load_yaml_mapping(sensors_source)
    steering = _require_mapping(vehicle, "steering", vehicle_source)
    encoders = _require_mapping(sensors, "encoders", sensors_source)
    ticks = encoders.get("ticks_per_rev")
    if not isinstance(ticks, int) or ticks <= 0:
        raise ValueError(f"{sensors_source}: encoders.ticks_per_rev must be a positive int")
    hardware = (
        load_hardware_config(hardware_source)
        if hardware_source.exists()
        else HardwareConfig()
    )
    lidar_position, lidar_rpy = _require_mount(sensors, sensors_source)
    return OnboardConfig(
        vehicle_name=str(vehicle.get("name", "unnamed_vehicle")),
        wheel_radius_m=_require_number(vehicle, "wheel_radius_m", vehicle_source),
        track_width_m=_require_number(vehicle, "track_width_m", vehicle_source),
        wheelbase_m=_require_number(vehicle, "wheelbase_m", vehicle_source),
        max_steering_rad=math.radians(
            _require_number(steering, "max_angle_deg", vehicle_source)
        ),
        max_steering_rate_rad_s=math.radians(
            _require_number(steering, "rate_deg_s", vehicle_source)
        ),
        ticks_per_rev=ticks,
        encoder_rate_hz=float(encoders.get("rate_hz", 50.0)),
        lidar_mount_position_m=lidar_position,
        lidar_mount_yaw_rad=math.radians(lidar_rpy[2]),
        tuning=load_estimator_tuning(estimator_path),
        hardware=hardware,
    )


__all__ = [
    "EstimatorTuning",
    "OnboardConfig",
    "load_estimator_tuning",
    "load_hardware_config",
    "load_onboard_config",
    "load_yaml_mapping",
]
