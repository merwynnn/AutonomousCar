"""Portable onboard configuration loading.

The estimator tuning lives in ``configs/estimator.yaml``. Vehicle geometry and
encoder resolution stay in the shared car/sensor YAML so the simulator and the
Raspberry Pi read exactly the same numbers. This module intentionally uses only
PyYAML and dataclasses; it does not import pydantic or any simulator code.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


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
    ticks_per_rev: int
    encoder_rate_hz: float
    tuning: EstimatorTuning


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


def load_onboard_config(
    vehicle_path: str | Path,
    sensors_path: str | Path,
    estimator_path: str | Path,
) -> OnboardConfig:
    """Resolve shared car/sensor geometry and estimator tuning for the Pi."""

    vehicle_source = Path(vehicle_path)
    sensors_source = Path(sensors_path)
    vehicle = load_yaml_mapping(vehicle_source)
    sensors = load_yaml_mapping(sensors_source)
    encoders = _require_mapping(sensors, "encoders", sensors_source)
    ticks = encoders.get("ticks_per_rev")
    if not isinstance(ticks, int) or ticks <= 0:
        raise ValueError(f"{sensors_source}: encoders.ticks_per_rev must be a positive int")
    return OnboardConfig(
        vehicle_name=str(vehicle.get("name", "unnamed_vehicle")),
        wheel_radius_m=_require_number(vehicle, "wheel_radius_m", vehicle_source),
        track_width_m=_require_number(vehicle, "track_width_m", vehicle_source),
        wheelbase_m=_require_number(vehicle, "wheelbase_m", vehicle_source),
        ticks_per_rev=ticks,
        encoder_rate_hz=float(encoders.get("rate_hz", 50.0)),
        tuning=load_estimator_tuning(estimator_path),
    )


__all__ = [
    "EstimatorTuning",
    "OnboardConfig",
    "load_estimator_tuning",
    "load_onboard_config",
    "load_yaml_mapping",
]
