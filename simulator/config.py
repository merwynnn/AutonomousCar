"""Configuration models and YAML loading.

All tunable simulation constants live in ``configs/``; algorithms receive these
objects instead of hard-coded values.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

Realism = Literal["ideal", "realistic", "harsh"]


class StrictModel(BaseModel):
    """Base model that tolerates new keys while validating known ones."""

    model_config = ConfigDict(extra="allow", populate_by_name=True)


class Dimensions(StrictModel):
    length_m: float = Field(gt=0)
    width_m: float = Field(gt=0)
    height_m: float = Field(gt=0)
    ground_clearance_m: float = Field(ge=0)


class Limits(StrictModel):
    max_length_m: float = 0.45
    max_width_m: float = 0.20
    max_height_m: float = 0.20
    min_ground_clearance_m: float = 0.005


class Steering(StrictModel):
    max_angle_deg: float = Field(gt=0, le=45)
    rate_deg_s: float = Field(gt=0)
    deadband_deg: float = Field(ge=0)
    backlash_deg: float = Field(ge=0)
    latency_ms: float = Field(ge=0)


class Motor(StrictModel):
    max_torque_nm: float = Field(gt=0)
    max_rpm: float = Field(gt=0)
    gear_ratio: float = Field(gt=0)
    current_limit_a: float = Field(gt=0)
    torque_lag_ms: float = Field(ge=0)


class Battery(StrictModel):
    cells: int = Field(ge=1)
    nominal_voltage_v: float = Field(gt=0)
    capacity_mah: float = Field(gt=0)
    internal_resistance_ohm: float = Field(ge=0)


class Tires(StrictModel):
    friction_front: float = Field(gt=0)
    friction_rear: float = Field(gt=0)
    rolling_resistance: float = Field(ge=0)


class Suspension(StrictModel):
    stiffness: float = Field(gt=0)
    damping: float = Field(gt=0)
    max_travel_m: float = Field(gt=0)


class Aero(StrictModel):
    drag_coefficient: float = Field(ge=0)
    frontal_area_m2: float = Field(ge=0)


class VehicleConfig(StrictModel):
    name: str
    description: str = ""
    scale: float = 0.1
    dimensions: Dimensions
    mass_kg: float = Field(gt=0)
    center_of_mass_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    wheelbase_m: float = Field(gt=0)
    track_width_m: float = Field(gt=0)
    wheel_radius_m: float = Field(gt=0)
    drive: Literal["4wd", "2wd"] = "4wd"
    steering: Steering
    motor: Motor
    battery: Battery
    tires: Tires
    suspension: Suspension
    aero: Aero
    limits: Limits = Field(default_factory=Limits)

    @model_validator(mode="after")
    def check_competition_limits(self) -> VehicleConfig:
        d = self.dimensions
        lim = self.limits
        if d.length_m > lim.max_length_m:
            raise ValueError(f"length {d.length_m} m exceeds {lim.max_length_m} m")
        if d.width_m > lim.max_width_m:
            raise ValueError(f"width {d.width_m} m exceeds {lim.max_width_m} m")
        if d.height_m > lim.max_height_m:
            raise ValueError(f"height {d.height_m} m exceeds {lim.max_height_m} m")
        if d.ground_clearance_m < lim.min_ground_clearance_m:
            raise ValueError(
                f"ground clearance {d.ground_clearance_m} m below "
                f"{lim.min_ground_clearance_m} m"
            )
        return self


class LidarNoise(StrictModel):
    range_sigma_m: float = Field(ge=0)
    range_bias_m: float = 0.0
    range_sigma_per_m: float = Field(ge=0)
    angle_sigma_deg: float = Field(ge=0)
    angle_offset_deg: float = 0.0
    range_scale_error: float = 0.0


class LidarConfig(StrictModel):
    frame_id: str = "lidar"
    beams: int = Field(gt=0)
    rate_hz: float = Field(gt=0)
    min_range_m: float = Field(ge=0)
    max_range_m: float = Field(gt=0)
    angular_resolution_deg: float = Field(gt=0)
    scan_direction: Literal["clockwise", "counterclockwise"] = "clockwise"
    mount: dict[str, Any] = Field(default_factory=dict)
    noise: LidarNoise
    quantization: dict[str, float] = Field(default_factory=dict)
    divergence: dict[str, Any] = Field(default_factory=dict)
    dropout: dict[str, Any] = Field(default_factory=dict)
    dead_beams: dict[str, float] = Field(default_factory=dict)
    dropped_scan_probability: float = 0.0
    packet_loss_probability: float = 0.0
    latency_ms: float = Field(ge=0)
    latency_jitter_ms: float = 0.0
    temperature_drift_m_per_min: float = 0.0
    self_filter: bool = True
    wall_height_warning_m: float = 0.16


class UltrasonicSensor(StrictModel):
    name: str
    frame_id: str
    mount: dict[str, Any] = Field(default_factory=dict)


class UltrasonicConfig(StrictModel):
    rate_hz: float = Field(gt=0)
    cone_angle_deg: float = Field(gt=0)
    rays: int = Field(gt=0)
    min_range_m: float = Field(ge=0)
    max_range_m: float = Field(gt=0)
    range_sigma_m: float = Field(ge=0)
    range_sigma_ratio: float = Field(ge=0)
    latency_ms: float = Field(ge=0)
    cross_talk_probability: float = Field(ge=0, le=1)
    specular_dropout_probability: float = Field(ge=0, le=1)
    sensors: list[UltrasonicSensor]


class ImuConfig(StrictModel):
    frame_id: str = "imu"
    rate_hz: float = Field(gt=0)
    latency_ms: float = Field(ge=0)
    gyro: dict[str, float] = Field(default_factory=dict)
    accel: dict[str, float] = Field(default_factory=dict)
    mounting_misalignment_deg: float = 0.0


class EncoderConfig(StrictModel):
    rate_hz: float = Field(gt=0)
    ticks_per_rev: int = Field(gt=0)
    wheel_radius_error: float = 0.0
    track_width_error: float = 0.0
    timestamp_jitter_ms: float = 0.0
    dropout_probability: float = Field(ge=0, le=1)


class SensorsConfig(StrictModel):
    name: str
    description: str = ""
    lidar: LidarConfig
    ultrasonic: UltrasonicConfig
    imu: ImuConfig
    encoders: EncoderConfig
    realism_presets: dict[str, Any] = Field(default_factory=dict)


class OdometryConfig(StrictModel):
    speed_scale_error: float = 0.02
    yaw_scale_error: float = 0.01
    speed_bias_mps: float = 0.0
    yaw_bias_deg_s: float = 0.20
    yaw_bias_walk_deg_s_sqrt_s: float = 0.01
    speed_noise_mps: float = 0.02
    yaw_noise_rad_s: float = 0.005


class RaceConfig(StrictModel):
    laps: int = Field(default=5, ge=1)
    wall_deformation_impulse_threshold: float = Field(default=0.35, ge=0)


class SimulationConfig(StrictModel):
    physics_hz: float = 240.0
    control_hz: float = 60.0
    max_time_s: float = 240.0


class ScenarioConfig(StrictModel):
    name: str
    description: str = ""
    vehicle: str
    sensors: str
    seed: int = 0
    realism: Realism = "realistic"
    race: RaceConfig = Field(default_factory=RaceConfig)
    simulation: SimulationConfig = Field(default_factory=SimulationConfig)
    odometry: OdometryConfig = Field(default_factory=OdometryConfig)


def load_yaml(path: str | Path) -> dict[str, Any]:
    """Load a YAML file into a dictionary."""

    with Path(path).open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a YAML mapping at the top level")
    return data


def load_vehicle(path: str | Path) -> VehicleConfig:
    return VehicleConfig.model_validate(load_yaml(path))


def load_sensors(path: str | Path) -> SensorsConfig:
    return SensorsConfig.model_validate(load_yaml(path))


def load_scenario(path: str | Path) -> ScenarioConfig:
    return ScenarioConfig.model_validate(load_yaml(path))


@dataclass(frozen=True)
class ScenarioBundle:
    """A scenario plus its resolved vehicle and sensor configs."""

    scenario: ScenarioConfig
    vehicle: VehicleConfig
    sensors: SensorsConfig
    scenario_path: Path


def load_bundle(path: str | Path) -> ScenarioBundle:
    """Load a scenario and resolve vehicle/sensor paths next to it."""

    scenario_path = Path(path)
    scenario = load_scenario(scenario_path)
    root = scenario_path.parent
    vehicle = load_vehicle(root / scenario.vehicle)
    sensors = load_sensors(root / scenario.sensors)
    return ScenarioBundle(
        scenario=scenario,
        vehicle=vehicle,
        sensors=sensors,
        scenario_path=scenario_path,
    )
