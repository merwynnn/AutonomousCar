from __future__ import annotations

import math
from pathlib import Path

import pytest
from onboard.actuation import (
    DriveActuator,
    DriveCalibration,
    HardwareConfig,
    SteeringActuator,
    SteeringCalibration,
)
from onboard.config import load_hardware_config, load_onboard_config
from simulator.config import load_bundle
from simulator.simulation import DEFAULT_SCENARIO

ROOT = Path(__file__).resolve().parents[1]
HARDWARE = ROOT / "configs" / "hardware.yaml"


def test_hardware_config_is_shared_by_simulator_and_onboard() -> None:
    from_file = load_hardware_config(HARDWARE)
    from_scenario = load_bundle(DEFAULT_SCENARIO).hardware
    from_onboard = load_onboard_config(
        ROOT / "configs" / "vehicle_awake2027.yaml",
        ROOT / "configs" / "sensors_awake2027.yaml",
        ROOT / "configs" / "estimator.yaml",
    ).hardware

    assert from_file == from_scenario == from_onboard
    assert from_file.protocol_version == 2
    assert from_file.calibration_status == "uncalibrated"


def test_onboard_config_loads_the_lidar_mount_for_compensation() -> None:
    config = load_onboard_config(
        ROOT / "configs" / "vehicle_awake2027.yaml",
        ROOT / "configs" / "sensors_awake2027.yaml",
        ROOT / "configs" / "estimator.yaml",
    )
    assert config.lidar_mount_position_m == pytest.approx((0.105, 0.0, 0.05))
    assert config.lidar_mount_yaw_rad == pytest.approx(0.0)


def test_steering_servo_mapping_round_trips_with_sign_and_trim() -> None:
    calibration = SteeringCalibration(
        center_us=1500.0,
        us_per_deg=12.0,
        sign=-1.0,
        trim_deg=-2.0,
        min_us=900.0,
        max_us=2100.0,
    )
    angle = math.radians(12.0)
    pulse = calibration.angle_to_servo_us(angle)
    assert pulse == pytest.approx(
        1500.0 - (12.0 + 2.0) * 12.0
    )
    assert calibration.servo_us_to_angle(pulse) == pytest.approx(angle)


def test_steering_rate_limit_and_deadband() -> None:
    calibration = SteeringCalibration(
        max_rate_deg_s=90.0,
        deadband_deg=0.0,
        backlash_deg=0.0,
        latency_ms=0.0,
    )
    actuator = SteeringActuator(calibration, max_angle_rad=math.radians(28.0))
    output = actuator.update(math.radians(20.0), dt=0.01)
    assert output == pytest.approx(math.radians(0.9), abs=1e-9)

    deadband = SteeringCalibration(
        max_rate_deg_s=90.0,
        deadband_deg=1.0,
        backlash_deg=0.0,
        latency_ms=0.0,
    )
    actuator = SteeringActuator(deadband, max_angle_rad=math.radians(28.0))
    assert actuator.update(0.0, dt=0.01) == pytest.approx(0.0)
    assert actuator.update(math.radians(0.5), dt=0.01) == pytest.approx(0.0)


def test_steering_latency_delays_the_output() -> None:
    calibration = SteeringCalibration(
        max_rate_deg_s=1000.0,
        deadband_deg=0.0,
        backlash_deg=0.0,
        latency_ms=20.0,
    )
    actuator = SteeringActuator(calibration, max_angle_rad=math.radians(28.0))
    assert actuator.update(0.2, dt=0.01) == pytest.approx(0.0)
    assert actuator.update(0.2, dt=0.01) == pytest.approx(0.0)
    assert actuator.update(0.2, dt=0.01) > 0.0


def test_backlash_holds_output_until_play_is_taken_up() -> None:
    calibration = SteeringCalibration(
        max_rate_deg_s=1000.0,
        deadband_deg=0.0,
        backlash_deg=1.0,
        latency_ms=0.0,
    )
    actuator = SteeringActuator(calibration, max_angle_rad=math.radians(28.0))
    first = actuator.update(0.0, dt=0.01)
    assert first == pytest.approx(0.0)
    moved = actuator.update(math.radians(5.0), dt=0.01)
    assert moved == pytest.approx(math.radians(4.0), abs=1e-9)
    held = actuator.update(math.radians(4.5), dt=0.01)
    assert held == pytest.approx(moved, abs=1e-9)
    resumed = actuator.update(math.radians(2.0), dt=0.01)
    assert resumed < moved


def test_drive_throttle_is_a_speed_request_with_deadband_and_sign() -> None:
    calibration = DriveCalibration(
        max_speed_mps=4.0,
        throttle_deadband=0.05,
        sign=-1.0,
    )
    actuator = DriveActuator(calibration, wheel_radius_m=0.035)
    assert actuator.target_speed_mps(0.02) == 0.0
    assert actuator.target_speed_mps(0.5) == pytest.approx(-2.0)


def test_drive_torque_is_limited_and_lagged() -> None:
    calibration = DriveCalibration(
        max_speed_mps=4.0,
        current_limit_a=40.0,
        torque_lag_ms=12.0,
        speed_kp=0.35,
        speed_ki=0.15,
    )
    actuator = DriveActuator(calibration, wheel_radius_m=0.035)
    torque = actuator.update(
        1.0,
        measured_wheel_speed_rad_s=0.0,
        max_torque_per_wheel_nm=0.2,
        battery_voltage_v=7.4,
        battery_resistance_ohm=0.035,
        dt=0.01,
    )
    assert 0.0 < torque <= 0.2
    reverse = actuator.update(
        -1.0,
        measured_wheel_speed_rad_s=0.0,
        max_torque_per_wheel_nm=0.2,
        battery_voltage_v=7.4,
        battery_resistance_ohm=0.035,
        dt=0.01,
    )
    assert reverse < 0.0


def test_hardware_config_rejects_invalid_sign() -> None:
    with pytest.raises(ValueError):
        HardwareConfig.from_mapping({"steering": {"sign": 0}})
