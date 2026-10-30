from __future__ import annotations

from pathlib import Path

import pytest
from pydantic import ValidationError

from awake_sim.config import load_scenario, load_sensors, load_vehicle

ROOT = Path(__file__).resolve().parents[1]


def test_vehicle_within_competition_limits() -> None:
    vehicle = load_vehicle(ROOT / "configs" / "vehicles" / "generic_1_10_4wd.yaml")
    assert vehicle.scale == 0.1
    assert vehicle.drive == "4wd"
    assert vehicle.dimensions.length_m <= vehicle.limits.max_length_m
    assert vehicle.dimensions.width_m <= vehicle.limits.max_width_m
    assert vehicle.dimensions.height_m <= vehicle.limits.max_height_m
    assert vehicle.dimensions.ground_clearance_m >= vehicle.limits.min_ground_clearance_m


def test_vehicle_rejects_oversized_car(tmp_path: Path) -> None:
    src = ROOT / "configs" / "vehicles" / "generic_1_10_4wd.yaml"
    text = src.read_text(encoding="utf-8").replace("length_m: 0.40", "length_m: 0.60")
    bad = tmp_path / "bad.yaml"
    bad.write_text(text, encoding="utf-8")
    with pytest.raises(ValidationError):
        load_vehicle(bad)


def test_sensor_realism_defaults() -> None:
    sensors = load_sensors(ROOT / "configs" / "sensors" / "ld06.yaml")
    assert sensors.lidar.beams == 720
    assert sensors.lidar.noise.range_sigma_m > 0
    assert sensors.lidar.dropout["base_probability"] > 0
    assert sensors.lidar.latency_ms > 0
    assert len(sensors.ultrasonic.sensors) == 3
    assert sensors.imu.gyro["bias_deg_s"] > 0
    assert sensors.encoders.ticks_per_rev > 0
    assert set(sensors.realism_presets) == {"ideal", "realistic", "harsh"}


def test_scenario() -> None:
    scenario = load_scenario(ROOT / "configs" / "scenarios" / "test_track_baseline.yaml")
    assert scenario.realism == "realistic"
    assert scenario.race.laps == 5
    assert scenario.simulation.physics_hz == 240
    assert scenario.seed == 42
