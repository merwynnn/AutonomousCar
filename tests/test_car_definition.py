from __future__ import annotations

from pathlib import Path

import pytest
from dashboard.spec import sensor_payload, vehicle_payload
from simulator.config import load_sensors, load_vehicle

ROOT = Path(__file__).resolve().parents[1]


def _vehicle():
    return load_vehicle(ROOT / "configs" / "vehicle.yaml")


def _sensors():
    return load_sensors(ROOT / "configs" / "sensors.yaml")


def test_vehicle_structure_and_derived_geometry() -> None:
    vehicle = _vehicle()
    assert vehicle.structure
    assert vehicle.resolved_structure()[0].collision
    assert vehicle.chassis_mass_kg == pytest.approx(
        vehicle.mass_kg - 4.0 * vehicle.wheel_mass_kg
    )
    assert vehicle.ride_height_m == pytest.approx(0.09)
    assert vehicle.wheel_width_m > 0


def test_mounts_are_typed_and_in_body_frame() -> None:
    sensors = _sensors()
    assert sensors.lidar.mount.position_m == (0.10, 0.0, 0.05)
    assert sensors.ultrasonic.sensors[0].mount.rpy_deg[2] == 15.0
    assert sensors.imu.mount.position_m[2] == 0.01


def test_dashboard_payload_uses_same_definition() -> None:
    vehicle = _vehicle()
    sensors = _sensors()
    payload = vehicle_payload(vehicle)
    sensor_payload_data = sensor_payload(sensors, vehicle)
    assert payload["mass_kg"] == vehicle.mass_kg
    assert len(payload["structure"]) == len(vehicle.structure)
    assert len(payload["wheels"]) == 4
    assert payload["ride_height_m"] == pytest.approx(vehicle.ride_height_m, abs=1e-4)
    assert {mount["id"] for mount in sensor_payload_data["mounts"]} == {
        "lidar",
        "front_left",
        "front_right",
        "rear_center",
        "imu",
    }
