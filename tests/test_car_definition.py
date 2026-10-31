from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from dashboard.spec import sensor_payload, vehicle_payload
from simulator.car_urdf import build_urdf
from simulator.config import BodyPart, load_sensors, load_vehicle

ROOT = Path(__file__).resolve().parents[1]


def _vehicle():
    return load_vehicle(ROOT / "simulator" / "configs" / "vehicle.yaml")


def _sensors():
    return load_sensors(ROOT / "simulator" / "configs" / "sensors.yaml")


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


def test_generated_urdf_matches_configuration() -> None:
    vehicle = _vehicle()
    root = ET.fromstring(build_urdf(vehicle))
    base = root.find("link[@name='base_link']")
    assert base is not None
    mass = base.find("./inertial/mass")
    assert mass is not None
    assert float(mass.attrib["value"]) == pytest.approx(vehicle.chassis_mass_kg)

    steer = root.find("joint[@name='fl_steer']")
    assert steer is not None
    origin = steer.find("origin")
    assert origin is not None
    xyz = [float(value) for value in origin.attrib["xyz"].split()]
    assert xyz[0] == pytest.approx(vehicle.wheelbase_m / 2.0)
    assert xyz[1] == pytest.approx(vehicle.track_width_m / 2.0)
    assert xyz[2] == pytest.approx(vehicle.wheel_offset_z_m)


def test_generated_urdf_tracks_code_changes() -> None:
    vehicle = _vehicle()
    vehicle.mass_kg = 2.5
    vehicle.wheelbase_m = 0.30
    vehicle.structure.append(
        BodyPart(
            name="roof_sensor_pod",
            shape="cylinder",
            position_m=(0.0, 0.0, 0.07),
            size_m=(0.04, 0.04, 0.02),
            color="#dddddd",
            collision=False,
        )
    )
    root = ET.fromstring(build_urdf(vehicle))
    assert root.find("link[@name='base_link']/visual") is not None
    assert len(root.findall("link[@name='base_link']/visual")) == len(vehicle.structure)
    steer = root.find("joint[@name='fl_steer']")
    assert steer is not None
    assert float(steer.find("origin").attrib["xyz"].split()[0]) == pytest.approx(0.15)


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
