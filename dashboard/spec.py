"""JSON payloads for the dashboard.

Everything the browser needs to draw the car is derived from the same
``VehicleConfig`` and ``SensorsConfig`` that drive PyBullet. There is no second
hand-written car model in the frontend.
"""

from __future__ import annotations

from typing import Any

from simulator.config import (
    SensorsConfig,
    VehicleConfig,
    mount_position,
    mount_rpy_deg,
)
from simulator.track import TrackGeometry


def _round(values: tuple[float, ...], digits: int = 4) -> list[float]:
    return [round(float(value), digits) for value in values]


def vehicle_payload(vehicle: VehicleConfig) -> dict[str, Any]:
    parts = []
    for part in vehicle.resolved_structure():
        parts.append(
            {
                "name": part.name,
                "shape": part.shape,
                "position_m": _round(part.position_m),
                "rpy_deg": _round(part.rpy_deg),
                "size_m": _round(part.size_m),
                "color": part.color,
                "opacity": part.opacity,
                "visible": part.visible,
                "collision": part.collision,
            }
        )

    half_wheelbase = vehicle.wheelbase_m / 2.0
    half_track = vehicle.track_width_m / 2.0
    wheels = []
    for name, front, side in (
        ("front_left", True, half_track),
        ("front_right", True, -half_track),
        ("rear_left", False, half_track),
        ("rear_right", False, -half_track),
    ):
        wheels.append(
            {
                "name": name,
                "position_m": _round(
                    (
                        half_wheelbase if front else -half_wheelbase,
                        side,
                        vehicle.wheel_offset_z_m,
                    )
                ),
                "radius_m": vehicle.wheel_radius_m,
                "width_m": vehicle.wheel_width_m,
                "mass_kg": vehicle.wheel_mass_kg,
                "steered": front,
                "driven": vehicle.drive == "4wd" or not front,
            }
        )

    return {
        "name": vehicle.name,
        "description": vehicle.description,
        "scale": vehicle.scale,
        "dimensions": vehicle.dimensions.model_dump(mode="json"),
        "mass_kg": vehicle.mass_kg,
        "chassis_mass_kg": round(vehicle.chassis_mass_kg, 4),
        "center_of_mass_m": _round(vehicle.center_of_mass_m),
        "wheelbase_m": vehicle.wheelbase_m,
        "track_width_m": vehicle.track_width_m,
        "wheel_radius_m": vehicle.wheel_radius_m,
        "wheel_width_m": vehicle.wheel_width_m,
        "wheel_mass_kg": vehicle.wheel_mass_kg,
        "wheel_offset_z_m": vehicle.wheel_offset_z_m,
        "ride_height_m": round(vehicle.ride_height_m, 4),
        "drive": vehicle.drive,
        "limits": vehicle.limits.model_dump(mode="json"),
        "steering": vehicle.steering.model_dump(mode="json"),
        "motor": vehicle.motor.model_dump(mode="json"),
        "battery": vehicle.battery.model_dump(mode="json"),
        "tires": vehicle.tires.model_dump(mode="json"),
        "suspension": vehicle.suspension.model_dump(mode="json"),
        "aero": vehicle.aero.model_dump(mode="json"),
        "structure": parts,
        "wheels": wheels,
    }


def sensor_payload(sensors: SensorsConfig, vehicle: VehicleConfig) -> dict[str, Any]:
    lidar = sensors.lidar
    sonars = sensors.ultrasonic
    imu = sensors.imu
    mounts: list[dict[str, Any]] = [
        {
            "id": "lidar",
            "kind": "lidar",
            "name": lidar.frame_id,
            "frame_id": lidar.frame_id,
            "position_m": _round(mount_position(lidar.mount)),
            "rpy_deg": _round(mount_rpy_deg(lidar.mount)),
            "ground_height_m": round(
                vehicle.ride_height_m + mount_position(lidar.mount)[2], 4
            ),
            "range_m": lidar.max_range_m,
            "fov_deg": 360.0,
            "color": "#d7a84b",
            "summary": f"{lidar.beams} beams · {lidar.rate_hz:g} Hz",
            "characteristics": lidar.model_dump(mode="json"),
        }
    ]
    for sensor in sonars.sensors:
        position = mount_position(sensor.mount)
        mounts.append(
            {
                "id": sensor.name,
                "kind": "ultrasonic",
                "name": sensor.name,
                "frame_id": sensor.frame_id,
                "position_m": _round(position),
                "rpy_deg": _round(mount_rpy_deg(sensor.mount)),
                "ground_height_m": round(vehicle.ride_height_m + position[2], 4),
                "range_m": sonars.max_range_m,
                "fov_deg": sonars.cone_angle_deg,
                "color": "#5f9fc7",
                "summary": (
                    f"{sonars.cone_angle_deg:g}° cone · "
                    f"{sonars.rate_hz:g} Hz · {sonars.max_range_m:g} m"
                ),
                "characteristics": sonars.model_dump(mode="json"),
            }
        )
    imu_position = mount_position(imu.mount)
    mounts.append(
        {
            "id": "imu",
            "kind": "imu",
            "name": imu.frame_id,
            "frame_id": imu.frame_id,
            "position_m": _round(imu_position),
            "rpy_deg": _round(mount_rpy_deg(imu.mount)),
            "ground_height_m": round(vehicle.ride_height_m + imu_position[2], 4),
            "range_m": 0.0,
            "fov_deg": 0.0,
            "color": "#9a86c7",
            "summary": f"{imu.rate_hz:g} Hz · {imu.latency_ms:g} ms latency",
            "characteristics": imu.model_dump(mode="json"),
        }
    )
    return {
        "name": sensors.name,
        "description": sensors.description,
        "lidar": lidar.model_dump(mode="json"),
        "ultrasonic": sonars.model_dump(mode="json"),
        "imu": imu.model_dump(mode="json"),
        "encoders": sensors.encoders.model_dump(mode="json"),
        "mounts": mounts,
    }


def track_payload(track: TrackGeometry) -> dict[str, Any]:
    return {
        "name": "test_track",
        "width_m": track.width_m,
        "bounds": list(track.bounds),
        "centerline": [
            [round(float(x), 4), round(float(y), 4)] for x, y in track.centerline
        ],
        "inner": [[round(float(x), 4), round(float(y), 4)] for x, y in track.inner],
        "outer": [[round(float(x), 4), round(float(y), 4)] for x, y in track.outer],
        "wall_height_m": 0.16,
        "wall_radius_m": 0.08,
        "start_distance_m": track.start_s,
        "track_length_m": round(track.length, 3),
        "obstacles": [
            {
                "name": obstacle.name,
                "kind": obstacle.kind,
                "x": obstacle.x,
                "y": obstacle.y,
                "yaw": obstacle.yaw,
                "length": obstacle.length,
                "width": obstacle.width,
                "height": obstacle.height,
                "radius": obstacle.radius,
            }
            for obstacle in track.obstacles
        ],
    }
