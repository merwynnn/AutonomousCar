"""Generate a MuJoCo MJCF model from the shared vehicle configuration.

The MJCF is intentionally a physics-only model: the browser dashboard renders
from ``VehicleConfig``. This keeps one source of truth and avoids a second visual
car definition drifting from the simulated one.
"""

from __future__ import annotations

import math
import re
from pathlib import Path
from xml.sax.saxutils import escape

from simulator.config import BodyPart, VehicleConfig
from simulator.track import TrackGeometry

_WALL_HEIGHT_M = 0.16
_WALL_RADIUS_M = 0.08
_GROUP_CAR = 1


def _safe_name(name: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", name.strip())
    return cleaned or "part"


def _fmt(values: tuple[float, ...] | list[float]) -> str:
    return " ".join(f"{float(value):.6g}" for value in values)


def _radians(values: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(math.radians(value) for value in values)  # type: ignore[return-value]


def _hex_rgba(color: str, opacity: float = 1.0) -> str:
    value = color.strip().lstrip("#")
    if len(value) == 3:
        value = "".join(char * 2 for char in value)
    if len(value) != 6:
        value = "a53235"
    return (
        f"{int(value[0:2], 16) / 255.0:.4f} "
        f"{int(value[2:4], 16) / 255.0:.4f} "
        f"{int(value[4:6], 16) / 255.0:.4f} {opacity:.3f}"
    )


def _collision_parts(vehicle: VehicleConfig) -> list[BodyPart]:
    parts = [part for part in vehicle.resolved_structure() if part.collision]
    return parts or vehicle.resolved_structure()


def _bounds(vehicle: VehicleConfig) -> tuple[float, float, float]:
    parts = _collision_parts(vehicle)
    min_x = min(part.position_m[0] - part.size_m[0] / 2.0 for part in parts)
    max_x = max(part.position_m[0] + part.size_m[0] / 2.0 for part in parts)
    min_y = min(part.position_m[1] - part.size_m[1] / 2.0 for part in parts)
    max_y = max(part.position_m[1] + part.size_m[1] / 2.0 for part in parts)
    min_z = min(part.position_m[2] - part.size_m[2] / 2.0 for part in parts)
    max_z = max(part.position_m[2] + part.size_m[2] / 2.0 for part in parts)
    return (
        max(0.01, max_x - min_x),
        max(0.01, max_y - min_y),
        max(0.01, max_z - min_z),
    )


def _box_inertia(mass: float, size: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = size
    return (
        mass * (y * y + z * z) / 12.0,
        mass * (x * x + z * z) / 12.0,
        mass * (x * x + y * y) / 12.0,
    )


def _wheel_inertia(vehicle: VehicleConfig) -> tuple[float, float]:
    radius = vehicle.wheel_radius_m
    width = vehicle.wheel_width_m
    mass = vehicle.wheel_mass_kg
    return (
        mass * (3.0 * radius * radius + width * width) / 12.0,
        mass * radius * radius / 2.0,
    )


def _geom_xml(part: BodyPart) -> str:
    size_x, size_y, size_z = part.size_m
    common = (
        f'name="{_safe_name(part.name)}_geom" group="{_GROUP_CAR}" '
        f'pos="{_fmt(part.position_m)}" euler="{_fmt(_radians(part.rpy_deg))}" '
        f'rgba="{_hex_rgba(part.color, part.opacity)}" '
        f'contype="2" conaffinity="1"'
    )
    if part.shape == "cylinder":
        return (
            f'<geom {common} type="cylinder" '
            f'size="{size_x / 2.0:.6g} {size_z / 2.0:.6g}" '
            f'friction="0.8 0.01 0.001"/>'
        )
    if part.shape == "sphere":
        return f'<geom {common} type="sphere" size="{size_x / 2.0:.6g}"/>'
    return (
        f'<geom {common} type="box" '
        f'size="{size_x / 2.0:.6g} {size_y / 2.0:.6g} {size_z / 2.0:.6g}" '
        f'friction="0.7 0.01 0.001"/>'
    )


def _wheel_body_xml(
    name: str,
    vehicle: VehicleConfig,
    position: tuple[float, float, float],
    friction: float,
) -> str:
    radius = vehicle.wheel_radius_m
    width = vehicle.wheel_width_m
    mass = vehicle.wheel_mass_kg
    ixx, iyy = _wheel_inertia(vehicle)
    return f"""
      <body name="{name}" pos="{_fmt(position)}">
        <joint name="{name}_spin" type="hinge" axis="0 1 0"
               damping="{max(0.001, vehicle.tires.rolling_resistance * 0.1):.6g}"
               armature="0.0002"/>
        <inertial pos="0 0 0" mass="{mass:.6g}" diaginertia="{ixx:.6g} {iyy:.6g} {ixx:.6g}"/>
        <geom name="{name}_geom" type="cylinder" size="{radius:.6g} {width / 2.0:.6g}"
              euler="1.570796 0 0" group="{_GROUP_CAR}" contype="4" conaffinity="1"
              friction="{max(0.5, friction):.5g} 0.01 0.0005" condim="6"
              rgba="0.05 0.05 0.06 1"/>
      </body>"""


def _car_body_xml(vehicle: VehicleConfig) -> str:
    parts = _collision_parts(vehicle)
    bounds = _bounds(vehicle)
    ixx, iyy, izz = _box_inertia(vehicle.chassis_mass_kg, bounds)
    body_geoms = "\n        ".join(_geom_xml(part) for part in parts)
    half_wheelbase = vehicle.wheelbase_m / 2.0
    half_track = vehicle.track_width_m / 2.0
    wheel_z = vehicle.wheel_offset_z_m
    steer_limit = math.radians(vehicle.steering.max_angle_deg)

    front = ""
    for side, suffix in ((half_track, "l"), (-half_track, "r")):
        knuckle = f"f{suffix}_knuckle"
        wheel = f"f{suffix}_wheel"
        steer = f"f{suffix}_steer"
        front += f"""
      <body name="{knuckle}" pos="{half_wheelbase:.6g} {side:.6g} {wheel_z:.6g}">
        <joint name="{steer}" type="hinge" axis="0 0 1"
               range="{-steer_limit:.6g} {steer_limit:.6g}"
               damping="0.08" armature="0.0002"/>
        <inertial pos="0 0 0" mass="0.02" diaginertia="1e-6 1e-6 1e-6"/>
        {_wheel_body_xml(wheel, vehicle, (0.0, 0.0, 0.0), vehicle.tires.friction_front)}
      </body>"""

    rear = ""
    for side, suffix in ((half_track, "l"), (-half_track, "r")):
        wheel = f"r{suffix}_wheel"
        rear += _wheel_body_xml(
            wheel,
            vehicle,
            (-half_wheelbase, side, wheel_z),
            vehicle.tires.friction_rear,
        )

    return f"""
    <body name="chassis" pos="0 0 {vehicle.ride_height_m:.6g}">
      <freejoint name="chassis_free"/>
      <inertial pos="{_fmt(vehicle.center_of_mass_m)}" mass="{vehicle.chassis_mass_kg:.6g}"
                diaginertia="{ixx:.6g} {iyy:.6g} {izz:.6g}"/>
      {body_geoms}
      {front}
      {rear}
    </body>"""


def _actuator_xml(vehicle: VehicleConfig) -> str:
    steer_limit = math.radians(vehicle.steering.max_angle_deg)
    steer_kp = max(1.0, vehicle.motor.max_torque_nm * vehicle.motor.gear_ratio * 4.0)
    wheel_torque = max(
        0.1, vehicle.motor.max_torque_nm * vehicle.motor.gear_ratio / 4.0
    )
    wheel_names = ["fl_wheel_spin", "fr_wheel_spin", "rl_wheel_spin", "rr_wheel_spin"]
    driven = wheel_names if vehicle.drive == "4wd" else wheel_names[2:]
    rows: list[str] = []
    for name in wheel_names:
        limit = wheel_torque if name in driven else 0.0
        rows.append(
            f'    <motor name="{name}_motor" joint="{name}" gear="1" '
            f'ctrlrange="{-limit:.6g} {limit:.6g}"/>'
        )
    rows.append(
        f'    <position name="fl_steer_servo" joint="fl_steer" kp="{steer_kp:.6g}" kv="0.4" '
        f'ctrlrange="{-steer_limit:.6g} {steer_limit:.6g}"/>'
    )
    rows.append(
        f'    <position name="fr_steer_servo" joint="fr_steer" kp="{steer_kp:.6g}" kv="0.4" '
        f'ctrlrange="{-steer_limit:.6g} {steer_limit:.6g}"/>'
    )
    return "<actuator>\n" + "\n".join(rows) + "\n  </actuator>"


def _track_xml(track: TrackGeometry) -> str:
    def quat(yaw: float) -> str:
        return f"{math.cos(yaw / 2.0):.6g} 0 0 {math.sin(yaw / 2.0):.6g}"

    pieces = [
        '<geom name="floor" type="plane" size="8 5 0.05" pos="7.5 4 0" group="0" contype="1" conaffinity="1" '
        'rgba="0.16 0.18 0.21 1" friction="0.9 0.01 0.001"/>'
    ]
    for loop_name, loop in (("inner", track.inner), ("outer", track.outer)):
        for index in range(len(loop)):
            a = loop[index]
            b = loop[(index + 1) % len(loop)]
            dx = float(b[0] - a[0])
            dy = float(b[1] - a[1])
            length = math.hypot(dx, dy)
            if length < 1e-6:
                continue
            yaw = math.atan2(dy, dx)
            pieces.append(
                f'<geom name="wall_{loop_name}_{index}" type="box" '
                f'pos="{(a[0] + b[0]) / 2.0:.6g} {(a[1] + b[1]) / 2.0:.6g} {_WALL_HEIGHT_M / 2.0:.6g}" '
                f'size="{length / 2.0 + 0.005:.6g} {_WALL_RADIUS_M:.6g} {_WALL_HEIGHT_M / 2.0:.6g}" '
                f'quat="{quat(yaw)}" group="0" contype="1" conaffinity="1" rgba="0.42 0.46 0.5 1" '
                f'friction="0.7 0.01 0.001"/>'
            )
    for index, obstacle in enumerate(track.obstacles):
        if obstacle.kind == "box":
            pieces.append(
                f'<geom name="obstacle_{index}" type="box" '
                f'pos="{obstacle.x:.6g} {obstacle.y:.6g} {obstacle.height / 2.0:.6g}" '
                f'size="{obstacle.length / 2.0:.6g} {obstacle.width / 2.0:.6g} {obstacle.height / 2.0:.6g}" '
                f'quat="{quat(obstacle.yaw)}" group="0" contype="1" conaffinity="1" rgba="0.45 0.35 0.22 1"/>'
            )
        else:
            pieces.append(
                f'<geom name="obstacle_{index}" type="cylinder" '
                f'pos="{obstacle.x:.6g} {obstacle.y:.6g} {obstacle.height / 2.0:.6g}" '
                f'size="{obstacle.radius:.6g} {obstacle.height / 2.0:.6g}" '
                f'group="0" contype="1" conaffinity="1" rgba="0.3 0.38 0.44 1"/>'
            )
    return "\n    ".join(pieces)


def build_mjcf(
    vehicle: VehicleConfig,
    track: TrackGeometry,
    timestep_s: float = 1.0 / 240.0,
) -> str:
    """Return a complete MJCF XML string."""

    return f"""<?xml version="1.0"?>
<mujoco model="{escape(_safe_name(vehicle.name))}_digital_twin">
  <compiler angle="radian" coordinate="local" inertiafromgeom="false" autolimits="true"/>
  <option timestep="{timestep_s:.9g}" integrator="implicitfast" gravity="0 0 -9.81"
          cone="elliptic" impratio="10"/>
  <worldbody>
    {_track_xml(track)}
{_car_body_xml(vehicle)}
  </worldbody>
  {_actuator_xml(vehicle)}
</mujoco>
"""


def write_mjcf(
    vehicle: VehicleConfig,
    track: TrackGeometry,
    timestep_s: float = 1.0 / 240.0,
    path: str | Path | None = None,
) -> str:
    """Return the MJCF string; optional ``path`` writes it for inspection."""

    source = build_mjcf(vehicle, track, timestep_s)
    if path is not None:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(source, encoding="utf-8")
    return source
