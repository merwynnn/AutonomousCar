"""Generate a PyBullet URDF from the vehicle configuration.

The car is defined by one YAML file (or one ``VehicleConfig`` built in Python).
This module turns that description into a real URDF so the 3D body, wheelbase,
track width, wheel size, masses and collision geometry all agree with what the
dashboard draws.

Body parts use the vehicle body frame: x forward, y left, z up, origin at the
chassis centre. Wheel positions are derived from ``wheelbase_m`` and
``track_width_m``; no dimensions are duplicated in a hand-written URDF.
"""

from __future__ import annotations

import hashlib
import math
import re
import tempfile
from pathlib import Path
from xml.sax.saxutils import escape

from simulator.config import BodyPart, VehicleConfig

# Cylinder axis is z in URDF; rotate it to the wheel axle (body y).
_WHEEL_RPY = "1.5708 0 0"


def _fmt(values: tuple[float, ...] | list[float]) -> str:
    return " ".join(f"{float(value):.6g}" for value in values)


def _hex_rgba(color: str, opacity: float) -> str:
    value = color.strip().lstrip("#")
    if len(value) == 3:
        value = "".join(char * 2 for char in value)
    if len(value) != 6:
        value = "a53235"
    red = int(value[0:2], 16) / 255.0
    green = int(value[2:4], 16) / 255.0
    blue = int(value[4:6], 16) / 255.0
    return f"{red:.4f} {green:.4f} {blue:.4f} {opacity:.3f}"


def _safe_name(name: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", name.strip())
    return cleaned or "part"


def _rpy_rad(part: BodyPart) -> tuple[float, float, float]:
    return tuple(math.radians(value) for value in part.rpy_deg)  # type: ignore[return-value]


def _geometry_xml(part: BodyPart) -> str:
    sx, sy, sz = part.size_m
    if part.shape == "cylinder":
        return f'<cylinder radius="{sx / 2.0:.6g}" length="{sz:.6g}"/>'
    if part.shape == "sphere":
        return f'<sphere radius="{sx / 2.0:.6g}"/>'
    return f'<box size="{_fmt((sx, sy, sz))}"/>'


def _part_xml(part: BodyPart, tag: str) -> str:
    if tag == "visual" and not part.visible:
        return ""
    if tag == "collision" and not part.collision:
        return ""
    material = ""
    if tag == "visual":
        material = (
            f'<material name="{_safe_name(part.name)}_mat">'
            f'<color rgba="{_hex_rgba(part.color, part.opacity)}"/>'
            "</material>"
        )
    return (
        f"<{tag}>"
        f'<origin xyz="{_fmt(part.position_m)}" rpy="{_fmt(_rpy_rad(part))}"/>'
        f"<geometry>{_geometry_xml(part)}</geometry>"
        f"{material}"
        f"</{tag}>"
    )


def _collision_bounds(vehicle: VehicleConfig) -> tuple[float, float, float]:
    parts = [part for part in vehicle.resolved_structure() if part.collision]
    if not parts:
        dims = vehicle.dimensions
        return (dims.length_m, dims.width_m, dims.height_m)
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
    ixx = mass * (y * y + z * z) / 12.0
    iyy = mass * (x * x + z * z) / 12.0
    izz = mass * (x * x + y * y) / 12.0
    return (ixx, iyy, izz)


def _wheel_link_xml(
    name: str,
    vehicle: VehicleConfig,
    colour: str = "#15171a",
) -> str:
    radius = vehicle.wheel_radius_m
    width = vehicle.wheel_width_m
    mass = vehicle.wheel_mass_kg
    # Cylinder axis is body y; Iyy is the spin axis.
    ixx = mass * (3.0 * radius * radius + width * width) / 12.0
    iyy = mass * radius * radius / 2.0
    izz = ixx
    return f"""
    <link name="{name}">
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="{mass:.6g}"/>
        <inertia ixx="{ixx:.6g}" ixy="0" ixz="0" iyy="{iyy:.6g}" iyz="0" izz="{izz:.6g}"/>
      </inertial>
      <collision>
        <origin xyz="0 0 0" rpy="{_WHEEL_RPY}"/>
        <geometry><cylinder radius="{radius:.6g}" length="{width:.6g}"/></geometry>
      </collision>
      <visual>
        <origin xyz="0 0 0" rpy="{_WHEEL_RPY}"/>
        <geometry><cylinder radius="{radius:.6g}" length="{width:.6g}"/></geometry>
        <material name="{name}_mat"><color rgba="{_hex_rgba(colour, 1.0)}"/></material>
      </visual>
    </link>"""


def build_urdf(vehicle: VehicleConfig) -> str:
    """Return a complete URDF string for ``vehicle``."""

    parts = vehicle.resolved_structure()
    visual_parts = "".join(_part_xml(part, "visual") for part in parts)
    collision_parts = "".join(_part_xml(part, "collision") for part in parts)
    if not collision_parts:
        dims = vehicle.dimensions
        collision_parts = (
            "<collision><origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
            f'<box size="{dims.length_m:.6g} {dims.width_m:.6g} '
            f'{max(0.01, dims.height_m * 0.45):.6g}"/></collision>'
        )

    inertia_size = _collision_bounds(vehicle)
    ixx, iyy, izz = _box_inertia(vehicle.chassis_mass_kg, inertia_size)
    mass = vehicle.chassis_mass_kg
    com = _fmt(vehicle.center_of_mass_m)
    half_wheelbase = vehicle.wheelbase_m / 2.0
    half_track = vehicle.track_width_m / 2.0
    wheel_z = vehicle.wheel_offset_z_m
    steer_limit = math.radians(vehicle.steering.max_angle_deg)
    steer_velocity = math.radians(vehicle.steering.rate_deg_s)
    steer_effort = max(5.0, vehicle.motor.max_torque_nm * vehicle.motor.gear_ratio * 2.0)
    wheel_velocity = vehicle.motor.max_rpm * 2.0 * math.pi / 60.0 / vehicle.motor.gear_ratio
    wheel_effort = max(1.0, vehicle.motor.max_torque_nm * vehicle.motor.gear_ratio)
    joint_damping = max(0.001, vehicle.suspension.damping * 0.01)

    front_knuckles = ""
    front_wheels = ""
    for side, suffix in ((half_track, "l"), (-half_track, "r")):
        knuckle = f"f{suffix}_knuckle"
        wheel = f"f{suffix}_wheel"
        steer = f"f{suffix}_steer"
        wheel_joint = f"f{suffix}_wheel_joint"
        front_knuckles += f"""
    <link name="{knuckle}">
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.02"/>
        <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
      </inertial>
    </link>
    <joint name="{steer}" type="revolute">
      <parent link="base_link"/>
      <child link="{knuckle}"/>
      <origin xyz="{half_wheelbase:.6g} {side:.6g} {wheel_z:.6g}" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="{-steer_limit:.6g}" upper="{steer_limit:.6g}" effort="{steer_effort:.6g}" velocity="{steer_velocity:.6g}"/>
      <dynamics damping="{max(0.05, joint_damping):.6g}" friction="0.01"/>
    </joint>"""
        front_wheels += _wheel_link_xml(wheel, vehicle)
        front_wheels += f"""
    <joint name="{wheel_joint}" type="continuous">
      <parent link="{knuckle}"/>
      <child link="{wheel}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit effort="{wheel_effort:.6g}" velocity="{wheel_velocity:.6g}"/>
      <dynamics damping="0.004" friction="0.001"/>
    </joint>"""

    rear_wheels = ""
    for side, suffix in ((half_track, "l"), (-half_track, "r")):
        wheel = f"r{suffix}_wheel"
        wheel_joint = f"r{suffix}_wheel_joint"
        rear_wheels += _wheel_link_xml(wheel, vehicle)
        rear_wheels += f"""
    <joint name="{wheel_joint}" type="continuous">
      <parent link="base_link"/>
      <child link="{wheel}"/>
      <origin xyz="{-half_wheelbase:.6g} {side:.6g} {wheel_z:.6g}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit effort="{wheel_effort:.6g}" velocity="{wheel_velocity:.6g}"/>
      <dynamics damping="0.004" friction="0.001"/>
    </joint>"""

    robot_name = _safe_name(vehicle.name)
    return f"""<?xml version="1.0"?>
<!-- Generated by simulator.car_urdf from the vehicle configuration. -->
<robot name="{escape(robot_name)}">
    <link name="base_link">
      <inertial>
        <origin xyz="{com}" rpy="0 0 0"/>
        <mass value="{mass:.6g}"/>
        <inertia ixx="{ixx:.6g}" ixy="0" ixz="0" iyy="{iyy:.6g}" iyz="0" izz="{izz:.6g}"/>
      </inertial>
{visual_parts}
{collision_parts}
    </link>
{front_knuckles}
{front_wheels}
{rear_wheels}
</robot>
"""


def write_urdf(vehicle: VehicleConfig, directory: str | Path | None = None) -> Path:
    """Write the generated URDF to a cached file and return its path."""

    source = build_urdf(vehicle)
    digest = hashlib.sha1(source.encode("utf-8")).hexdigest()[:16]
    root = Path(directory) if directory is not None else Path(tempfile.gettempdir()) / "awake_sim_urdf"
    root.mkdir(parents=True, exist_ok=True)
    target = root / f"{_safe_name(vehicle.name)}_{digest}.urdf"
    if not target.exists() or target.read_text(encoding="utf-8") != source:
        target.write_text(source, encoding="utf-8")
    return target
