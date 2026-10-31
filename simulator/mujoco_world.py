"""MuJoCo physics backend for the shared vehicle and track definition."""

from __future__ import annotations

import math

import mujoco
import numpy as np

from simulator.config import VehicleConfig
from simulator.mjcf import build_mjcf
from simulator.track import Pose2, TrackGeometry
from simulator.vehicle import Actuation, VehicleState

RayHit = tuple[
    float,
    tuple[float, float, float],
    tuple[float, float, float],
    int,
]


class MuJoCoWorld:
    """Contact-based MuJoCo world with a 4WD Ackermann car.

    The browser still renders from ``VehicleConfig``; MuJoCo only owns physics
    and raycasts. Sensors use world rays by geometry group, so the car itself is
    excluded from lidar and ultrasonic returns.
    """

    def __init__(
        self,
        track: TrackGeometry,
        vehicle: VehicleConfig,
        physics_hz: float = 240.0,
        gui: bool = False,
    ) -> None:
        if gui:
            raise ValueError("MuJoCo backend does not support --gui; use the dashboard or pybullet")
        self.track = track
        self.vehicle = vehicle
        self.physics_hz = physics_hz
        self.dt = 1.0 / physics_hz
        self.model = mujoco.MjModel.from_xml_string(
            build_mjcf(vehicle, track, timestep_s=self.dt)
        )
        self.data = mujoco.MjData(self.model)
        self.client = 0
        self.actuation = Actuation()
        self._steer_target = 0.0
        self._geomgroup = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)

        self.chassis_id = self._body_id("chassis")
        self.joint_ids = {
            name: self._joint_id(name)
            for name in (
                "chassis_free",
                "fl_steer",
                "fr_steer",
                "fl_wheel_spin",
                "fr_wheel_spin",
                "rl_wheel_spin",
                "rr_wheel_spin",
            )
        }
        self.actuator_ids = {
            name: self._actuator_id(name)
            for name in (
                "fl_wheel_spin_motor",
                "fr_wheel_spin_motor",
                "rl_wheel_spin_motor",
                "rr_wheel_spin_motor",
                "fl_steer_servo",
                "fr_steer_servo",
            )
        }
        self.wheel_joint_ids = [
            self.joint_ids[name]
            for name in (
                "fl_wheel_spin",
                "fr_wheel_spin",
                "rl_wheel_spin",
                "rr_wheel_spin",
            )
        ]
        self.steer_joint_ids = [self.joint_ids["fl_steer"], self.joint_ids["fr_steer"]]
        self.driven_wheels = (
            list(range(4)) if vehicle.drive == "4wd" else [2, 3]
        )
        self.wall_id_set = {
            geom_id
            for geom_id in range(self.model.ngeom)
            if (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or "").startswith(
                "wall_"
            )
        }
        self._free_qpos = int(self.model.jnt_qposadr[self.joint_ids["chassis_free"]])
        self._free_qvel = int(self.model.jnt_dofadr[self.joint_ids["chassis_free"]])
        self.reset(track.sample(track.start_s))

    # ------------------------------------------------------------------ ids
    def _body_id(self, name: str) -> int:
        result = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if result < 0:
            raise RuntimeError(f"MJCF body {name!r} not found")
        return int(result)

    def _joint_id(self, name: str) -> int:
        result = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if result < 0:
            raise RuntimeError(f"MJCF joint {name!r} not found")
        return int(result)

    def _actuator_id(self, name: str) -> int:
        result = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if result < 0:
            raise RuntimeError(f"MJCF actuator {name!r} not found")
        return int(result)

    # ---------------------------------------------------------------- reset
    def reset(self, pose: Pose2) -> None:
        mujoco.mj_resetData(self.model, self.data)
        yaw = pose.theta
        self.data.qpos[self._free_qpos : self._free_qpos + 3] = [
            pose.x,
            pose.y,
            self.vehicle.ride_height_m,
        ]
        self.data.qpos[self._free_qpos + 3 : self._free_qpos + 7] = [
            math.cos(yaw / 2.0),
            0.0,
            0.0,
            math.sin(yaw / 2.0),
        ]
        for joint_id in self.steer_joint_ids:
            self.data.qpos[self.model.jnt_qposadr[joint_id]] = 0.0
        self.data.ctrl[:] = 0.0
        self.actuation = Actuation()
        self._steer_target = 0.0
        mujoco.mj_forward(self.model, self.data)
        # Let tyre contact and the free body settle without moving the car.
        for _ in range(20):
            self._apply_controls(Actuation())
            mujoco.mj_step(self.model, self.data)

    # --------------------------------------------------------------- control
    def set_actuation(self, actuation: Actuation) -> None:
        max_steer = math.radians(self.vehicle.steering.max_angle_deg)
        self.actuation = Actuation(
            throttle=max(-1.0, min(1.0, actuation.throttle)),
            steering=max(-max_steer, min(max_steer, actuation.steering)),
        )

    def step(self) -> None:
        self._apply_controls(self.actuation)
        self._apply_aero_drag()
        mujoco.mj_step(self.model, self.data)

    def _apply_aero_drag(self) -> None:
        velocity = self.data.qvel[self._free_qvel : self._free_qvel + 2]
        speed = float(np.linalg.norm(velocity))
        if speed < 1e-6:
            self.data.qfrc_applied[:] = 0.0
            return
        drag = (
            0.5
            * 1.225
            * self.vehicle.aero.drag_coefficient
            * self.vehicle.aero.frontal_area_m2
            * speed
        )
        self.data.qfrc_applied[:] = 0.0
        self.data.qfrc_applied[self._free_qvel] = -drag * float(velocity[0])
        self.data.qfrc_applied[self._free_qvel + 1] = -drag * float(velocity[1])

    def _apply_controls(self, actuation: Actuation) -> None:
        motor = self.vehicle.motor
        max_wheel_speed = motor.max_rpm * 2.0 * math.pi / 60.0 / motor.gear_ratio
        max_delta = math.radians(self.vehicle.steering.rate_deg_s) * self.dt
        error = actuation.steering - self._steer_target
        deadband = math.radians(self.vehicle.steering.deadband_deg)
        if abs(error) > deadband:
            self._steer_target += max(-max_delta, min(max_delta, error))
        steer_ctrl = max(
            -math.radians(self.vehicle.steering.max_angle_deg),
            min(math.radians(self.vehicle.steering.max_angle_deg), self._steer_target),
        )
        self.data.ctrl[self.actuator_ids["fl_steer_servo"]] = steer_ctrl
        self.data.ctrl[self.actuator_ids["fr_steer_servo"]] = steer_ctrl

        # Approximate motor/ESC current limiting and LiPo voltage sag.
        current_demand = abs(actuation.throttle) * motor.current_limit_a
        battery_voltage = max(
            0.5 * self.vehicle.battery.nominal_voltage_v,
            self.vehicle.battery.nominal_voltage_v
            - current_demand * self.vehicle.battery.internal_resistance_ohm,
        )
        voltage_factor = battery_voltage / self.vehicle.battery.nominal_voltage_v
        total_torque = motor.max_torque_nm * motor.gear_ratio * 0.90 * voltage_factor
        per_wheel = total_torque / max(1, len(self.driven_wheels))
        for index, joint_id in enumerate(self.wheel_joint_ids):
            actuator = self.actuator_ids[
                (
                    "fl_wheel_spin_motor",
                    "fr_wheel_spin_motor",
                    "rl_wheel_spin_motor",
                    "rr_wheel_spin_motor",
                )[index]
            ]
            if index not in self.driven_wheels:
                self.data.ctrl[actuator] = 0.0
                continue
            omega = float(
                self.data.qvel[int(self.model.jnt_dofadr[joint_id])]
            )
            speed_factor = max(
                0.0, 1.0 - min(1.0, abs(omega) / max(1e-6, max_wheel_speed))
            )
            torque = actuation.throttle * per_wheel * speed_factor
            self.data.ctrl[actuator] = torque

    # ----------------------------------------------------------------- state
    def state(self) -> VehicleState:
        position = self.data.xpos[self.chassis_id]
        quaternion = self.data.xquat[self.chassis_id]
        w, x, y, z = (float(value) for value in quaternion)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        linear = self.data.qvel[self._free_qvel : self._free_qvel + 3]
        angular = self.data.qvel[self._free_qvel + 3 : self._free_qvel + 6]
        speed = float(linear[0] * math.cos(yaw) + linear[1] * math.sin(yaw))
        steering = float(
            sum(
                self.data.qpos[self.model.jnt_qposadr[joint_id]]
                for joint_id in self.steer_joint_ids
            )
            / len(self.steer_joint_ids)
        )
        wheel_speeds = (
            float(self.data.qvel[int(self.model.jnt_dofadr[self.wheel_joint_ids[0]])]),
            float(self.data.qvel[int(self.model.jnt_dofadr[self.wheel_joint_ids[1]])]),
            float(self.data.qvel[int(self.model.jnt_dofadr[self.wheel_joint_ids[2]])]),
            float(self.data.qvel[int(self.model.jnt_dofadr[self.wheel_joint_ids[3]])]),
        )
        return VehicleState(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
            yaw=yaw,
            speed=speed,
            yaw_rate=float(angular[2]),
            steering=steering,
            wheel_speeds=wheel_speeds,
        )

    def wheel_positions(self) -> list[tuple[float, float, float]]:
        names = ("fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel")
        positions: list[tuple[float, float, float]] = []
        for name in names:
            body_id = self._body_id(name)
            position = self.data.xpos[body_id]
            positions.append((float(position[0]), float(position[1]), float(position[2])))
        return positions

    # -------------------------------------------------------------- raycasts
    def raycast_vectors(
        self,
        origins: list[tuple[float, float, float]],
        directions: list[tuple[float, float, float]],
        max_range: float,
    ) -> list[RayHit | None]:
        if not origins:
            return []
        results: list[RayHit | None] = []
        for origin, direction in zip(origins, directions, strict=True):
            point = np.asarray(origin, dtype=np.float64)
            vector = np.asarray(direction, dtype=np.float64)
            geom_id = np.array([-1], dtype=np.int32)
            normal = np.zeros(3, dtype=np.float64)
            distance = mujoco.mj_ray(
                self.model,
                self.data,
                point,
                vector,
                self._geomgroup,
                True,
                self.chassis_id,
                geom_id,
                normal,
            )
            if distance < 0.0 or distance > max_range:
                results.append(None)
                continue
            hit = point + vector * distance
            results.append(
                (
                    float(distance),
                    (float(hit[0]), float(hit[1]), float(hit[2])),
                    (float(normal[0]), float(normal[1]), float(normal[2])),
                    int(geom_id[0]),
                )
            )
        return results

    def raycast_batch(
        self,
        origins: list[tuple[float, float, float]],
        angles: list[float],
        max_range: float,
    ) -> list[RayHit | None]:
        directions = [(math.cos(angle), math.sin(angle), 0.0) for angle in angles]
        return self.raycast_vectors(origins, directions, max_range)

    def close(self) -> None:
        # MuJoCo is in-process; nothing to disconnect.
        return
