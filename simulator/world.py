"""PyBullet world, track collision geometry and the generic 1/10 car."""

from __future__ import annotations

import contextlib
import math
from pathlib import Path

import pybullet as p
import pybullet_data

from simulator.car_urdf import write_urdf
from simulator.config import VehicleConfig
from simulator.track import Pose2, TrackGeometry
from simulator.vehicle import Actuation, VehicleState

ROOT = Path(__file__).resolve().parent


class PyBulletWorld:
    """PyBullet world with a 4WD Ackermann car and pipe-wall collisions."""

    def __init__(
        self,
        track: TrackGeometry,
        vehicle: VehicleConfig,
        gui: bool = False,
    ) -> None:
        self.track = track
        self.vehicle = vehicle
        self.physics_hz = 240.0
        self.dt = 1.0 / self.physics_hz
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        if self.client < 0:
            raise RuntimeError("cannot connect to PyBullet")

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0.0, 0.0, -9.81)
        p.setTimeStep(self.dt)
        p.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSolverIterations=20,
            numSubSteps=1,
        )
        if gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=8.0,
                cameraYaw=45.0,
                cameraPitch=-55.0,
                cameraTargetPosition=[7.5, 4.0, 0.0],
            )

        plane = p.loadURDF("plane.urdf")
        p.changeDynamics(plane, -1, lateralFriction=0.9, restitution=0.0)

        self.wall_ids = self._build_walls()
        self.wall_id_set = set(self.wall_ids)
        self.obstacle_ids = self._build_obstacles()
        self.car = self._load_car()
        self.joint_index = self._joint_map()
        self.wheel_joints = [
            self.joint_index[name]
            for name in (
                "fl_wheel_joint",
                "fr_wheel_joint",
                "rl_wheel_joint",
                "rr_wheel_joint",
            )
        ]
        self.steer_joints = [self.joint_index["fl_steer"], self.joint_index["fr_steer"]]
        self.driven_wheels = (
            self.wheel_joints if self.vehicle.drive == "4wd" else self.wheel_joints[2:]
        )
        self._steer_target = 0.0
        self._configure_car()

        self.actuation = Actuation()
        self.reset(self.track.sample(self.track.start_s))

    # ------------------------------------------------------------------ setup
    def _build_walls(self) -> list[int]:
        wall_height = 0.16
        wall_radius = 0.08
        half_extents_thickness = wall_radius
        ids: list[int] = []
        for loop in (self.track.inner, self.track.outer):
            for i in range(len(loop)):
                a = loop[i]
                b = loop[(i + 1) % len(loop)]
                dx = float(b[0] - a[0])
                dy = float(b[1] - a[1])
                length = math.hypot(dx, dy)
                if length < 1e-6:
                    continue
                yaw = math.atan2(dy, dx)
                # Slight overlap prevents cracks between adjacent boxes.
                shape = p.createCollisionShape(
                    p.GEOM_BOX,
                    halfExtents=[length / 2 + 0.005, half_extents_thickness, wall_height / 2],
                )
                position = [
                    float((a[0] + b[0]) / 2),
                    float((a[1] + b[1]) / 2),
                    wall_height / 2,
                ]
                orientation = [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]
                body = p.createMultiBody(
                    baseMass=0.0,
                    baseCollisionShapeIndex=shape,
                    basePosition=position,
                    baseOrientation=orientation,
                )
                p.changeDynamics(body, -1, lateralFriction=0.7, restitution=0.0)
                ids.append(body)
        return ids

    def _build_obstacles(self) -> list[int]:
        ids: list[int] = []
        for obstacle in self.track.obstacles:
            if obstacle.kind == "box":
                shape = p.createCollisionShape(
                    p.GEOM_BOX,
                    halfExtents=[
                        obstacle.length / 2.0,
                        obstacle.width / 2.0,
                        obstacle.height / 2.0,
                    ],
                )
            else:
                shape = p.createCollisionShape(
                    p.GEOM_CYLINDER,
                    radius=obstacle.radius,
                    height=obstacle.height,
                )
            body = p.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=shape,
                basePosition=[obstacle.x, obstacle.y, obstacle.height / 2.0],
                baseOrientation=[
                    0.0,
                    0.0,
                    math.sin(obstacle.yaw / 2.0),
                    math.cos(obstacle.yaw / 2.0),
                ],
            )
            p.changeDynamics(body, -1, lateralFriction=0.8, restitution=0.0)
            ids.append(body)
        return ids

    def _load_car(self) -> int:
        car = p.loadURDF(
            str(write_urdf(self.vehicle)),
            basePosition=[0.0, 0.0, self.vehicle.ride_height_m],
            baseOrientation=[0.0, 0.0, 0.0, 1.0],
            useFixedBase=False,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
        )
        return car

    def _joint_map(self) -> dict[str, int]:
        result: dict[str, int] = {}
        for index in range(p.getNumJoints(self.car)):
            info = p.getJointInfo(self.car, index)
            result[info[1].decode("utf-8")] = index
        return result

    def _configure_car(self) -> None:
        p.changeDynamics(
            self.car,
            -1,
            lateralFriction=float((self.vehicle.tires.friction_front + self.vehicle.tires.friction_rear) / 2.0),
            rollingFriction=max(0.0005, self.vehicle.tires.rolling_resistance * 0.2),
            spinningFriction=0.002,
            restitution=0.0,
            linearDamping=0.005 + 0.02 * self.vehicle.aero.drag_coefficient,
            angularDamping=0.02 + 0.01 * self.vehicle.suspension.damping,
            ccdSweptSphereRadius=max(0.005, self.vehicle.wheel_radius_m * 0.6),
        )
        front = set(self.wheel_joints[:2])
        for joint in self.wheel_joints:
            friction = (
                self.vehicle.tires.friction_front
                if joint in front
                else self.vehicle.tires.friction_rear
            )
            p.changeDynamics(
                self.car,
                joint,
                lateralFriction=friction,
                rollingFriction=max(0.0005, self.vehicle.tires.rolling_resistance * 0.2),
                spinningFriction=0.002,
            )
        for joint in self.steer_joints:
            p.changeDynamics(
                self.car,
                joint,
                jointDamping=max(0.05, self.vehicle.suspension.damping * 0.25),
            )

    # ---------------------------------------------------------------- control
    def reset(self, pose: Pose2) -> None:
        quaternion = p.getQuaternionFromEuler([0.0, 0.0, pose.theta])
        p.resetBasePositionAndOrientation(
            self.car,
            [pose.x, pose.y, self.vehicle.ride_height_m],
            quaternion,
        )
        p.resetBaseVelocity(self.car, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        for joint in [*self.wheel_joints, *self.steer_joints]:
            p.resetJointState(self.car, joint, 0.0, 0.0)
        self.actuation = Actuation()
        self._steer_target = 0.0
        # Let the suspension/contact settle without moving the car.
        for _ in range(20):
            self._apply_actuation(Actuation(), 0.0)
            p.stepSimulation()

    def set_actuation(self, actuation: Actuation) -> None:
        max_steer = math.radians(self.vehicle.steering.max_angle_deg)
        self.actuation = Actuation(
            throttle=max(-1.0, min(1.0, actuation.throttle)),
            steering=max(-max_steer, min(max_steer, actuation.steering)),
        )

    def step(self) -> None:
        self._apply_actuation(self.actuation, 1.0)
        p.stepSimulation()

    def _apply_actuation(self, actuation: Actuation, _scale: float) -> None:
        motor = self.vehicle.motor
        max_wheel_speed = (
            motor.max_rpm * 2.0 * math.pi / 60.0 / motor.gear_ratio
        )
        target = actuation.throttle * max_wheel_speed
        force = max(0.6, min(20.0, motor.max_torque_nm * motor.gear_ratio * 2.5))
        for joint in self.wheel_joints:
            if joint in self.driven_wheels:
                p.setJointMotorControl2(
                    self.car,
                    joint,
                    p.VELOCITY_CONTROL,
                    targetVelocity=target,
                    force=force if abs(actuation.throttle) > 0.01 else 0.6,
                )
            else:
                # Free rolling wheel on a 2WD car.
                p.setJointMotorControl2(
                    self.car,
                    joint,
                    p.VELOCITY_CONTROL,
                    targetVelocity=0.0,
                    force=0.01,
                )

        # Rate-limit the steering command to the configured servo rate.
        max_delta = math.radians(self.vehicle.steering.rate_deg_s) * self.dt
        error = actuation.steering - self._steer_target
        deadband = math.radians(self.vehicle.steering.deadband_deg)
        if abs(error) > deadband:
            self._steer_target += max(-max_delta, min(max_delta, error))
        for joint in self.steer_joints:
            p.setJointMotorControl2(
                self.car,
                joint,
                p.POSITION_CONTROL,
                targetPosition=self._steer_target,
                force=max(5.0, motor.max_torque_nm * motor.gear_ratio * 2.0),
                positionGain=1.0,
                velocityGain=1.0,
            )

    # ------------------------------------------------------------------ state
    def state(self) -> VehicleState:
        position, orientation = p.getBasePositionAndOrientation(self.car)
        linear, angular = p.getBaseVelocity(self.car)
        roll, pitch, yaw = p.getEulerFromQuaternion(orientation)
        rotation = p.getMatrixFromQuaternion(orientation)
        com = self.vehicle.center_of_mass_m
        # PyBullet reports the base centre of mass. Convert to the body frame
        # origin used everywhere else (wheel joints, sensor mounts, URDF parts).
        com_world = (
            rotation[0] * com[0] + rotation[1] * com[1] + rotation[2] * com[2],
            rotation[3] * com[0] + rotation[4] * com[1] + rotation[5] * com[2],
            rotation[6] * com[0] + rotation[7] * com[1] + rotation[8] * com[2],
        )
        origin = (
            float(position[0] - com_world[0]),
            float(position[1] - com_world[1]),
            float(position[2] - com_world[2]),
        )
        vx = linear[0] * math.cos(yaw) + linear[1] * math.sin(yaw)
        steering = 0.0
        if self.steer_joints:
            steering = float(p.getJointState(self.car, self.steer_joints[0])[0])
        wheel_speeds = (
            float(p.getJointState(self.car, self.wheel_joints[0])[1]),
            float(p.getJointState(self.car, self.wheel_joints[1])[1]),
            float(p.getJointState(self.car, self.wheel_joints[2])[1]),
            float(p.getJointState(self.car, self.wheel_joints[3])[1]),
        )
        return VehicleState(
            x=origin[0],
            y=origin[1],
            z=origin[2],
            yaw=float(yaw),
            speed=float(vx),
            yaw_rate=float(angular[2]),
            steering=steering,
            wheel_speeds=wheel_speeds,
        )

    def wheel_positions(self) -> list[tuple[float, float, float]]:
        links = [
            self.joint_index[name]
            for name in ("fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint")
        ]
        positions: list[tuple[float, float, float]] = []
        for link in links:
            state = p.getLinkState(self.car, link)
            positions.append((float(state[0][0]), float(state[0][1]), float(state[0][2])))
        return positions

    def raycast(
        self,
        origin: tuple[float, float, float],
        angle: float,
        max_range: float,
        height_offset: float = 0.0,
    ) -> tuple[float, tuple[float, float, float], tuple[float, float, float]] | None:
        """Raycast in the world XY plane from ``origin`` at a world angle."""

        start = [origin[0], origin[1], origin[2] + height_offset]
        end = [
            origin[0] + math.cos(angle) * max_range,
            origin[1] + math.sin(angle) * max_range,
            origin[2] + height_offset,
        ]
        hits = p.rayTestBatch([start], [end], numThreads=0)
        if not hits:
            return None
        object_id, _link, fraction, hit_position, hit_normal = hits[0]
        if object_id < 0 or fraction >= 1.0:
            return None
        distance = fraction * max_range
        return (
            distance,
            (float(hit_position[0]), float(hit_position[1]), float(hit_position[2])),
            (float(hit_normal[0]), float(hit_normal[1]), float(hit_normal[2])),
        )

    def raycast_batch(
        self,
        origins: list[tuple[float, float, float]],
        angles: list[float],
        max_range: float,
    ) -> list[tuple[float, tuple[float, float, float], tuple[float, float, float], int] | None]:
        """Batched horizontal raycasts. Returns (range, point, normal, object id)."""

        if not origins:
            return []
        ends = [
            [
                origin[0] + math.cos(angle) * max_range,
                origin[1] + math.sin(angle) * max_range,
                origin[2],
            ]
            for origin, angle in zip(origins, angles, strict=True)
        ]
        return self._raycast_segments(origins, ends, max_range)

    def raycast_vectors(
        self,
        origins: list[tuple[float, float, float]],
        directions: list[tuple[float, float, float]],
        max_range: float,
    ) -> list[tuple[float, tuple[float, float, float], tuple[float, float, float], int] | None]:
        """Batched 3D raycasts from unit direction vectors."""

        if not origins:
            return []
        ends = [
            [
                origin[0] + direction[0] * max_range,
                origin[1] + direction[1] * max_range,
                origin[2] + direction[2] * max_range,
            ]
            for origin, direction in zip(origins, directions, strict=True)
        ]
        return self._raycast_segments(origins, ends, max_range)

    def _raycast_segments(
        self,
        origins: list[tuple[float, float, float]] | list[list[float]],
        ends: list[list[float]],
        max_range: float,
    ) -> list[tuple[float, tuple[float, float, float], tuple[float, float, float], int] | None]:
        hits = p.rayTestBatch(origins, ends, numThreads=0)
        result: list[
            tuple[float, tuple[float, float, float], tuple[float, float, float], int] | None
        ] = []
        for object_id, _link, fraction, hit_position, hit_normal in hits:
            if object_id < 0 or fraction >= 1.0:
                result.append(None)
                continue
            result.append(
                (
                    float(fraction) * max_range,
                    (float(hit_position[0]), float(hit_position[1]), float(hit_position[2])),
                    (float(hit_normal[0]), float(hit_normal[1]), float(hit_normal[2])),
                    int(object_id),
                )
            )
        return result

    def close(self) -> None:
        with contextlib.suppress(Exception):
            p.disconnect(self.client)
