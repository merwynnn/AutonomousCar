"""Wheel/IMU odometry model with scale errors, bias and noise."""

from __future__ import annotations

import math

import numpy as np

from awake_sim.config import OdometryConfig
from awake_sim.tracks.geometry import Pose2


class OdometryModel:
    """Integrates measured speed/yaw rate into a drifting odometry pose."""

    def __init__(self, config: OdometryConfig, seed: int = 0) -> None:
        self.config = config
        self.rng = np.random.default_rng(seed + 401)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.yaw_bias = math.radians(config.yaw_bias_deg_s)
        self._speed_bias = config.speed_bias_mps

    def reset(self, pose: Pose2) -> None:
        self.x = pose.x
        self.y = pose.y
        self.theta = pose.theta
        self.yaw_bias = math.radians(self.config.yaw_bias_deg_s)

    def update(self, true_speed: float, true_yaw_rate: float, dt: float) -> Pose2:
        scale = 1.0 + self.config.speed_scale_error
        measured_speed = true_speed * scale + self._speed_bias
        measured_speed += self.rng.normal(0.0, self.config.speed_noise_mps)
        # Slow random walk of the gyro bias.
        self.yaw_bias += self.rng.normal(
            0.0, math.radians(self.config.yaw_bias_walk_deg_s_sqrt_s) * math.sqrt(dt)
        )
        measured_yaw = (
            true_yaw_rate * (1.0 + self.config.yaw_scale_error)
            + self.yaw_bias
            + self.rng.normal(0.0, self.config.yaw_noise_rad_s)
        )
        self.x += measured_speed * math.cos(self.theta) * dt
        self.y += measured_speed * math.sin(self.theta) * dt
        self.theta = math.atan2(
            math.sin(self.theta + measured_yaw * dt),
            math.cos(self.theta + measured_yaw * dt),
        )
        return Pose2(self.x, self.y, self.theta)

    @property
    def pose(self) -> Pose2:
        return Pose2(self.x, self.y, self.theta)
