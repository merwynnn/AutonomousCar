"""Wheel + IMU complementary estimator shared by simulation and hardware.

This is deliberately simple and honest:

- rear encoder ticks provide forward speed and a wheel-derived yaw rate;
- the gyro provides short-term yaw rate;
- the steering angle provides a kinematic yaw-rate sanity check;
- a zero-velocity update learns the gyro bias when the car is still;
- a slip flag is raised when wheel and gyro yaw rates disagree.

It is not a Kalman filter. The covariance is a conservative diagnostic estimate
that can be replaced by a real EKF later without changing the controller API.
"""

from __future__ import annotations

import math

import numpy as np

from onboard.api import (
    REAR_WHEEL_INDICES,
    EncoderSample,
    Pose2,
    SensorFrame,
    StateEstimate,
)
from onboard.config import EstimatorTuning

_TWO_PI = 2.0 * math.pi


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % _TWO_PI - math.pi


class WheelImuEstimator:
    """Fuse rear-wheel encoders, gyro and steering into a planar estimate."""

    def __init__(
        self,
        tuning: EstimatorTuning | None = None,
        *,
        wheel_radius_m: float,
        track_width_m: float,
        wheelbase_m: float,
        ticks_per_rev: int,
    ) -> None:
        if wheel_radius_m <= 0.0:
            raise ValueError("wheel_radius_m must be positive")
        if track_width_m <= 0.0:
            raise ValueError("track_width_m must be positive")
        if wheelbase_m <= 0.0:
            raise ValueError("wheelbase_m must be positive")
        if ticks_per_rev <= 0:
            raise ValueError("ticks_per_rev must be positive")
        self.tuning = tuning or EstimatorTuning()
        self.wheel_radius_m = wheel_radius_m
        self.track_width_m = track_width_m
        self.wheelbase_m = wheelbase_m
        self.ticks_per_rev = ticks_per_rev

        self._pose = Pose2(0.0, 0.0, 0.0)
        self._speed = 0.0
        self._yaw_rate = 0.0
        self._gyro_bias = 0.0
        self._covariance = np.eye(3) * 1e-6
        self._last_time: float | None = None
        self._last_ticks: tuple[int, int, int, int] | None = None
        self._last_encoder_time: float | None = None
        self._last_encoder_sample_time: float | None = None
        self._last_wheel_speeds: tuple[float, float, float, float] | None = None
        self._slip_detected = False
        self._source = "initialized"
        self._state = self._snapshot(0.0)

    # ------------------------------------------------------------ properties
    @property
    def pose(self) -> Pose2:
        """Backward-compatible alias for the estimated pose."""

        return self._pose

    @property
    def last_speed(self) -> float:
        return self._speed

    @property
    def last_yaw_rate(self) -> float:
        return self._yaw_rate

    @property
    def state(self) -> StateEstimate:
        return self._snapshot(self._state.t)

    # --------------------------------------------------------------- lifecycle
    def reset(self, initial_pose: Pose2) -> StateEstimate:
        self._pose = Pose2(initial_pose.x, initial_pose.y, initial_pose.theta)
        self._speed = 0.0
        self._yaw_rate = 0.0
        self._gyro_bias = 0.0
        self._covariance = np.eye(3) * 1e-6
        self._last_time = None
        self._last_ticks = None
        self._last_encoder_time = None
        self._last_encoder_sample_time = None
        self._last_wheel_speeds = None
        self._slip_detected = False
        self._source = "initialized"
        self._state = self._snapshot(0.0)
        return self._snapshot(0.0)

    def update(self, sensors: SensorFrame, now: float) -> StateEstimate:
        now = float(now)
        if self._last_time is None:
            dt = 0.0
        else:
            dt = now - self._last_time
            dt = 0.0 if dt < 0.0 else min(dt, 0.25)
        self._last_time = now

        encoders = sensors.encoders
        imu = sensors.imu
        steering = sensors.steering

        wheel_valid = False
        v_wheel: float | None = None
        omega_wheel: float | None = None
        wheel_speeds: tuple[float, float, float, float] | None = None
        if encoders is not None and not encoders.dropped:
            if encoders.t_measure != self._last_encoder_sample_time:
                self._last_encoder_sample_time = encoders.t_measure
                measured = self._encoder_speeds(encoders)
                if measured is not None:
                    self._last_wheel_speeds = measured
            wheel_speeds = self._last_wheel_speeds
            if wheel_speeds is not None:
                left = wheel_speeds[REAR_WHEEL_INDICES[0]]
                right = wheel_speeds[REAR_WHEEL_INDICES[1]]
                v_wheel = 0.5 * (left + right)
                omega_wheel = (right - left) / self.track_width_m
                wheel_valid = True

        gyro_valid = imu is not None and imu.valid
        omega_gyro = (
            float(imu.gyro_rad_s[2]) - self._gyro_bias if gyro_valid and imu is not None else None
        )

        steer_valid = steering is not None and steering.valid
        omega_steer: float | None = None
        if steer_valid and steering is not None:
            speed_for_steer = v_wheel if v_wheel is not None else self._speed
            omega_steer = (
                speed_for_steer * math.tan(steering.angle_rad) / self.wheelbase_m
            )

        slip = False
        if wheel_valid and gyro_valid and omega_wheel is not None and omega_gyro is not None:
            disagreement = abs(omega_wheel - omega_gyro)
            if (
                abs(v_wheel or 0.0) >= self.tuning.slip_speed_threshold_mps
                and disagreement > self.tuning.slip_yaw_threshold_rad_s
            ):
                slip = True
            elif (
                abs(v_wheel or 0.0) < self.tuning.slip_speed_threshold_mps
                and abs(omega_wheel) < self.tuning.zero_velocity_yaw_rate_rad_s
                and abs(omega_gyro) > 3.0 * self.tuning.zero_velocity_yaw_rate_rad_s
            ):
                # Wheels report a standstill while the body rotates: the car is
                # sliding or a wheel is spinning in place.
                slip = True

        wheel_stationary = (
            wheel_valid
            and abs(v_wheel or 0.0) < self.tuning.zero_velocity_speed_mps
        )
        gyro_stationary = (
            not gyro_valid
            or abs(omega_gyro or 0.0) < self.tuning.zero_velocity_yaw_rate_rad_s
        )
        stationary = wheel_stationary and gyro_stationary and not slip

        if stationary:
            speed = 0.0
            yaw_rate = 0.0
            if gyro_valid and omega_gyro is not None and dt > 0.0:
                # With the car still, any corrected gyro rate is bias.
                self._gyro_bias += self.tuning.gyro_bias_gain * omega_gyro * dt
                self._gyro_bias = float(
                    np.clip(
                        self._gyro_bias,
                        -self.tuning.max_gyro_bias_rad_s,
                        self.tuning.max_gyro_bias_rad_s,
                    )
                )
            source = "zero_velocity"
        else:
            if wheel_valid and v_wheel is not None:
                speed = (
                    0.5 * v_wheel + 0.5 * self._speed
                    if slip
                    else v_wheel
                )
            else:
                speed = self._speed

            if gyro_valid and omega_gyro is not None:
                if slip:
                    auxiliary = omega_steer if omega_steer is not None else omega_wheel
                    alpha = 0.95
                else:
                    auxiliary = omega_wheel if wheel_valid else omega_steer
                    alpha = self.tuning.gyro_weight
                if auxiliary is None:
                    yaw_rate = omega_gyro
                else:
                    yaw_rate = alpha * omega_gyro + (1.0 - alpha) * auxiliary
            elif wheel_valid and omega_wheel is not None:
                yaw_rate = omega_wheel
            elif omega_steer is not None:
                yaw_rate = omega_steer
            else:
                yaw_rate = self._yaw_rate

            if (
                gyro_valid
                and omega_gyro is not None
                and wheel_valid
                and omega_wheel is not None
                and not slip
                and dt > 0.0
            ):
                # Slowly track the gyro bias from the wheel-derived yaw rate.
                error = omega_wheel - omega_gyro
                self._gyro_bias -= self.tuning.gyro_bias_tracking_gain * error * dt
                self._gyro_bias = float(
                    np.clip(
                        self._gyro_bias,
                        -self.tuning.max_gyro_bias_rad_s,
                        self.tuning.max_gyro_bias_rad_s,
                    )
                )

            if gyro_valid and wheel_valid:
                source = "wheel_imu"
            elif gyro_valid:
                source = "gyro_only"
            elif wheel_valid:
                source = "wheel_only"
            elif omega_steer is not None:
                source = "steering_only"
            else:
                source = "dead_reckoning"

        if dt > 0.0:
            theta_mid = _wrap_angle(self._pose.theta + 0.5 * yaw_rate * dt)
            x = self._pose.x + speed * math.cos(theta_mid) * dt
            y = self._pose.y + speed * math.sin(theta_mid) * dt
            theta = _wrap_angle(self._pose.theta + yaw_rate * dt)
            self._pose = Pose2(x, y, theta)
            self._update_covariance(dt, speed, yaw_rate, wheel_valid, gyro_valid)

        self._speed = speed
        self._yaw_rate = yaw_rate
        self._slip_detected = slip
        self._source = source
        self._state = self._snapshot(now)
        return self._snapshot(now)

    # --------------------------------------------------------------- helpers
    def _encoder_speeds(
        self, sample: EncoderSample
    ) -> tuple[float, float, float, float] | None:
        if sample.ticks is None:
            return sample.wheel_speeds_mps

        if self._last_ticks is None:
            self._last_ticks = sample.ticks
            self._last_encoder_time = sample.t_measure
            return sample.wheel_speeds_mps

        dt = sample.dt
        if dt <= 0.0 and self._last_encoder_time is not None:
            dt = sample.t_measure - self._last_encoder_time
        if dt <= 0.0:
            return sample.wheel_speeds_mps

        speeds: list[float] = []
        for tick, previous in zip(sample.ticks, self._last_ticks, strict=True):
            delta = tick - previous
            if delta > self.ticks_per_rev / 2:
                delta -= self.ticks_per_rev
            elif delta < -self.ticks_per_rev / 2:
                delta += self.ticks_per_rev
            angle = delta / self.ticks_per_rev * _TWO_PI
            speeds.append(angle / dt * self.wheel_radius_m)
        self._last_ticks = sample.ticks
        self._last_encoder_time = sample.t_measure
        return (speeds[0], speeds[1], speeds[2], speeds[3])

    def _update_covariance(
        self,
        dt: float,
        speed: float,
        yaw_rate: float,
        wheel_valid: bool,
        gyro_valid: bool,
    ) -> None:
        q_position = (self.tuning.position_noise_mps * dt) ** 2 + (
            abs(speed) * dt * 0.02
        ) ** 2
        self._covariance[0, 0] += q_position
        self._covariance[1, 1] += q_position
        self._covariance[2, 2] += (self.tuning.yaw_noise_rad_s * dt) ** 2 + (
            abs(yaw_rate) * dt * 0.02
        ) ** 2
        if wheel_valid:
            self._covariance[0, 0] *= 0.995
            self._covariance[1, 1] *= 0.995
        if gyro_valid:
            self._covariance[2, 2] *= 0.95

    def _snapshot(self, now: float) -> StateEstimate:
        return StateEstimate(
            t=now,
            pose=self._pose,
            speed_mps=self._speed,
            yaw_rate_rad_s=self._yaw_rate,
            covariance=np.array(self._covariance, copy=True),
            valid=True,
            source=self._source,
            gyro_bias_rad_s=self._gyro_bias,
            slip_detected=self._slip_detected,
        )


__all__ = ["WheelImuEstimator"]
