"""Non-ideal IMU model.

The physics provides true body rates and acceleration. This model adds the
BNO085-class errors that matter to the estimator: constant gyro bias, bias
random walk, scale error, noise, cross-axis coupling, mounting misalignment,
saturation and latency.
"""

from __future__ import annotations

import math

import numpy as np
from onboard.api import ImuSample

from simulator.config import ImuConfig
from simulator.vehicle import VehicleState


def _value(values: dict[str, float], key: str, default: float = 0.0) -> float:
    return float(values.get(key, default))


class ImuModel:
    def __init__(self, config: ImuConfig, seed: int = 0) -> None:
        self.config = config
        self.rng = np.random.default_rng(seed + 401)
        self._bias = math.radians(_value(config.gyro, "bias_deg_s"))
        self._last_time: float | None = None
        self._last_speed = 0.0

    def reset(self) -> None:
        self._bias = math.radians(_value(self.config.gyro, "bias_deg_s"))
        self._last_time = None
        self._last_speed = 0.0

    def sample(
        self,
        state: VehicleState,
        t: float,
        dt: float | None = None,
        realism_scale: float = 1.0,
    ) -> ImuSample:
        cfg = self.config
        scale = max(0.0, realism_scale)
        if dt is None:
            dt = 0.0 if self._last_time is None else t - self._last_time
        dt = max(0.0, min(dt, 0.25))

        walk = math.radians(_value(cfg.gyro, "random_walk_deg_s_sqrt_hz"))
        if dt > 0.0:
            self._bias += (
                float(self.rng.normal(0.0, walk * math.sqrt(dt))) * scale
            )
        bias = self._bias * scale

        true_yaw_rate = float(state.yaw_rate)
        scale_error = _value(cfg.gyro, "scale_error")
        cross_axis = _value(cfg.gyro, "cross_axis")
        misalignment = math.radians(cfg.mounting_misalignment_deg)
        noise_sigma = math.radians(_value(cfg.gyro, "noise_deg_s")) * scale

        yaw_rate = true_yaw_rate * (1.0 + scale_error * scale) + bias
        yaw_rate += float(self.rng.normal(0.0, noise_sigma))
        coupling = (cross_axis + misalignment) * scale * true_yaw_rate
        gyro_x = coupling + float(self.rng.normal(0.0, noise_sigma))
        gyro_y = -0.5 * coupling + float(self.rng.normal(0.0, noise_sigma))

        saturation = math.radians(_value(cfg.gyro, "saturation_deg_s", 500.0))
        gyro = (
            float(np.clip(gyro_x, -saturation, saturation)),
            float(np.clip(gyro_y, -saturation, saturation)),
            float(np.clip(yaw_rate, -saturation, saturation)),
        )

        longitudinal = (float(state.speed) - self._last_speed) / dt if dt > 0.0 else 0.0
        lateral = float(state.speed) * true_yaw_rate
        accel_bias = _value(cfg.accel, "bias_m_s2")
        accel_noise = _value(cfg.accel, "noise_m_s2")
        vibration = _value(cfg.accel, "vibration_m_s2")
        accel_sigma = (accel_noise + vibration) * scale
        accel_x = float(
            self.rng.normal(longitudinal + accel_bias * scale, accel_sigma)
        )
        accel_y = float(self.rng.normal(lateral + accel_bias * scale, accel_sigma))
        accel_z = float(self.rng.normal(9.81 + accel_bias * scale, accel_sigma))
        accel_saturation = _value(cfg.accel, "saturation_m_s2", 156.0)
        accel = (
            float(np.clip(accel_x, -accel_saturation, accel_saturation)),
            float(np.clip(accel_y, -accel_saturation, accel_saturation)),
            float(np.clip(accel_z, -accel_saturation, accel_saturation)),
        )

        self._last_time = t
        self._last_speed = float(state.speed)
        return ImuSample(
            gyro_rad_s=gyro,
            accel_m_s2=accel,
            t_measure=t,
            t_available=t + cfg.latency_ms / 1000.0 * scale,
            frame_id=cfg.frame_id,
        )


__all__ = ["ImuModel"]
