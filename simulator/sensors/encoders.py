"""Encoder measurement model.

The physics wheel joints report angular speed. The encoder counts wheel rotation
and the shared estimator converts ticks back to metres using the nominal wheel
radius. Calibration errors therefore become real odometry error, exactly like
the car.
"""

from __future__ import annotations

import math

import numpy as np
from onboard.api import EncoderSample

from simulator.config import EncoderConfig
from simulator.vehicle import VehicleState

_TWO_PI = 2.0 * math.pi


class EncoderModel:
    def __init__(
        self,
        config: EncoderConfig,
        wheel_radius_m: float,
        seed: int = 0,
    ) -> None:
        self.config = config
        self.wheel_radius_m = wheel_radius_m
        self.rng = np.random.default_rng(seed + 301)
        self._ticks = np.zeros(4, dtype=np.int64)
        self._tick_accumulator = np.zeros(4, dtype=float)
        self._last_time: float | None = None

    def reset(self, t: float = 0.0) -> None:
        self._ticks = np.zeros(4, dtype=np.int64)
        self._tick_accumulator = np.zeros(4, dtype=float)
        self._last_time = t

    def sample(
        self,
        state: VehicleState,
        t: float,
        realism_scale: float = 1.0,
    ) -> EncoderSample:
        cfg = self.config
        scale = max(0.0, realism_scale)
        if self._last_time is None:
            self._last_time = t
        dt = t - self._last_time
        if dt <= 0.0:
            self._last_time = t
            return EncoderSample(
                ticks=(
                    int(self._ticks[0]),
                    int(self._ticks[1]),
                    int(self._ticks[2]),
                    int(self._ticks[3]),
                ),
                wheel_speeds_mps=(0.0, 0.0, 0.0, 0.0),
                t_measure=t,
                t_available=t,
                dt=0.0,
                frame_id="encoders",
            )
        self._last_time = t

        true_speeds = np.array(
            [speed * self.wheel_radius_m for speed in state.wheel_speeds],
            dtype=float,
        )
        mean_speed = float(np.mean(true_speeds))
        # A track-width calibration error scales the left/right differential
        # without changing the mean forward speed.
        differential_scale = 1.0 + cfg.track_width_error * scale
        measured = mean_speed + (true_speeds - mean_speed) * differential_scale
        # Positive wheel_radius_error means the encoder under-reports distance.
        measured = measured / (1.0 + cfg.wheel_radius_error * scale)

        jitter = (
            float(self.rng.uniform(0.0, cfg.timestamp_jitter_ms)) / 1000.0 * scale
        )
        t_available = t + cfg.latency_ms / 1000.0 * scale + jitter

        dropped = bool(self.rng.random() < cfg.dropout_probability * scale)
        if dropped:
            return EncoderSample(
                ticks=(
                    int(self._ticks[0]),
                    int(self._ticks[1]),
                    int(self._ticks[2]),
                    int(self._ticks[3]),
                ),
                wheel_speeds_mps=None,
                t_measure=t,
                t_available=t_available,
                dt=dt,
                frame_id="encoders",
                dropped=True,
            )

        increments = measured * dt / self.wheel_radius_m / _TWO_PI * cfg.ticks_per_rev
        self._tick_accumulator += increments
        rounded = np.rint(self._tick_accumulator).astype(np.int64)
        self._ticks += rounded
        self._tick_accumulator -= rounded

        return EncoderSample(
            ticks=(
                int(self._ticks[0]),
                int(self._ticks[1]),
                int(self._ticks[2]),
                int(self._ticks[3]),
            ),
            wheel_speeds_mps=(
                float(measured[0]),
                float(measured[1]),
                float(measured[2]),
                float(measured[3]),
            ),
            t_measure=t,
            t_available=t_available,
            dt=dt,
            frame_id="encoders",
        )


__all__ = ["EncoderModel"]
