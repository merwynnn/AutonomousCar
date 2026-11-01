"""Steering-angle sensor model."""

from __future__ import annotations

import math

import numpy as np
from onboard.api import SteeringSample

from simulator.config import SteeringSensorConfig
from simulator.vehicle import VehicleState


class SteeringSensorModel:
    def __init__(self, config: SteeringSensorConfig, seed: int = 0) -> None:
        self.config = config
        self.rng = np.random.default_rng(seed + 451)

    def sample(
        self,
        state: VehicleState,
        t: float,
        realism_scale: float = 1.0,
    ) -> SteeringSample:
        cfg = self.config
        scale = max(0.0, realism_scale)
        noise = math.radians(cfg.noise_deg) * scale
        quantum = math.radians(cfg.quantization_deg)
        measured = float(state.steering) + float(self.rng.normal(0.0, noise))
        if quantum > 0.0:
            measured = round(measured / quantum) * quantum
        return SteeringSample(
            angle_rad=measured,
            t_measure=t,
            t_available=t + cfg.latency_ms / 1000.0 * scale,
            frame_id=cfg.frame_id,
        )


__all__ = ["SteeringSensorModel"]
