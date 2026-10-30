"""Non-ideal ultrasonic sensor model (three sensors by default)."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from awake_sim.config import UltrasonicConfig
from awake_sim.sim.vehicle import VehicleState
from awake_sim.sim.world import PyBulletWorld


@dataclass
class UltrasonicReading:
    name: str
    range_m: float | None
    valid: bool
    t_measure: float
    t_available: float


class UltrasonicModel:
    def __init__(
        self,
        config: UltrasonicConfig,
        world: PyBulletWorld,
        seed: int = 0,
    ) -> None:
        self.config = config
        self.world = world
        self.rng = np.random.default_rng(seed + 201)
        self.ray_offsets = np.linspace(
            -math.radians(config.cone_angle_deg) / 2.0,
            math.radians(config.cone_angle_deg) / 2.0,
            max(1, config.rays),
        )

    def sample(
        self,
        state: VehicleState,
        t: float,
        realism_scale: float = 1.0,
    ) -> list[UltrasonicReading]:
        cfg = self.config
        scale = max(0.0, realism_scale)
        readings: list[UltrasonicReading] = []
        for sensor in cfg.sensors:
            mount = sensor.mount
            position = mount.get("position_m", [0.0, 0.0, 0.08])
            rpy = mount.get("rpy_deg", [0.0, 0.0, 0.0])
            local_yaw = math.radians(float(rpy[2]))
            x = state.x + math.cos(state.yaw) * position[0] - math.sin(state.yaw) * position[1]
            y = state.y + math.sin(state.yaw) * position[0] + math.cos(state.yaw) * position[1]
            z = state.z + (float(position[2]) - 0.09)
            origins = [(x, y, z)] * self.ray_offsets.size
            angles = [
                state.yaw + local_yaw + float(offset) for offset in self.ray_offsets
            ]
            hits = self.world.raycast_batch(origins, angles, cfg.max_range_m)
            ranges = [hit[0] for hit in hits if hit is not None]
            if not ranges:
                reading_range: float | None = None
            else:
                true_range = float(min(ranges))
                sigma = (cfg.range_sigma_m + cfg.range_sigma_ratio * true_range) * scale
                measured_value = true_range + self.rng.normal(0.0, sigma)
                measured_value = round(measured_value / 0.01) * 0.01
                if measured_value < cfg.min_range_m:
                    measured_value = cfg.min_range_m
                # Specular losses and occasional cross-talk spikes.
                specular = cfg.specular_dropout_probability * scale
                if self.rng.random() < specular:
                    measured: float | None = None
                elif self.rng.random() < cfg.cross_talk_probability * scale:
                    measured = float(self.rng.uniform(0.05, 0.4))
                else:
                    measured = measured_value
                reading_range = None if measured is None else float(measured)
            latency = cfg.latency_ms / 1000.0 * scale
            readings.append(
                UltrasonicReading(
                    name=sensor.name,
                    range_m=reading_range,
                    valid=reading_range is not None,
                    t_measure=t,
                    t_available=t + latency,
                )
            )
        return readings
