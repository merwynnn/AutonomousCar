"""Non-ideal ultrasonic sensor model (three sensors by default)."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from simulator.config import (
    UltrasonicConfig,
    mount_position,
    mount_rpy_deg,
)
from simulator.transforms import rpy_matrix, yaw_matrix
from simulator.vehicle import VehicleState
from simulator.world import PyBulletWorld


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
            position = mount_position(sensor.mount)
            roll, pitch, yaw_deg = mount_rpy_deg(sensor.mount)
            body_rotation = yaw_matrix(state.yaw)
            sensor_rotation = body_rotation @ rpy_matrix(
                math.radians(roll), math.radians(pitch), math.radians(yaw_deg)
            )
            origin = np.array([state.x, state.y, state.z]) + (
                body_rotation @ np.array(position, dtype=float)
            )
            origins = [
                (float(origin[0]), float(origin[1]), float(origin[2]))
            ] * self.ray_offsets.size
            directions = []
            for offset in self.ray_offsets:
                angle = float(offset)
                direction = sensor_rotation @ np.array(
                    [math.cos(angle), math.sin(angle), 0.0]
                )
                directions.append(
                    (float(direction[0]), float(direction[1]), float(direction[2]))
                )
            hits = self.world.raycast_vectors(origins, directions, cfg.max_range_m)
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
