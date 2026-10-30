"""Non-ideal 2D spinning lidar model backed by PyBullet raycasts."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from awake_sim.config import LidarConfig
from awake_sim.sim.vehicle import VehicleState
from awake_sim.sim.world import PyBulletWorld


@dataclass
class LidarScan:
    angles: np.ndarray
    ranges: np.ndarray  # NaN for invalid beams
    intensities: np.ndarray
    valid: np.ndarray
    t_measure: float
    t_available: float
    frame_id: str = "lidar"

    @property
    def beam_count(self) -> int:
        return int(self.angles.size)

    def valid_count(self) -> int:
        return int(np.count_nonzero(self.valid))


class LidarModel:
    """A 2D lidar with noise, dropouts, divergence, motion distortion and latency."""

    def __init__(
        self,
        config: LidarConfig,
        world: PyBulletWorld,
        seed: int = 0,
    ) -> None:
        self.config = config
        self.world = world
        self.rng = np.random.default_rng(seed + 101)
        self.dead_rng = np.random.default_rng(seed + 102)
        self.dropped_scan_rng = np.random.default_rng(seed + 103)
        self.beams = config.beams
        self.angles = np.linspace(-math.pi, math.pi, self.beams, endpoint=False)
        subrays = int(config.divergence.get("subrays", 3))
        half_angle = math.radians(float(config.divergence.get("half_angle_deg", 0.15)))
        self.sub_offsets = np.linspace(-half_angle, half_angle, max(1, subrays))
        dead_fraction = float(config.dead_beams.get("fraction", 0.0))
        self.dead_mask = self.dead_rng.random(self.beams) < dead_fraction
        mount = config.mount.get("position_m", [0.10, 0.0, 0.14])
        self.mount_x = float(mount[0])
        self.mount_y = float(mount[1])
        self.mount_height = float(mount[2])

    def should_drop_scan(self, scale: float) -> bool:
        probability = float(self.config.dropped_scan_probability) * scale
        return bool(self.dropped_scan_rng.random() < probability)

    def scan(
        self,
        state: VehicleState,
        t: float,
        realism_scale: float = 1.0,
    ) -> LidarScan:
        cfg = self.config
        scale = max(0.0, realism_scale)
        period = 1.0 / cfg.rate_hz
        indices = np.arange(self.beams)
        # Each beam is captured at a different time during the rotating scan.
        beam_dt = (indices / self.beams) * period
        yaw_at = state.yaw + state.yaw_rate * beam_dt
        x_at = state.x + math.cos(state.yaw) * state.speed * beam_dt
        y_at = state.y + math.sin(state.yaw) * state.speed * beam_dt
        lidar_z = state.z + (self.mount_height - 0.09)
        origin_x = x_at + np.cos(yaw_at) * self.mount_x - np.sin(yaw_at) * self.mount_y
        origin_y = y_at + np.sin(yaw_at) * self.mount_x + np.cos(yaw_at) * self.mount_y

        origins: list[tuple[float, float, float]] = []
        ray_angles: list[float] = []
        for i in range(self.beams):
            for offset in self.sub_offsets:
                origins.append((float(origin_x[i]), float(origin_y[i]), float(lidar_z)))
                ray_angles.append(float(yaw_at[i] + self.angles[i] + offset))

        hits = self.world.raycast_batch(origins, ray_angles, cfg.max_range_m)
        sub_count = len(self.sub_offsets)
        ranges = np.full((self.beams, sub_count), np.inf)
        normals = np.zeros((self.beams, sub_count, 3))
        objects = np.full((self.beams, sub_count), -1, dtype=int)
        for k, hit in enumerate(hits):
            if hit is None:
                continue
            beam = k // sub_count
            sub = k % sub_count
            ranges[beam, sub] = hit[0]
            normals[beam, sub] = hit[2]
            objects[beam, sub] = hit[3]

        best = np.argmin(ranges, axis=1)
        best_range = ranges[np.arange(self.beams), best]
        best_normal = normals[np.arange(self.beams), best]
        best_object = objects[np.arange(self.beams), best]
        hit_mask = np.isfinite(best_range)

        # Incidence angle between the ray and the surface normal.
        ray_dir = np.column_stack(
            [
                np.cos(yaw_at + self.angles),
                np.sin(yaw_at + self.angles),
                np.zeros(self.beams),
            ]
        )
        cos_incidence = np.abs(np.sum(ray_dir * best_normal, axis=1))
        cos_incidence = np.clip(cos_incidence, 0.0, 1.0)

        ranges_meas = np.full(self.beams, np.nan)
        intensities = np.zeros(self.beams)
        safe_range = np.where(hit_mask, best_range, 0.0)
        sigma = (
            cfg.noise.range_sigma_m
            + cfg.noise.range_sigma_per_m * safe_range
        ) * scale
        noise = self.rng.normal(0.0, 1.0, self.beams) * sigma
        ranges_meas[hit_mask] = (
            best_range[hit_mask] * (1.0 + cfg.noise.range_scale_error * scale)
            + cfg.noise.range_bias_m * scale
            + noise[hit_mask]
        )
        quantum = float(cfg.quantization.get("range_m", 0.001))
        if quantum > 0:
            ranges_meas = np.round(ranges_meas / quantum) * quantum

        base_drop = float(cfg.dropout.get("base_probability", 0.0))
        max_drop = float(cfg.dropout.get("max_probability", 0.9))
        incidence_threshold = math.radians(
            float(cfg.dropout.get("incidence_threshold_deg", 70.0))
        )
        # Smooth increase of dropout near grazing incidence.
        grazing = np.clip(
            (np.arccos(cos_incidence) - incidence_threshold) / max(1e-6, math.pi / 2 - incidence_threshold),
            0.0,
            1.0,
        )
        range_factor = np.clip(safe_range / cfg.max_range_m, 0.0, 1.0)
        drop_probability = np.clip(
            (base_drop + 0.25 * grazing + 0.05 * range_factor) * scale,
            0.0,
            max_drop,
        )
        drop = self.rng.random(self.beams) < drop_probability
        drop |= self.dead_mask
        drop |= ~hit_mask
        ranges_meas[drop] = np.nan

        reflectivity = np.where(
            np.isin(best_object, list(self.world.wall_id_set)),
            0.55,
            0.40,
        )
        intensity_signal = (
            reflectivity
            * np.power(cos_incidence, 0.5)
            * np.exp(-safe_range / 15.0)
            * 255.0
        )
        intensity_noise = self.rng.normal(0.0, 2.0 * scale, self.beams)
        intensities = np.clip(intensity_signal + intensity_noise, 0.0, 255.0)
        intensities[drop | ~hit_mask] = 0.0

        latency = (cfg.latency_ms + self.rng.uniform(0.0, cfg.latency_jitter_ms)) / 1000.0
        return LidarScan(
            angles=self.angles.copy(),
            ranges=ranges_meas,
            intensities=intensities,
            valid=~np.isnan(ranges_meas),
            t_measure=t,
            t_available=t + latency * scale,
            frame_id=cfg.frame_id,
        )
