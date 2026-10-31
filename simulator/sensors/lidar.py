"""Non-ideal 2D spinning lidar model backed by PyBullet raycasts."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from simulator.config import LidarConfig, mount_position, mount_rpy_deg
from simulator.transforms import rpy_matrix, yaw_matrix
from simulator.vehicle import VehicleState
from simulator.world import PyBulletWorld


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
        direction = -1.0 if config.scan_direction == "clockwise" else 1.0
        self.angles = direction * np.linspace(-math.pi, math.pi, self.beams, endpoint=False)
        subrays = int(config.divergence.get("subrays", 3))
        half_angle = math.radians(float(config.divergence.get("half_angle_deg", 0.15)))
        self.sub_offsets = np.linspace(-half_angle, half_angle, max(1, subrays))
        dead_fraction = float(config.dead_beams.get("fraction", 0.0))
        self.dead_mask = self.dead_rng.random(self.beams) < dead_fraction
        self.mount_position = np.array(mount_position(config.mount), dtype=float)
        self.mount_rotation = rpy_matrix(*np.radians(mount_rpy_deg(config.mount)))
        self.angle_offset = math.radians(config.noise.angle_offset_deg)

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
        z_at = np.full(self.beams, state.z)

        start_body_rotation = yaw_matrix(state.yaw)
        start_origin = np.array([state.x, state.y, state.z]) + (
            start_body_rotation @ self.mount_position
        )
        start_sensor_rotation = start_body_rotation @ self.mount_rotation

        origins: list[tuple[float, float, float]] = []
        ray_directions: list[tuple[float, float, float]] = []
        for i in range(self.beams):
            body_rotation = yaw_matrix(float(yaw_at[i]))
            sensor_rotation = body_rotation @ self.mount_rotation
            origin = np.array([x_at[i], y_at[i], z_at[i]]) + (
                body_rotation @ self.mount_position
            )
            for offset in self.sub_offsets:
                angle = float(self.angles[i] + offset)
                direction = sensor_rotation @ np.array(
                    [math.cos(angle), math.sin(angle), 0.0]
                )
                origins.append((float(origin[0]), float(origin[1]), float(origin[2])))
                ray_directions.append(
                    (float(direction[0]), float(direction[1]), float(direction[2]))
                )

        hits = self.world.raycast_vectors(origins, ray_directions, cfg.max_range_m)
        sub_count = len(self.sub_offsets)
        ranges = np.full((self.beams, sub_count), np.inf)
        normals = np.zeros((self.beams, sub_count, 3))
        objects = np.full((self.beams, sub_count), -1, dtype=int)
        points = np.zeros((self.beams, sub_count, 3))
        directions = np.array(ray_directions, dtype=float)
        for k, hit in enumerate(hits):
            if hit is None:
                continue
            beam = k // sub_count
            sub = k % sub_count
            ranges[beam, sub] = hit[0]
            normals[beam, sub] = hit[2]
            objects[beam, sub] = hit[3]
            points[beam, sub] = hit[1]

        best = np.argmin(ranges, axis=1)
        best_range = ranges[np.arange(self.beams), best]
        best_normal = normals[np.arange(self.beams), best]
        best_object = objects[np.arange(self.beams), best]
        hit_mask = np.isfinite(best_range)
        # Publish the scan in the scan-start lidar frame. A real spinning lidar
        # measures each beam at a slightly different pose; transforming the hit
        # point back here is what creates motion distortion for a downstream
        # estimator to handle. The mount RPY is applied here too, so a tilted
        # sensor changes ranges and angles.
        scan_angles = self.angles.copy()
        for i in range(self.beams):
            if not hit_mask[i]:
                continue
            relative = points[i, best[i]] - start_origin
            local = start_sensor_rotation.T @ relative
            scan_angles[i] = math.atan2(local[1], local[0])

        # Incidence angle between the ray and the surface normal.
        best_direction = directions[np.arange(self.beams) * sub_count + best]
        cos_incidence = np.abs(np.sum(best_direction * best_normal, axis=1))
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

        angle_noise = self.rng.normal(
            0.0, math.radians(cfg.noise.angle_sigma_deg) * scale, self.beams
        )
        scan_angles = scan_angles + angle_noise + self.angle_offset
        latency = (cfg.latency_ms + self.rng.uniform(0.0, cfg.latency_jitter_ms)) / 1000.0
        return LidarScan(
            angles=scan_angles,
            ranges=ranges_meas,
            intensities=intensities,
            valid=~np.isnan(ranges_meas),
            t_measure=t,
            t_available=t + latency * scale,
            frame_id=cfg.frame_id,
        )
