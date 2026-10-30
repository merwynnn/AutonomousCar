"""Racing line and time-optimal speed profile."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from awake_sim.config import VehicleConfig
from awake_sim.tracks.geometry import TrackGeometry, project_to_polyline


def resample_closed(points: np.ndarray, spacing: float = 0.10) -> np.ndarray:
    """Resample a closed polyline at approximately constant arc-length spacing."""

    pts = np.asarray(points, dtype=float)
    closed = np.vstack([pts, pts[:1]])
    seg = np.diff(closed, axis=0)
    seg_len = np.linalg.norm(seg, axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = cumulative[-1]
    count = max(8, int(round(total / spacing)))
    targets = np.linspace(0.0, total, count, endpoint=False)
    out = np.zeros((count, 2))
    for i, target in enumerate(targets):
        idx = int(np.searchsorted(cumulative, target, side="right") - 1)
        idx = min(idx, len(seg) - 1)
        t = (target - cumulative[idx]) / max(seg_len[idx], 1e-9)
        out[i] = closed[idx] + t * seg[idx]
    return out


def closed_curvature(points: np.ndarray, spacing: float) -> np.ndarray:
    """Signed curvature of a closed polyline (positive = left turn)."""

    pts = np.asarray(points, dtype=float)
    nxt = np.roll(pts, -1, axis=0)
    prev = np.roll(pts, 1, axis=0)
    tangent_a = pts - prev
    tangent_b = nxt - pts
    angle_a = np.arctan2(tangent_a[:, 1], tangent_a[:, 0])
    angle_b = np.arctan2(tangent_b[:, 1], tangent_b[:, 0])
    dtheta = (angle_b - angle_a + np.pi) % (2.0 * np.pi) - np.pi
    return dtheta / max(spacing, 1e-6)


def _smooth_closed(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return values
    kernel = np.ones(window) / window
    padded = np.concatenate([values[-window:], values, values[:window]])
    smooth = np.convolve(padded, kernel, mode="same")
    return smooth[window:-window]


@dataclass
class RacingLine:
    points: np.ndarray
    curvature: np.ndarray
    speed_profile: np.ndarray
    spacing: float

    @property
    def length(self) -> float:
        return self.points.shape[0] * self.spacing

    def nearest_index(self, x: float, y: float) -> int:
        _, _, idx = project_to_polyline(self.points, (x, y))
        return idx

    def target_ahead(self, x: float, y: float, lookahead_m: float) -> tuple[np.ndarray, int]:
        idx = self.nearest_index(x, y)
        steps = max(1, int(round(lookahead_m / self.spacing)))
        return self.points[(idx + steps) % len(self.points)], idx


def build_racing_line(
    track: TrackGeometry,
    vehicle: VehicleConfig,
    spacing: float = 0.10,
    lateral_margin: float = 0.10,
    max_speed: float = 3.5,
    a_lat_max: float = 4.0,
    a_accel: float = 2.5,
    a_brake: float = 4.5,
) -> RacingLine:
    """Heuristic racing line: cut the inside of corners, then time-optimal speed.

    This is a deliberate first approximation, not a full minimum-curvature QP.
    It stays within the lane using the track half-width and a safety margin.
    """

    center = resample_closed(track.centerline, spacing)
    raw_curvature = closed_curvature(center, spacing)
    smoothed = _smooth_closed(raw_curvature, 9)
    tangent = np.roll(center, -1, axis=0) - np.roll(center, 1, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
    normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])

    max_offset = max(0.0, track.width_m / 2.0 - vehicle.dimensions.width_m / 2.0 - lateral_margin)
    curvature_ref = 1.0 / max(0.5, vehicle.wheelbase_m * 3.0)
    cut = np.sign(smoothed) * max_offset * np.clip(np.abs(smoothed) / curvature_ref, 0.0, 1.0)
    cut = _smooth_closed(cut, 15)
    points = center + normal * cut[:, None]

    curvature = closed_curvature(points, spacing)
    curvature = _smooth_closed(curvature, 5)
    v_curve = np.sqrt(a_lat_max / np.maximum(np.abs(curvature), 1e-4))
    speed = np.minimum(max_speed, np.minimum(v_curve, max_speed))
    speed = np.clip(speed, 0.8, max_speed)
    # Forward/backward passes around the closed loop for acceleration/braking.
    n = len(speed)
    for _ in range(3):
        for i in range(n):
            prev = speed[(i - 1) % n]
            speed[i] = min(speed[i], np.sqrt(prev * prev + 2.0 * a_accel * spacing))
        for i in range(n - 1, -1, -1):
            nxt = speed[(i + 1) % n]
            speed[i] = min(speed[i], np.sqrt(nxt * nxt + 2.0 * a_brake * spacing))
    return RacingLine(points=points, curvature=curvature, speed_profile=speed, spacing=spacing)


def speed_at(line: RacingLine, index: int) -> float:
    return float(line.speed_profile[index % len(line.speed_profile)])
