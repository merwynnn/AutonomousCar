"""2D track geometry helpers used by the demo server and, later, the generator.

Everything is in metres in the world frame (x forward, y left, z up).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

Point = tuple[float, float]


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    theta: float


@dataclass
class TrackGeometry:
    centerline: np.ndarray  # (N, 2), closed loop without duplicate endpoint
    inner: np.ndarray       # (N, 2), inner wall polyline
    outer: np.ndarray       # (N, 2), outer wall polyline
    width_m: float
    start_s: float
    bounds: tuple[float, float, float, float]

    @property
    def length(self) -> float:
        d = np.diff(np.vstack([self.centerline, self.centerline[:1]]), axis=0)
        return float(np.sum(np.linalg.norm(d, axis=1)))

    def segments(self) -> list[tuple[np.ndarray, np.ndarray]]:
        segs: list[tuple[np.ndarray, np.ndarray]] = []
        for loop in (self.inner, self.outer):
            for i in range(len(loop)):
                segs.append((loop[i], loop[(i + 1) % len(loop)]))
        return segs

    def sample(self, s: float) -> Pose2:
        """Sample the centreline by arc length; heading follows the tangent."""

        s = s % self.length
        seg = np.diff(np.vstack([self.centerline, self.centerline[:1]]), axis=0)
        seg_len = np.linalg.norm(seg, axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seg_len)])
        idx = int(np.searchsorted(cum, s, side="right") - 1)
        idx = min(idx, len(self.centerline) - 1)
        t = (s - cum[idx]) / max(seg_len[idx], 1e-9)
        p0 = self.centerline[idx]
        p1 = self.centerline[(idx + 1) % len(self.centerline)]
        p = p0 + t * (p1 - p0)
        d = p1 - p0
        theta = float(np.arctan2(d[1], d[0]))
        return Pose2(float(p[0]), float(p[1]), theta)


def catmull_rom_closed(points: list[Point] | np.ndarray, samples_per_segment: int = 40) -> np.ndarray:
    """Closed Catmull-Rom spline through the given control points."""

    pts = np.asarray(points, dtype=float)
    if len(pts) < 4:
        raise ValueError("need at least 4 control points")
    n = len(pts)
    out: list[np.ndarray] = []
    for i in range(n):
        p0 = pts[(i - 1) % n]
        p1 = pts[i]
        p2 = pts[(i + 1) % n]
        p3 = pts[(i + 2) % n]
        for j in range(samples_per_segment):
            t = j / samples_per_segment
            t2 = t * t
            t3 = t2 * t
            out.append(
                0.5
                * (
                    2.0 * p1
                    + (-p0 + p2) * t
                    + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
                    + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
                )
            )
    return np.asarray(out, dtype=float)


def offset_polyline(polyline: np.ndarray, offset: float) -> np.ndarray:
    """Offset a closed 2D polyline by a signed distance (left is positive)."""

    pts = np.asarray(polyline, dtype=float)
    nxt = np.roll(pts, -1, axis=0)
    prev = np.roll(pts, 1, axis=0)
    tangent = nxt - prev
    norm = np.linalg.norm(tangent, axis=1, keepdims=True)
    tangent = tangent / np.maximum(norm, 1e-9)
    normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    return pts + offset * normal


def peanut_control_points() -> list[Point]:
    """Control points for the PDF-like peanut test track (15 x 8 m area)."""

    # Two lobes connected by a narrower middle; start/finish at bottom centre.
    return [
        (12.2, 4.0),   # right end
        (11.6, 5.8),
        (9.8, 6.6),
        (7.5, 5.5),    # top waist
        (5.2, 6.6),
        (3.4, 5.8),
        (2.8, 4.0),    # left end
        (3.4, 2.2),
        (5.2, 1.4),
        (7.5, 2.5),    # bottom waist
        (9.8, 1.4),
        (11.6, 2.2),
    ]


def build_test_track(width_m: float = 0.9, samples_per_segment: int = 32) -> TrackGeometry:
    centerline = catmull_rom_closed(peanut_control_points(), samples_per_segment)
    inner = offset_polyline(centerline, width_m / 2.0)
    outer = offset_polyline(centerline, -width_m / 2.0)
    # start at the bottom-centre point nearest (7.5, 2.5)
    start_idx = int(np.argmin(np.linalg.norm(centerline - np.array([7.5, 2.5]), axis=1)))
    d = np.diff(np.vstack([centerline, centerline[:1]]), axis=0)
    seg_len = np.linalg.norm(d, axis=1)
    start_s = float(np.sum(seg_len[:start_idx]))
    return TrackGeometry(
        centerline=centerline,
        inner=inner,
        outer=outer,
        width_m=width_m,
        start_s=start_s,
        bounds=(0.0, 0.0, 15.0, 8.0),
    )


def project_to_polyline(
    points: np.ndarray, position: Point
) -> tuple[float, float, int]:
    """Project a point onto a closed polyline.

    Returns ``(s, lateral, index)`` where ``s`` is arc length along the
    centreline, ``lateral`` is the signed distance (left positive), and ``index``
    is the winning segment.
    """

    pts = np.asarray(points, dtype=float)
    nxt = np.roll(pts, -1, axis=0)
    seg = nxt - pts
    seg_len2 = np.maximum(np.sum(seg * seg, axis=1), 1e-12)
    rel = np.asarray(position, dtype=float) - pts
    t = np.clip(np.sum(rel * seg, axis=1) / seg_len2, 0.0, 1.0)
    projections = pts + t[:, None] * seg
    delta = np.asarray(position, dtype=float) - projections
    dist = np.linalg.norm(delta, axis=1)
    idx = int(np.argmin(dist))
    seg_len = np.sqrt(seg_len2)
    cumulative = np.concatenate([[0.0], np.cumsum(seg_len)])
    s = float(cumulative[idx] + t[idx] * seg_len[idx])
    cross = seg[idx, 0] * delta[idx, 1] - seg[idx, 1] * delta[idx, 0]
    lateral = float(cross / max(seg_len[idx], 1e-12))
    return s, lateral, idx


def _cross(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def raycast_segments(
    origin: Point,
    direction: Point,
    segments: list[tuple[np.ndarray, np.ndarray]],
    max_range: float,
) -> tuple[float, np.ndarray] | None:
    """Cast a 2D ray against segments; return (range, point) of the nearest hit."""

    o = np.asarray(origin, dtype=float)
    d = np.asarray(direction, dtype=float)
    d = d / max(np.linalg.norm(d), 1e-12)
    best_t = max_range
    best_p: np.ndarray | None = None
    for p0, p1 in segments:
        e = p1 - p0
        denom = _cross(d, e)
        if abs(denom) < 1e-12:
            continue
        diff = p0 - o
        t = _cross(diff, e) / denom
        u = _cross(diff, d) / denom
        if t >= 0.0 and 0.0 <= u <= 1.0 and t < best_t:
            best_t = t
            best_p = o + t * d
    if best_p is None:
        return None
    return best_t, best_p


def ramp_scan(
    track: TrackGeometry,
    pose: Pose2,
    beams: int,
    max_range: float,
    rng: np.random.Generator,
    noise_sigma_m: float = 0.012,
    dropout_probability: float = 0.01,
) -> np.ndarray:
    """Approximate scan for the demo server.

    Samples the wall polylines and bins them into beams. The real simulator uses
    PyBullet raycasts; this is only for the M0 dashboard demo.
    """

    wall_pts = np.vstack([track.inner, track.outer])
    rel = wall_pts - np.array([pose.x, pose.y])
    ranges = np.linalg.norm(rel, axis=1)
    angles = np.arctan2(rel[:, 1], rel[:, 0]) - pose.theta
    angles = (angles + np.pi) % (2.0 * np.pi) - np.pi
    valid = ranges < max_range
    ranges = ranges[valid]
    angles = angles[valid]

    beam_angles = np.linspace(-np.pi, np.pi, beams, endpoint=False)
    idx = np.floor((angles + np.pi) / (2.0 * np.pi) * beams).astype(int) % beams
    out = np.full(beams, np.nan)
    for i, r in zip(idx, ranges, strict=True):
        if np.isnan(out[i]) or r < out[i]:
            out[i] = r
    # noise, quantization and dropout
    out = out + rng.normal(0.0, noise_sigma_m, size=beams)
    out = np.round(out / 0.001) * 0.001
    drop = rng.random(beams) < dropout_probability
    out[drop] = np.nan
    out[(out < 0.03) | (out > max_range)] = np.nan
    return np.column_stack([beam_angles, out])
