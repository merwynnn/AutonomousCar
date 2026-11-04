"""Procedural random track generator for the dashboard.

The competition track shape is unknown. These tracks are smooth closed loops
inside the 15 x 8 m arena, with a configurable obstacle density. The default
test track remains available separately.
"""

from __future__ import annotations

import math

import numpy as np

from simulator.track import Obstacle, TrackGeometry, offset_polyline

ARENA_WIDTH_M = 15.0
ARENA_HEIGHT_M = 8.0
ARENA_CENTER = (ARENA_WIDTH_M / 2.0, ARENA_HEIGHT_M / 2.0)
WALL_RADIUS_M = 0.08


def _start_distance(centerline: np.ndarray) -> float:
    """Start on the straightest section, away from sharp random corners."""

    previous = np.roll(centerline, 1, axis=0)
    following = np.roll(centerline, -1, axis=0)
    incoming = centerline - previous
    outgoing = following - centerline
    incoming_len = np.maximum(np.linalg.norm(incoming, axis=1, keepdims=True), 1e-9)
    outgoing_len = np.maximum(np.linalg.norm(outgoing, axis=1, keepdims=True), 1e-9)
    incoming = incoming / incoming_len
    outgoing = outgoing / outgoing_len
    turn = np.abs(
        incoming[:, 0] * outgoing[:, 1] - incoming[:, 1] * outgoing[:, 0]
    )
    idx = int(np.argmin(turn))
    segment_lengths = np.linalg.norm(
        np.diff(np.vstack([centerline, centerline[:1]]), axis=0), axis=1
    )
    return float(segment_lengths[:idx].sum())


def build_random_track(
    seed: int = 42,
    width_m: float = 0.9,
    object_density: float = 0.25,
    samples: int = 280,
    object_scale: float = 1.25,
    object_scale_jitter: float = 0.30,
) -> TrackGeometry:
    """Generate a smooth closed-loop track inside the competition arena.

    ``object_density`` is 0..1: 0 gives an empty track, 1 gives the maximum
    number of crates/poles. ``object_scale`` scales the nominal crate/pole
    size; ``object_scale_jitter`` (0..1) randomises each object by that
    fraction, so a track mixes small and noticeably larger obstacles. Objects
    sit just inside the wall and are clamped so their footprint stays on the
    track.
    """

    width_m = max(0.8, min(1.0, float(width_m)))
    object_density = max(0.0, min(1.0, float(object_density)))
    object_scale = max(0.5, min(2.0, float(object_scale)))
    object_scale_jitter = max(0.0, min(1.0, float(object_scale_jitter)))
    rng = np.random.default_rng(seed)
    angles = np.linspace(0.0, 2.0 * math.pi, samples, endpoint=False)
    radius_x = 4.8 + rng.uniform(-0.25, 0.25)
    radius_y = 2.35 + rng.uniform(-0.15, 0.15)
    phase = rng.uniform(0.0, 2.0 * math.pi, size=3)
    amplitude = rng.uniform(0.035, 0.095, size=3)
    warp = (
        1.0
        + amplitude[0] * np.sin(2.0 * angles + phase[0])
        + amplitude[1] * np.sin(3.0 * angles + phase[1])
        + amplitude[2] * np.sin(5.0 * angles + phase[2])
    )
    centerline = np.column_stack(
        [
            ARENA_CENTER[0] + radius_x * np.cos(angles) * warp,
            ARENA_CENTER[1] + radius_y * np.sin(angles) * warp,
        ]
    )
    inner = offset_polyline(centerline, width_m / 2.0)
    outer = offset_polyline(centerline, -width_m / 2.0)

    # Keep the pipe walls inside the 15 x 8 m floor with a small maintenance
    # margin around the outside.
    all_points = np.vstack([inner, outer])
    min_x, min_y = all_points.min(axis=0)
    max_x, max_y = all_points.max(axis=0)
    margin = 0.25
    shift_x = 0.0
    shift_y = 0.0
    if min_x < margin:
        shift_x += margin - min_x
    if max_x + shift_x > ARENA_WIDTH_M - margin:
        shift_x += ARENA_WIDTH_M - margin - (max_x + shift_x)
    if min_y < margin:
        shift_y += margin - min_y
    if max_y + shift_y > ARENA_HEIGHT_M - margin:
        shift_y += ARENA_HEIGHT_M - margin - (max_y + shift_y)
    if shift_x or shift_y:
        shift = np.array([shift_x, shift_y])
        centerline = centerline + shift
        inner = inner + shift
        outer = outer + shift

    count = int(round(object_density * 12.0))
    obstacles: list[Obstacle] = []
    if count:
        fractions = np.sort(rng.uniform(0.06, 0.94, size=count))
        tangents = np.roll(centerline, -1, axis=0) - np.roll(centerline, 1, axis=0)
        tangents /= np.maximum(np.linalg.norm(tangents, axis=1, keepdims=True), 1e-9)
        normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])
        wall_inner = width_m / 2.0 - WALL_RADIUS_M
        for index, fraction in enumerate(fractions):
            point_index = int((fraction * len(centerline)) % len(centerline))
            side = -1.0 if index % 2 == 0 else 1.0
            scale = object_scale * (
                1.0 + rng.uniform(-object_scale_jitter, object_scale_jitter)
                if object_scale_jitter
                else 1.0
            )
            if index % 3 == 0:
                length = 0.13 * scale
                width = 0.06 * scale
                height = 0.20 * scale
                lateral_half = width / 2.0
                radius = 0.0
                name = f"crate_{index:02d}"
                kind = "box"
            else:
                length = 0.0
                width = 0.0
                height = 0.28 * scale
                radius = 0.025 * scale
                lateral_half = radius
                name = f"pole_{index:02d}"
                kind = "cylinder"
            # Keep the full footprint inside the clear lane, not just centre.
            max_offset = max(0.05, wall_inner - lateral_half - 0.01)
            desired = max(0.1, wall_inner - rng.uniform(0.08, 0.16))
            offset = side * max(0.05, min(desired, max_offset))
            x = float(centerline[point_index, 0] + normals[point_index, 0] * offset)
            y = float(centerline[point_index, 1] + normals[point_index, 1] * offset)
            yaw = float(math.atan2(tangents[point_index, 1], tangents[point_index, 0]))
            obstacles.append(
                Obstacle(
                    name=name,
                    kind=kind,
                    x=x,
                    y=y,
                    yaw=yaw,
                    length=length,
                    width=width,
                    height=height,
                    radius=radius,
                )
            )

    return TrackGeometry(
        centerline=centerline,
        inner=inner,
        outer=outer,
        width_m=width_m,
        start_s=_start_distance(centerline),
        bounds=(0.0, 0.0, ARENA_WIDTH_M, ARENA_HEIGHT_M),
        obstacles=obstacles,
    )
