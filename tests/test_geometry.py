from __future__ import annotations

import numpy as np

from awake_sim.tracks.geometry import (
    build_test_track,
    catmull_rom_closed,
    offset_polyline,
    ramp_scan,
    raycast_segments,
)


def test_catmull_rom_is_closed_and_smooth() -> None:
    pts = catmull_rom_closed([(0, 0), (1, 0), (1, 1), (0, 1)], samples_per_segment=10)
    assert pts.shape == (40, 2)
    assert np.all(np.isfinite(pts))
    assert np.linalg.norm(pts[0] - pts[-1]) > 0.0  # closed by wrapping, not duplicate


def test_offset_polyline() -> None:
    square = np.array([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)])
    out = offset_polyline(square, 0.1)
    assert out.shape == square.shape
    assert np.all(np.isfinite(out))


def test_test_track_geometry() -> None:
    track = build_test_track(width_m=0.9)
    assert track.centerline.shape[0] > 100
    assert track.length > 15.0
    assert track.inner.shape == track.outer.shape == track.centerline.shape
    assert track.bounds == (0.0, 0.0, 15.0, 8.0)
    # inner/outer should be separated by roughly the track width
    sep = np.linalg.norm(track.inner - track.outer, axis=1)
    assert np.median(sep) == np.median(sep)  # finite
    assert 0.7 < float(np.median(sep)) < 1.2


def test_sample_wraps() -> None:
    track = build_test_track()
    a = track.sample(0.0)
    b = track.sample(track.length)
    assert abs(a.x - b.x) < 1e-6
    assert abs(a.y - b.y) < 1e-6


def test_raycast_against_wall() -> None:
    wall = [(np.array([1.0, -1.0]), np.array([1.0, 1.0]))]
    hit = raycast_segments((0.0, 0.0), (1.0, 0.0), wall, max_range=5.0)
    assert hit is not None
    rng, point = hit
    assert abs(rng - 1.0) < 1e-9
    assert np.allclose(point, [1.0, 0.0])


def test_raycast_miss() -> None:
    wall = [(np.array([1.0, -1.0]), np.array([1.0, 1.0]))]
    assert raycast_segments((0.0, 0.0), (-1.0, 0.0), wall, max_range=5.0) is None


def test_ramp_scan_has_noise_and_dropouts() -> None:
    track = build_test_track()
    pose = track.sample(track.start_s)
    rng = np.random.default_rng(1)
    scan = ramp_scan(track, pose, beams=360, max_range=12.0, rng=rng)
    assert scan.shape == (360, 2)
    finite = np.isfinite(scan[:, 1])
    assert finite.sum() > 50
    assert (~finite).sum() > 0
    assert np.all(scan[finite, 1] > 0)
