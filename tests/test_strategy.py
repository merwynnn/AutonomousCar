from __future__ import annotations

from pathlib import Path

import numpy as np

from awake_sim.config import load_vehicle
from awake_sim.control.gap_follow import FurthestGapController
from awake_sim.control.pure_pursuit import PurePursuitController
from awake_sim.control.racing_line import build_racing_line
from awake_sim.sensors.lidar import LidarScan
from awake_sim.tracks.geometry import build_test_track, project_to_polyline

ROOT = Path(__file__).resolve().parents[1]


def test_racing_line_stays_inside_track() -> None:
    vehicle = load_vehicle(ROOT / "configs" / "vehicles" / "generic_1_10_4wd.yaml")
    track = build_test_track()
    line = build_racing_line(track, vehicle)
    assert len(line.points) > 150
    assert line.speed_profile.min() >= 0.8
    assert line.speed_profile.max() <= 3.5
    assert np.all(np.isfinite(line.curvature))
    # Every racing-line point must remain inside the lane (with a margin).
    max_lateral = 0.0
    for x, y in line.points:
        _, lateral, _ = project_to_polyline(track.centerline, (float(x), float(y)))
        max_lateral = max(max_lateral, abs(lateral))
    assert max_lateral <= track.width_m / 2.0 - vehicle.dimensions.width_m / 2.0
    assert abs(line.length - track.length) < track.length * 0.15


def test_pure_pursuit_outputs_valid_actuation() -> None:
    vehicle = load_vehicle(ROOT / "configs" / "vehicles" / "generic_1_10_4wd.yaml")
    track = build_test_track()
    line = build_racing_line(track, vehicle)
    controller = PurePursuitController(line, wheelbase_m=vehicle.wheelbase_m)
    pose = track.sample(track.start_s)
    action = controller.update(pose.x, pose.y, pose.theta, 1.0)
    assert -0.55 <= action.steering <= 0.55
    assert 0.0 <= action.throttle <= 1.0


def _gap_scan(center_rad: float, width_rad: float = 1.2, beams: int = 720) -> LidarScan:
    angles = np.linspace(-np.pi, np.pi, beams, endpoint=False)
    ranges = np.full(beams, 0.45)
    ranges[np.abs(angles - center_rad) < width_rad / 2.0] = 3.0
    return LidarScan(
        angles=angles,
        ranges=ranges,
        intensities=np.full(beams, 100.0),
        valid=np.ones(beams, dtype=bool),
        t_measure=0.0,
        t_available=0.0,
    )


def test_furthest_gap_steers_toward_the_open_side() -> None:
    controller = FurthestGapController()
    action = controller.update(_gap_scan(0.35), 1.0 / 60.0, 2.4)
    assert controller.debug.valid
    assert 0.15 < controller.debug.angle_rad < 0.50
    assert controller.debug.clearance_m > 2.0
    assert action.steering > 0.0
    assert action.throttle > 0.3


def test_furthest_gap_clamps_steering_and_reports_no_return() -> None:
    controller = FurthestGapController()
    controller.update(_gap_scan(1.2), 1.0 / 60.0, 2.4)
    assert abs(controller.debug.angle_rad) <= controller.max_steering

    beams = 720
    invalid = LidarScan(
        angles=np.linspace(-np.pi, np.pi, beams, endpoint=False),
        ranges=np.full(beams, np.nan),
        intensities=np.zeros(beams),
        valid=np.zeros(beams, dtype=bool),
        t_measure=0.0,
        t_available=0.0,
    )
    action = controller.update(invalid, 1.0 / 60.0, 2.4)
    assert not controller.debug.valid
    assert action.throttle == 0.0
