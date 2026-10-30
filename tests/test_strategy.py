from __future__ import annotations

from pathlib import Path

import numpy as np

from awake_sim.config import load_vehicle
from awake_sim.control.pure_pursuit import PurePursuitController
from awake_sim.control.racing_line import build_racing_line
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
