from __future__ import annotations

from simulator.config import load_bundle
from simulator.simulation import DEFAULT_SCENARIO
from simulator.track import build_test_track, project_to_polyline


def test_scenario_bundle_resolves_vehicle_and_sensors() -> None:
    bundle = load_bundle(DEFAULT_SCENARIO)
    assert bundle.scenario.name == "test_track_baseline"
    assert bundle.vehicle.wheelbase_m > 0.0
    assert bundle.sensors.lidar.beams == 720
    assert bundle.scenario.simulation.control_hz == 60.0


def test_obstacles_are_on_track_and_clear_of_walls() -> None:
    track = build_test_track(width_m=0.9)
    wall_inner = track.width_m / 2.0 - 0.08
    assert len(track.obstacles) == 8
    for obstacle in track.obstacles:
        _, lateral, _ = project_to_polyline(track.centerline, (obstacle.x, obstacle.y))
        half_width = (
            obstacle.radius if obstacle.kind == "cylinder" else obstacle.width / 2.0
        )
        assert abs(lateral) + half_width < wall_inner
