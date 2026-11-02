from __future__ import annotations

import json
import math

import numpy as np
from algorithms.debug import (
    DebugFrame,
    LabelOverlay,
    Plot,
    PointMarker,
    PointsOverlay,
    PolylineOverlay,
    RayOverlay,
    Readout,
    SectorOverlay,
    Series,
)
from algorithms.gap_follow import FurthestGapController
from algorithms.registry import CONTROLLERS, DEFAULT_CONTROLLER, controller_list
from onboard.api import LidarObservation, Observation, Pose2, SensorFrame, StateEstimate
from simulator.simulation import Simulation


def _observation(lidar: LidarObservation | None, t: float = 1.0) -> Observation:
    estimate = StateEstimate(
        t=t,
        pose=Pose2(0.0, 0.0, 0.0),
        speed_mps=0.0,
        yaw_rate_rad_s=0.0,
        covariance=np.eye(3),
        valid=True,
    )
    return Observation(
        t=t,
        dt=0.02,
        sensors=SensorFrame(lidar=lidar),
        state=estimate,
    )


def test_gap_debug_frame_exposes_candidates_and_score() -> None:
    simulation = Simulation(lidar_subrays=1, realism="ideal")
    try:
        observation = simulation.reset()
        controller = FurthestGapController()
        for _ in range(4):
            observation = simulation.step(controller.update(observation, 1.2))
        frame = controller.debug_frame()
        payload = frame.to_dict()
        json.dumps(payload)

        assert payload["controller"] == "gap_follow"
        assert payload["valid"]
        assert len(payload["plots"]) == 2
        assert [table["id"] for table in payload["tables"]] == ["candidates"]
        assert {overlay["kind"] for overlay in payload["overlays"]} >= {
            "sector",
            "polyline",
            "ray",
            "points",
        }

        candidates = controller.debug.candidates
        assert len(candidates) >= 3
        assert candidates[0].selected
        assert [candidate.score for candidate in candidates] == sorted(
            (candidate.score for candidate in candidates),
            reverse=True,
        )
        for candidate in candidates:
            assert math.isclose(
                candidate.score,
                candidate.clearance_m - candidate.centering_penalty_m,
                abs_tol=1e-6,
            )
        table = payload["tables"][0]
        assert table["selected_row"] == 0
        assert table["rows"][0][4] == round(candidates[0].score, 4)
    finally:
        simulation.close()


def test_gap_debug_reports_missing_front_returns() -> None:
    controller = FurthestGapController()
    lidar = LidarObservation(
        angles_rad=np.array([0.0, 0.1]),
        ranges_m=np.array([np.nan, np.nan]),
        intensities=np.zeros(2),
        valid=np.zeros(2, dtype=bool),
        t_measure=1.0,
        t_available=1.01,
    )
    controller.update(_observation(lidar))
    frame = controller.debug_frame()
    payload = frame.to_dict()

    assert not controller.debug.valid
    assert controller.debug.status == "NO FRONT"
    assert not payload["valid"]
    assert payload["status"] == "NO FRONT"
    assert payload["plots"] == []
    assert payload["tables"] == []


def test_gap_debug_reports_missing_lidar() -> None:
    controller = FurthestGapController()
    controller.update(_observation(None))
    frame = controller.debug_frame()

    assert controller.debug.status == "NO SCAN"
    assert not frame.valid
    assert frame.status == "NO SCAN"


def test_debug_frame_serialises_all_overlay_primitives() -> None:
    frame = DebugFrame(
        controller="test",
        t=1.25,
        title="Test debug",
        readouts=(Readout("value", "Value", 1.23456, "m", 2),),
        plots=(
            Plot(
                id="plot",
                title="Plot",
                series=(
                    Series(
                        "series",
                        "series",
                        ((0.0, 1.0), (1.0, 2.0)),
                    ),
                ),
            ),
        ),
        overlays=(
            PointsOverlay(points=(PointMarker(1.0, 2.0, "#1", "candidate"),)),
            PolylineOverlay(points=((0.0, 0.0), (1.0, 1.0))),
            RayOverlay(angle_rad=0.2, range_m=2.0),
            SectorOverlay(start_angle_rad=-0.2, end_angle_rad=0.2, range_m=1.5),
            LabelOverlay(position=(0.5, 0.5), text="label"),
        ),
        meta={"sample": np.array([1.0, 2.0])},
    )
    payload = frame.to_dict()
    json.dumps(payload)

    assert payload["readouts"][0]["value"] == 1.2346
    assert payload["meta"]["sample"] == [1.0, 2.0]
    assert [overlay["kind"] for overlay in payload["overlays"]] == [
        "points",
        "polyline",
        "ray",
        "sector",
        "label",
    ]


def test_controller_registry_is_dashboard_ready() -> None:
    assert DEFAULT_CONTROLLER in CONTROLLERS
    payload = controller_list()
    assert {item["id"] for item in payload} == set(CONTROLLERS)
    for spec in CONTROLLERS.values():
        controller = spec.build()
        assert hasattr(controller, "reset")
        assert hasattr(controller, "update")
