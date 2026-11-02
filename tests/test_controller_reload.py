from __future__ import annotations

import importlib

import pytest
from dashboard.server import DashboardSimulation
from onboard.api import Actuation, Observation


def test_controller_reload_rebuilds_instances_without_resetting_the_run() -> None:
    simulation = DashboardSimulation()
    try:
        simulation.paused = False
        for _ in range(3):
            simulation.step_once()
        run_time = simulation.sim.sim_time
        previous = simulation.active_controller

        ok, message = simulation.reload_controllers()

        assert ok, message
        assert simulation.sim.sim_time == run_time
        assert simulation.auto_controller == "gap_follow"
        assert simulation.active_controller is not previous
        assert (
            simulation.active_controller.__class__.__name__
            == previous.__class__.__name__
        )

        simulation.step_once()
        assert simulation.sim.sim_time > run_time
    finally:
        simulation.sim.close()


def test_controller_reload_command_is_queued() -> None:
    simulation = DashboardSimulation()
    try:
        simulation.handle_command({"type": "controller.reload"})
        assert simulation._controller_reload_pending
    finally:
        simulation.sim.close()


def test_import_error_pauses_without_killing_the_simulation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    simulation = DashboardSimulation()
    try:
        simulation.paused = False
        previous = simulation.active_controller
        real_reload = importlib.reload

        def failing_reload(module: object) -> object:
            name = getattr(module, "__name__", "")
            if name == "algorithms.gap_follow":
                raise SyntaxError("simulated algorithm syntax error")
            return real_reload(module)  # type: ignore[arg-type]

        monkeypatch.setattr(importlib, "reload", failing_reload)
        ok, message = simulation.reload_controllers()

        assert not ok
        assert "algorithms.gap_follow" in message
        assert simulation.paused
        assert simulation.controller_error == message
        assert simulation.active_controller is previous

        monkeypatch.undo()
        ok, message = simulation.reload_controllers()
        assert ok, message
        assert simulation.controller_error is None
        assert simulation.active_controller is not previous
    finally:
        simulation.sim.close()


def test_controller_runtime_error_pauses_instead_of_crashing() -> None:
    class BrokenController:
        def reset(self) -> None:
            pass

        def update(
            self,
            observation: Observation,
            speed_limit: float | None = None,
        ) -> Actuation:
            raise RuntimeError("boom")

    simulation = DashboardSimulation()
    try:
        simulation.controllers["gap_follow"] = BrokenController()
        simulation.paused = False

        simulation.step_once()

        assert simulation.paused
        assert simulation.controller_error is not None
        assert "Controller error" in simulation.controller_error
        assert "boom" in simulation.controller_error
        assert simulation.state_message()["controller_error"] == simulation.controller_error
    finally:
        simulation.sim.close()


def test_debug_frame_error_is_contained() -> None:
    class BadDebugController:
        def reset(self) -> None:
            pass

        def update(
            self,
            observation: Observation,
            speed_limit: float | None = None,
        ) -> Actuation:
            return Actuation()

        def debug_frame(self) -> object:
            raise RuntimeError("debug exploded")

    simulation = DashboardSimulation()
    try:
        simulation.controllers["gap_follow"] = BadDebugController()
        message = simulation.state_message()

        assert message["debug"]["status"] == "DEBUG ERROR"
        assert "debug exploded" in message["debug"]["meta"]["detail"]
        assert message["controller_error"] is None
    finally:
        simulation.sim.close()
