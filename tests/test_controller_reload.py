from __future__ import annotations

from dashboard.server import DashboardSimulation


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
