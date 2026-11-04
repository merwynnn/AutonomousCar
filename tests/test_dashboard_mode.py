from __future__ import annotations

from dashboard.server import DashboardSimulation


def test_neutral_drive_packets_do_not_force_manual_mode() -> None:
    simulation = DashboardSimulation()
    try:
        assert simulation.control_mode == "auto"
        # A stale manual tab keeps sending neutral packets; they must not
        # overwrite another client's Auto selection.
        simulation.handle_command(
            {"type": "drive.command", "throttle": 0.0, "steering": 0.0}
        )
        assert simulation.control_mode == "auto"

        simulation.handle_command(
            {"type": "drive.command", "throttle": 0.4, "steering": 0.0}
        )
        assert simulation.control_mode == "manual"

        simulation.handle_command({"type": "control.mode", "mode": "auto"})
        assert simulation.control_mode == "auto"
    finally:
        simulation.sim.close()
