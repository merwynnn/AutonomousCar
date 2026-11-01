"""``RobotAPI`` adapter for the simulator.

The protocol lives in ``onboard.api``. The simulator is just one provider of it,
next to the Raspberry Pi sensor/actuator stack in ``onboard.runtime``.
"""

from __future__ import annotations

from pathlib import Path
from typing import Literal

from onboard.api import Actuation, Observation
from onboard.config import EstimatorTuning

from simulator.config import Realism, SensorsConfig, VehicleConfig
from simulator.simulation import Simulation, SimulationStatus
from simulator.track import TrackGeometry
from simulator.vehicle import VehicleState


class SimRobotAPI:
    """Expose a ``Simulation`` through the portable robot interface."""

    def __init__(self, simulation: Simulation) -> None:
        self.simulation = simulation

    @classmethod
    def create(
        cls,
        scenario_path: str | Path | None = None,
        *,
        gui: bool = False,
        lidar_subrays: int | None = None,
        seed: int | None = None,
        realism: Realism | None = None,
        vehicle: VehicleConfig | None = None,
        sensors: SensorsConfig | None = None,
        backend: Literal["pybullet", "mujoco"] = "mujoco",
        track: TrackGeometry | None = None,
        estimator_config: EstimatorTuning | None = None,
    ) -> SimRobotAPI:
        return cls(
            Simulation(
                scenario_path,
                gui=gui,
                lidar_subrays=lidar_subrays,
                seed=seed,
                realism=realism,
                vehicle=vehicle,
                sensors=sensors,
                backend=backend,
                track=track,
                estimator_config=estimator_config,
            )
        )

    def reset(self) -> Observation:
        return self.simulation.reset()

    def step(self, command: Actuation) -> Observation:
        return self.simulation.step(command)

    def close(self) -> None:
        self.simulation.close()

    def status(self) -> SimulationStatus:
        return self.simulation.status()

    def true_state(self) -> VehicleState:
        """Debug-only ground truth; never pass this to a controller."""

        return self.simulation.true_state()

    def __enter__(self) -> SimRobotAPI:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


__all__ = ["SimRobotAPI"]
