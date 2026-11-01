"""Standalone AWAKE simulator core.

``simulator/`` is host-only physics, sensors, track and timing. The portable
contract, estimator and Pi runtime live in ``onboard/``; controllers in
``algorithms/`` import only ``onboard.api`` and NumPy.
"""

from simulator.bridge import SimRobotAPI
from simulator.simulation import Simulation, SimulationStatus

__all__ = ["SimRobotAPI", "Simulation", "SimulationStatus"]
