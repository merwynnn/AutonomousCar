"""Standalone AWAKE simulator core.

The deployable folders are ``simulator/`` (physics, sensors, track, timing) and
``algorithms/`` (the sensor-only controller contract and your controllers).
"""

from simulator.simulation import Simulation, SimulationStatus

__all__ = ["Simulation", "SimulationStatus"]
