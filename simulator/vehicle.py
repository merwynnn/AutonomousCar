"""Vehicle state types.

``Actuation`` is defined in ``algorithms.interface`` so the simulator and a
controller exchange the exact same object. ``VehicleState`` is simulator-only
ground truth and is never given to an algorithm.
"""

from __future__ import annotations

from dataclasses import dataclass

from algorithms.interface import Actuation

__all__ = ["Actuation", "VehicleState"]


@dataclass
class VehicleState:
    x: float
    y: float
    z: float
    yaw: float
    speed: float  # signed body-forward speed, m/s
    yaw_rate: float
    steering: float  # actual front-wheel angle, rad
    wheel_speeds: tuple[float, float, float, float]
    wheel_slip: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
