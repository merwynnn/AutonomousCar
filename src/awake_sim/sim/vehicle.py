"""Vehicle state and actuation types."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Actuation:
    """Normalised driver command."""

    throttle: float = 0.0  # -1 (full reverse) .. +1 (full forward)
    steering: float = 0.0  # radians, positive left


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
