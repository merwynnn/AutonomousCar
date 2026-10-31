"""Sensor-only contract between the simulator and a driving algorithm.

The simulator imports this module. Algorithms import only this module (and
NumPy), never the PyBullet world, the track geometry or any ground-truth pose.
This keeps a controller portable to the real car.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class Pose2:
    """Planar pose in metres and radians."""

    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class Actuation:
    """Normalised command sent to the car."""

    throttle: float = 0.0  # -1 (reverse) .. +1 (forward)
    steering: float = 0.0  # radians, positive left


@dataclass(frozen=True)
class LidarObservation:
    """One 2D lidar scan in the sensor frame."""

    angles_rad: np.ndarray
    ranges_m: np.ndarray  # NaN for invalid beams
    intensities: np.ndarray
    valid: np.ndarray
    t_measure: float
    t_available: float
    frame_id: str = "lidar"

    @property
    def beam_count(self) -> int:
        return int(self.angles_rad.size)


@dataclass(frozen=True)
class UltrasonicObservation:
    """One ultrasonic range reading."""

    name: str
    range_m: float | None
    valid: bool
    t_measure: float
    t_available: float


@dataclass(frozen=True)
class Observation:
    """Everything an algorithm is allowed to know at one control tick.

    There is deliberately no ground-truth pose, no future scan and no map. The
    only pose is the drifting wheel/IMU odometry estimate.
    """

    t: float
    dt: float
    odometry: Pose2
    speed_mps: float
    yaw_rate_rad_s: float
    lidar: LidarObservation | None
    ultrasonic: tuple[UltrasonicObservation, ...]


class Controller(Protocol):
    """Implement this protocol to drive the car."""

    def reset(self) -> None:
        """Called once before the first update."""

    def update(self, observation: Observation) -> Actuation:
        """Return a command for this control tick."""


class RobotAPI(Protocol):
    """The simulator and a future hardware adapter both implement this."""

    def reset(self) -> Observation:
        """Reset the robot and return the first observation."""

    def step(self, command: Actuation) -> Observation:
        """Apply a command for one control period and return the next observation."""

    def close(self) -> None:
        """Release hardware/simulation resources."""
