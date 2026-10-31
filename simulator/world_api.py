"""Protocol implemented by every physics backend's world."""

from __future__ import annotations

from typing import Protocol

from simulator.track import Pose2
from simulator.vehicle import Actuation, VehicleState


class World(Protocol):
    """Minimal world interface used by the simulator and sensor models."""

    dt: float
    wall_id_set: set[int]

    def reset(self, pose: Pose2) -> None: ...

    def set_actuation(self, actuation: Actuation) -> None: ...

    def step(self) -> None: ...

    def state(self) -> VehicleState: ...

    def wheel_positions(self) -> list[tuple[float, float, float]]: ...

    def raycast_batch(
        self,
        origins: list[tuple[float, float, float]],
        angles: list[float],
        max_range: float,
    ) -> list[tuple[float, tuple[float, float, float], tuple[float, float, float], int] | None]: ...

    def raycast_vectors(
        self,
        origins: list[tuple[float, float, float]],
        directions: list[tuple[float, float, float]],
        max_range: float,
    ) -> list[tuple[float, tuple[float, float, float], tuple[float, float, float], int] | None]: ...

    def close(self) -> None: ...
