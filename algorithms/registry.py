"""Dashboard-selectable controller registry.

The registry lives beside the algorithms so a new controller is added in one
place and is immediately available to the dashboard. It contains no host or
physics imports.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from onboard.api import Actuation, Observation

from algorithms.gap_follow import FurthestGapController
from algorithms.learned_gap import LearnedGapController
from algorithms.wall_follow import WallFollowController


@dataclass(frozen=True)
class ControllerSpec:
    """Description and factory for one selectable controller."""

    id: str
    label: str
    factory: Callable[[], Any]

    def build(self) -> Any:
        return self.factory()

    def update(
        self,
        controller: Any,
        observation: Observation,
        speed_limit: float,
    ) -> Actuation:
        update = controller.update
        return update(observation, speed_limit)


DEFAULT_CONTROLLER = "gap_follow"

CONTROLLERS: dict[str, ControllerSpec] = {
    "gap_follow": ControllerSpec(
        id="gap_follow",
        label="Furthest gap",
        factory=FurthestGapController,
    ),
    "wall_follow": ControllerSpec(
        id="wall_follow",
        label="Wall follow",
        factory=WallFollowController,
    ),
    "learned_gap": ControllerSpec(
        id="learned_gap",
        label="Learned gap (CMA-ES)",
        factory=LearnedGapController,
    ),
}


def controller_list() -> list[dict[str, str]]:
    """Return the dashboard payload for the controller selector."""

    return [{"id": spec.id, "label": spec.label} for spec in CONTROLLERS.values()]


__all__ = [
    "CONTROLLERS",
    "DEFAULT_CONTROLLER",
    "ControllerSpec",
    "controller_list",
]
