"""Shared state-estimator protocol."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from onboard.api import Pose2, SensorFrame, StateEstimate


@runtime_checkable
class StateEstimator(Protocol):
    """Consumes raw sensor frames and produces the fused observation state."""

    @property
    def state(self) -> StateEstimate:
        """Latest estimate."""

    def reset(self, initial_pose: Pose2) -> StateEstimate:
        """Reset the estimator at a known starting pose."""

    def update(self, sensors: SensorFrame, now: float) -> StateEstimate:
        """Fuse the latest available measurements and return the new estimate."""


__all__ = ["StateEstimator"]
