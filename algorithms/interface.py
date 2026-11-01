"""Compatibility re-export of the portable onboard contract.

The real contract lives in :mod:`onboard.api`, which is the code that can run on
the Raspberry Pi. Algorithms may keep importing ``algorithms.interface`` for a
while, but new code should import ``onboard.api`` directly.
"""

from __future__ import annotations

from onboard.api import (
    REAR_WHEEL_INDICES,
    WHEEL_ORDER,
    Actuation,
    Controller,
    EncoderSample,
    ImuSample,
    LidarFrame,
    LidarObservation,
    Observation,
    Pose2,
    RobotAPI,
    SensorFrame,
    StateEstimate,
    SteeringSample,
    UltrasonicObservation,
    UltrasonicReading,
)

__all__ = [
    "Actuation",
    "Controller",
    "EncoderSample",
    "ImuSample",
    "LidarFrame",
    "LidarObservation",
    "Observation",
    "Pose2",
    "REAR_WHEEL_INDICES",
    "RobotAPI",
    "SensorFrame",
    "StateEstimate",
    "SteeringSample",
    "UltrasonicObservation",
    "UltrasonicReading",
    "WHEEL_ORDER",
]
