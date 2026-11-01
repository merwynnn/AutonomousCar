"""Hardware-facing protocols for the onboard drivers.

The reference car maps them as follows:

- ``LidarDriver``      -> LD19/LD06 USB serial lidar on the Pi;
- ``SerialTransport``  -> RP2040/Teensy link carrying encoders, IMU, steering
  and ultrasonic packets plus actuator commands;
- ``ActuatorSink``     -> VESC throttle and steering servo through that link.

Only the protocols live here. Board-specific setup stays in the deployment
driver so simulation has no hardware dependency.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from onboard.api import LidarObservation


@runtime_checkable
class LidarDriver(Protocol):
    """A 2D lidar that can return the latest complete scan."""

    def read(self, now: float) -> LidarObservation | None:
        """Return the latest scan whose ``t_available`` is not in the future."""

    def close(self) -> None:
        """Release the serial port."""


@runtime_checkable
class SerialTransport(Protocol):
    """Non-blocking line-oriented serial link to the low-level board."""

    def readline(self) -> bytes | None:
        """Return one complete line, or ``None`` when no line is ready."""

    def write(self, data: bytes) -> None:
        """Queue bytes for transmission."""

    def close(self) -> None:
        """Close the port."""


__all__ = ["LidarDriver", "SerialTransport"]
