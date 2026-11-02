"""Real serial transport for the low-level board.

``pyserial`` is an optional dependency so the simulator and the Pi estimator
tests do not need a serial port. Install ``awake-sim-core[hardware]`` on the Pi.
"""

from __future__ import annotations

from typing import Any


class SerialPortTransport:
    """Non-blocking line transport backed by ``pyserial``."""

    def __init__(
        self,
        port: str,
        *,
        baudrate: int = 115200,
        timeout_s: float = 0.0,
    ) -> None:
        try:
            import serial
        except ImportError as exc:  # pragma: no cover - depends on optional extra
            raise RuntimeError(
                "pyserial is required for SerialPortTransport; install "
                "awake-sim-core[hardware]"
            ) from exc
        self._serial: Any = serial.Serial(port, baudrate=baudrate, timeout=timeout_s)

    def readline(self) -> bytes | None:
        line = self._serial.readline()
        return line or None

    def write(self, data: bytes) -> None:
        self._serial.write(data)

    def close(self) -> None:
        self._serial.close()


__all__ = ["SerialPortTransport"]
