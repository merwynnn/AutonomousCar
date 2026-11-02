"""A protocol-compatible fake low-level board for hardware-free testing.

``MockBoard`` implements the same line-oriented transport as the real
RP2040/Teensy link. It turns actuator packets into a simple first-order vehicle
response and returns timestamped sensor packets, so ``SensorRobot`` and the
serial drivers can be exercised end to end without a car.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

from onboard.actuation import HardwareConfig, SteeringActuator


@dataclass
class MockBoard:
    """Fake serial board that speaks the JSON-lines protocol."""

    hardware: HardwareConfig = field(default_factory=HardwareConfig)
    wheelbase_m: float = 0.257
    wheel_radius_m: float = 0.035
    dt: float = 1.0 / 60.0
    max_accel_mps2: float = 4.0
    _time: float = 0.0
    _speed_mps: float = 0.0
    _steering_rad: float = 0.0
    _target_speed_mps: float = 0.0
    _encoder_ticks: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    _steering: SteeringActuator = field(init=False)
    _pending: bool = False
    _closed: bool = False

    def __post_init__(self) -> None:
        self._steering = SteeringActuator(
            self.hardware.steering,
            max_angle_rad=math.radians(28.0),
        )

    def write(self, data: bytes) -> None:
        text = data.decode("utf-8")
        for line in text.splitlines():
            if not line.strip():
                continue
            payload = json.loads(line)
            if payload.get("type") != "actuation":
                continue
            steering = payload.get("steering_rad", payload.get("steering", 0.0))
            self._steering_rad = self._steering.update(float(steering), self.dt)
            if "target_speed_mps" in payload:
                self._target_speed_mps = float(payload["target_speed_mps"])
            else:
                self._target_speed_mps = (
                    self.hardware.drive.sign
                    * float(payload.get("throttle", 0.0))
                    * self.hardware.drive.max_speed_mps
                )
            self._pending = True

    def readline(self) -> bytes | None:
        if self._closed or not self._pending:
            return None
        self._pending = False
        packet_time = self._time
        self._time += self.dt

        delta = self._target_speed_mps - self._speed_mps
        max_delta = self.max_accel_mps2 * self.dt
        self._speed_mps += max(-max_delta, min(max_delta, delta))

        wheel_speed = self._speed_mps / self.wheel_radius_m
        ticks_per_m = self._ticks_per_m()
        for index in range(4):
            self._encoder_ticks[index] += int(
                wheel_speed * self.wheel_radius_m * self.dt * ticks_per_m
            )

        yaw_rate = self._speed_mps * math.tan(self._steering_rad) / self.wheelbase_m
        motor = {
            "erpm": wheel_speed * 5.0 * 60.0 / (2.0 * math.pi),
            "current_a": abs(self._target_speed_mps)
            / max(1e-6, self.hardware.drive.max_speed_mps)
            * self.hardware.drive.current_limit_a,
            "duty": max(-1.0, min(1.0, self._target_speed_mps / self.hardware.drive.max_speed_mps)),
            "voltage_v": 7.4,
            "temperature_c": 25.0,
        }
        packet = {
            "type": "sensors",
            "t": packet_time,
            "encoders": {
                "ticks": self._encoder_ticks,
                "dt": self.dt,
            },
            "imu": {
                "gyro_rad_s": [0.0, 0.0, yaw_rate],
                "accel_m_s2": [0.0, 0.0, 9.81],
            },
            "steering": {"angle_rad": self._steering_rad},
            "motor": motor,
        }
        return (json.dumps(packet, separators=(",", ":")) + "\n").encode("utf-8")

    def _ticks_per_m(self) -> float:
        # Matches the reference encoder resolution and wheel radius.
        return 1024 / (2.0 * math.pi * self.wheel_radius_m)

    def close(self) -> None:
        self._closed = True


__all__ = ["MockBoard"]
