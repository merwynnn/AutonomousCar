"""Shared actuator calibration and command shaping.

The simulator and the real car both use this module. It deliberately contains
no physics, no serial I/O and no simulator imports: it converts the portable
``Actuation`` intent into calibrated steering and drive targets, applying the
same limits and non-idealities on both sides of the sim-to-real boundary.

Units are SI except for the servo calibration, which is expressed in
microseconds because that is what the low-level board emits.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _number(data: dict[str, Any], key: str, default: float) -> float:
    value = data.get(key, default)
    if not isinstance(value, int | float):
        raise ValueError(f"expected a numeric '{key}'")
    return float(value)


@dataclass(frozen=True)
class SteeringCalibration:
    """Servo calibration and steering-linkage non-idealities."""

    center_us: float = 1500.0
    us_per_deg: float = 10.0
    sign: float = 1.0
    trim_deg: float = 0.0
    min_us: float = 1000.0
    max_us: float = 2000.0
    max_rate_deg_s: float = 90.0
    deadband_deg: float = 0.5
    backlash_deg: float = 0.5
    latency_ms: float = 18.0

    def __post_init__(self) -> None:
        if self.sign not in (-1.0, 1.0):
            raise ValueError("steering.sign must be +1 or -1")
        if self.us_per_deg <= 0.0:
            raise ValueError("steering.us_per_deg must be positive")
        if self.max_us <= self.min_us:
            raise ValueError("steering.max_us must exceed min_us")

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> SteeringCalibration:
        return cls(
            center_us=_number(data, "center_us", cls.center_us),
            us_per_deg=_number(data, "us_per_deg", cls.us_per_deg),
            sign=_number(data, "sign", cls.sign),
            trim_deg=_number(data, "trim_deg", cls.trim_deg),
            min_us=_number(data, "min_us", cls.min_us),
            max_us=_number(data, "max_us", cls.max_us),
            max_rate_deg_s=_number(data, "max_rate_deg_s", cls.max_rate_deg_s),
            deadband_deg=_number(data, "deadband_deg", cls.deadband_deg),
            backlash_deg=_number(data, "backlash_deg", cls.backlash_deg),
            latency_ms=_number(data, "latency_ms", cls.latency_ms),
        )

    def angle_to_servo_us(self, angle_rad: float) -> float:
        """Map a positive-left road-wheel angle to a servo pulse width."""

        angle_deg = math.degrees(angle_rad) - self.trim_deg
        pulse = self.center_us + self.sign * angle_deg * self.us_per_deg
        return _clamp(pulse, self.min_us, self.max_us)

    def servo_us_to_angle(self, pulse_us: float) -> float:
        """Inverse of :meth:`angle_to_servo_us`."""

        angle_deg = (pulse_us - self.center_us) / (self.sign * self.us_per_deg)
        return math.radians(angle_deg + self.trim_deg)


@dataclass(frozen=True)
class DriveCalibration:
    """Throttle-to-speed calibration and motor/ESC limits."""

    command_mode: str = "current"
    max_speed_mps: float = 4.0
    current_limit_a: float = 40.0
    throttle_deadband: float = 0.02
    sign: float = 1.0
    torque_lag_ms: float = 12.0
    speed_kp: float = 0.35
    speed_ki: float = 0.15

    def __post_init__(self) -> None:
        if self.sign not in (-1.0, 1.0):
            raise ValueError("drive.sign must be +1 or -1")
        if self.command_mode not in ("current", "duty", "erpm"):
            raise ValueError("drive.command_mode must be current, duty or erpm")
        if self.max_speed_mps <= 0.0:
            raise ValueError("drive.max_speed_mps must be positive")

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> DriveCalibration:
        return cls(
            command_mode=str(data.get("command_mode", cls.command_mode)),
            max_speed_mps=_number(data, "max_speed_mps", cls.max_speed_mps),
            current_limit_a=_number(data, "current_limit_a", cls.current_limit_a),
            throttle_deadband=_number(data, "throttle_deadband", cls.throttle_deadband),
            sign=_number(data, "sign", cls.sign),
            torque_lag_ms=_number(data, "torque_lag_ms", cls.torque_lag_ms),
            speed_kp=_number(data, "speed_kp", cls.speed_kp),
            speed_ki=_number(data, "speed_ki", cls.speed_ki),
        )


@dataclass(frozen=True)
class BoardConfig:
    """Low-level board safety settings."""

    watchdog_ms: float = 200.0

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> BoardConfig:
        return cls(watchdog_ms=_number(data, "watchdog_ms", cls.watchdog_ms))


@dataclass(frozen=True)
class HardwareConfig:
    """Everything the actuator layer needs on simulation and hardware."""

    protocol_version: int = 2
    calibration_status: str = "uncalibrated"
    steering: SteeringCalibration = field(default_factory=SteeringCalibration)
    drive: DriveCalibration = field(default_factory=DriveCalibration)
    board: BoardConfig = field(default_factory=BoardConfig)

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> HardwareConfig:
        steering = data.get("steering", {})
        drive = data.get("drive", {})
        board = data.get("board", {})
        return cls(
            protocol_version=int(data.get("protocol_version", cls.protocol_version)),
            calibration_status=str(data.get("calibration_status", cls.calibration_status)),
            steering=SteeringCalibration.from_mapping(
                steering if isinstance(steering, dict) else {}
            ),
            drive=DriveCalibration.from_mapping(drive if isinstance(drive, dict) else {}),
            board=BoardConfig.from_mapping(board if isinstance(board, dict) else {}),
        )


class SteeringActuator:
    """Rate-limited, delayed, deadband/backlash steering command shaper."""

    def __init__(
        self,
        calibration: SteeringCalibration,
        *,
        max_angle_rad: float,
    ) -> None:
        if max_angle_rad <= 0.0:
            raise ValueError("max_angle_rad must be positive")
        self.calibration = calibration
        self.max_angle_rad = max_angle_rad
        self._now = 0.0
        self._rate_target = 0.0
        self._output = 0.0
        self._initialized = False
        self._delayed_target = 0.0
        self._queue: deque[tuple[float, float]] = deque([(0.0, 0.0)])

    def reset(self) -> None:
        self._now = 0.0
        self._rate_target = 0.0
        self._output = 0.0
        self._initialized = False
        self._delayed_target = 0.0
        self._queue.clear()
        self._queue.append((0.0, 0.0))

    def update(self, command_rad: float, dt: float) -> float:
        """Return the road-wheel angle the servo should track now."""

        if dt <= 0.0:
            raise ValueError("dt must be positive")
        self._now += dt

        target = _clamp(command_rad, -self.max_angle_rad, self.max_angle_rad)
        max_step = math.radians(self.calibration.max_rate_deg_s) * dt
        self._rate_target += _clamp(
            target - self._rate_target,
            -max_step,
            max_step,
        )

        self._queue.append((self._now, self._rate_target))
        latency_s = self.calibration.latency_ms / 1000.0
        if latency_s <= 0.0:
            self._delayed_target = self._rate_target
            self._queue.clear()
            self._queue.append((self._now, self._rate_target))
        else:
            while (
                len(self._queue) > 1
                and self._now - self._queue[0][0] >= latency_s - 1e-9
            ):
                self._delayed_target = self._queue.popleft()[1]
        delayed = self._delayed_target

        if not self._initialized:
            self._output = delayed
            self._initialized = True
            return self._output

        deadband = math.radians(self.calibration.deadband_deg)
        if abs(delayed - self._output) <= deadband:
            return self._output

        backlash = math.radians(self.calibration.backlash_deg)
        delta = delayed - self._output
        if abs(delta) <= backlash:
            return self._output
        self._output = delayed - math.copysign(backlash, delta)
        return self._output

    def angle_to_servo_us(self, angle_rad: float) -> float:
        return self.calibration.angle_to_servo_us(angle_rad)

    def servo_us_to_angle(self, pulse_us: float) -> float:
        return self.calibration.servo_us_to_angle(pulse_us)


class DriveActuator:
    """Throttle-to-speed PI with battery sag and torque lag."""

    def __init__(
        self,
        calibration: DriveCalibration,
        *,
        wheel_radius_m: float,
    ) -> None:
        if wheel_radius_m <= 0.0:
            raise ValueError("wheel_radius_m must be positive")
        self.calibration = calibration
        self.wheel_radius_m = wheel_radius_m
        self._integral = 0.0
        self._torque = 0.0

    def reset(self) -> None:
        self._integral = 0.0
        self._torque = 0.0

    def target_speed_mps(self, throttle: float) -> float:
        command = _clamp(throttle, -1.0, 1.0)
        if abs(command) < self.calibration.throttle_deadband:
            return 0.0
        return self.calibration.sign * command * self.calibration.max_speed_mps

    def update(
        self,
        throttle: float,
        *,
        measured_wheel_speed_rad_s: float,
        max_torque_per_wheel_nm: float,
        battery_voltage_v: float,
        battery_resistance_ohm: float,
        dt: float,
    ) -> float:
        """Return the torque command for one driven wheel."""

        if dt <= 0.0:
            raise ValueError("dt must be positive")
        target = self.target_speed_mps(throttle)
        measured = measured_wheel_speed_rad_s * self.wheel_radius_m
        error = target - measured
        self._integral = _clamp(self._integral + error * dt, -2.0, 2.0)

        current_demand = abs(throttle) * self.calibration.current_limit_a
        voltage_factor = _clamp(
            (battery_voltage_v - current_demand * battery_resistance_ohm)
            / battery_voltage_v,
            0.5,
            1.0,
        )
        torque = (
            self.calibration.speed_kp * error
            + self.calibration.speed_ki * self._integral
        ) * voltage_factor
        torque = _clamp(torque, -max_torque_per_wheel_nm, max_torque_per_wheel_nm)

        tau = self.calibration.torque_lag_ms / 1000.0
        alpha = dt / (tau + dt) if tau > 0.0 else 1.0
        self._torque += alpha * (torque - self._torque)
        return self._torque


__all__ = [
    "BoardConfig",
    "DriveActuator",
    "DriveCalibration",
    "HardwareConfig",
    "SteeringActuator",
    "SteeringCalibration",
]
