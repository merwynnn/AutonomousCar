"""Simplified VESC-class motor telemetry for the MuJoCo digital twin."""

from __future__ import annotations

import math

from onboard.api import MotorSample

from simulator.config import VehicleConfig
from simulator.vehicle import VehicleState


class MotorModel:
    """Produce plausible ERPM/current/duty/voltage telemetry from the world state.

    This is deliberately a measurement model, not a second motor model: the
    physical torque comes from the MuJoCo actuator and the shared
    ``onboard.actuation.DriveActuator``. The telemetry exists so controllers can
    detect saturation, battery sag and a stalled drivetrain.
    """

    def __init__(
        self,
        vehicle: VehicleConfig,
        *,
        seed: int = 0,
        latency_ms: float = 10.0,
    ) -> None:
        self.vehicle = vehicle
        self.latency_ms = latency_ms
        self._seed = seed

    def sample(
        self,
        state: VehicleState,
        throttle: float,
        now: float,
        *,
        realism_scale: float = 1.0,
    ) -> MotorSample:
        motor = self.vehicle.motor
        driven = state.wheel_speeds if self.vehicle.drive == "4wd" else state.wheel_speeds[2:]
        omega = sum(driven) / max(1, len(driven))
        max_omega = motor.max_rpm * 2.0 * math.pi / 60.0 / motor.gear_ratio
        speed_fraction = min(1.0, abs(omega) / max(1e-6, max_omega))

        current = (
            abs(throttle)
            * motor.current_limit_a
            * (0.35 + 0.65 * (1.0 - speed_fraction))
        )
        current *= max(0.0, min(1.0, realism_scale))
        voltage = max(
            0.5 * self.vehicle.battery.nominal_voltage_v,
            self.vehicle.battery.nominal_voltage_v
            - current * self.vehicle.battery.internal_resistance_ohm,
        )
        duty = max(
            -1.0,
            min(1.0, throttle * (0.2 + 0.8 * speed_fraction)),
        )
        temperature = 25.0 + 20.0 * min(1.0, current / max(1.0, motor.current_limit_a))
        return MotorSample(
            erpm=omega * motor.gear_ratio * 60.0 / (2.0 * math.pi),
            current_a=current,
            duty=duty,
            voltage_v=voltage,
            temperature_c=temperature,
            t_measure=now,
            t_available=now + self.latency_ms / 1000.0 * max(0.0, realism_scale),
        )


__all__ = ["MotorModel"]
