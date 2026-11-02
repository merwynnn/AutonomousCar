# ADR-003 — Normalized intent, calibrated actuator layer

## Status

Accepted — 2026-11-02.

## Context

The controller must not depend on PWM microseconds, VESC duty or a particular
servo. At the same time, the simulator and the real car must interpret the same
`Actuation` command consistently. Previously the MuJoCo world applied its own
throttle mapping while the serial path forwarded an uncalibrated throttle and
steering angle to firmware that was not in this repository.

## Decision

- `Actuation.throttle` is a normalized target speed in `[-1, +1]` relative to
  `configs/hardware.yaml: drive.max_speed_mps`.
- `Actuation.steering` is a desired road-wheel angle in radians, positive left.
- `onboard/actuation.py` owns clamping, trim, sign, rate limit, deadband,
  backlash, command latency, battery sag, speed PI and torque lag.
- `configs/hardware.yaml` is the single source for servo/VESC/board calibration.
- The MuJoCo world and the Pi serial path both use the same model.
- `Observation.limits` exposes the static steering/speed limits to controllers;
  controllers must not hard-code them.
- `SensorFrame.motor` exposes VESC-class telemetry as a measurement, not ground
  truth.

## Consequences

- A controller tuned in simulation sees the same command semantics on the car.
- Servo/VESC differences are calibration inputs, not algorithm inputs.
- `hardware.yaml` starts as `uncalibrated`; the dashboard warns until it is
  measured.
- The first real-car session must fit the calibration values and rerun the same
  replay/acceptance tests.
