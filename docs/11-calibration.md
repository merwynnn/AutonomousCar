# Calibration and validation

A realistic simulator is defined by how well it predicts the real car, not by
the renderer. With limited real-car time, use datasheet values first, then fit a
small number of measurable parameters.

## Parameters to measure

| Parameter | How to measure | Simulator field |
|---|---|---|
| Total mass | Kitchen scale | `mass_kg` |
| Corner masses / COM | Four scales + balance point | `center_of_mass_m`, structure |
| Wheel radius | Caliper | `wheel_radius_m` |
| Wheelbase / track | Tape measure | `wheelbase_m`, `track_width_m` |
| Steering angle | Protractor + servo command sweep | `steering.max_angle_deg` |
| Steering rate/lag | Step command + camera or encoder | `steering.rate_deg_s`, `latency_ms` |
| Motor torque/current | VESC telemetry + bench load | `motor` |
| Top speed / gearing | Straight-line test | `motor.max_rpm`, `gear_ratio` |
| Rolling resistance | Coast-down | `tires.rolling_resistance` |
| Tire friction | Skid-pad or low-speed turn | `tires.friction_front/rear` |
| Lidar mount height | Caliper | `lidar.mount.position_m[2]` |
| Sensor latency | Timestamp comparison | `latency_ms` |

## Safe test manoeuvres

Run these at low battery current and with a spotter:

1. **Straight acceleration** from rest at 20 %, 40 %, 60 % throttle.
2. **Coast-down** from a fixed speed; fit rolling resistance and drag.
3. **Braking** from a fixed speed; fit brake torque and tire friction.
4. **Step steer** at low speed; fit steering rate, lag and deadband.
5. **Constant-radius turn**; fit tire slip and understeer.
6. **Lidar static test** facing a wall at known distances; fit noise and offset.
7. **Ultrasonic static test** at known distances and angles.

## Simulator validation protocol

For each manoeuvre, store:

- real timestamped command and sensor data;
- simulator command replay;
- pose/speed/yaw-rate overlay;
- error metrics (RMSE, final position error, time-to-speed).

Accept a parameter set only if it stays within a declared tolerance across all
manoeuvres, not just one.

## Uncertainty and Monte Carlo

Because real testing is limited, carry parameter uncertainty explicitly:

- mass ±5 %;
- tire friction ±15 %;
- motor torque/current ±10 %;
- steering latency ±10 ms;
- lidar range noise/bias as measured;
- wall friction and track shape randomized.

Run batches over these ranges and report the distribution of lap outcomes. A
policy that completes 5 laps under uncertainty is more valuable than one that is
fast only in the nominal simulation.

## What to validate first

For this competition, prioritize:

1. **Completing 5 laps without wall contact.**
2. **Lidar seeing the 16 cm walls at the chosen mount height.**
3. **Steering and braking response at low speed.**
4. **Robustness to track shape and obstacle placement.**

Top-speed accuracy is secondary until the car is reliable.
