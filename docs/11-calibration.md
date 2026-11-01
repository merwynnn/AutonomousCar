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
| Encoder resolution | Datasheet / one-wheel revolution | `encoders.ticks_per_rev` |
| Encoder distance scale | Push car 2–5 m, compare ticks | `encoders.wheel_radius_error` |
| Gyro bias | Stationary log, average yaw rate | `imu.gyro.bias_deg_s` |
| Gyro noise / walk | Allan deviation, stationary log | `imu.gyro.noise_deg_s`, `random_walk_deg_s_sqrt_hz` |

## Safe test manoeuvres

Run these at low battery current and with a spotter:

1. **Straight acceleration** from rest at 20 %, 40 %, 60 % throttle.
2. **Coast-down** from a fixed speed; fit rolling resistance and drag.
3. **Braking** from a fixed speed; fit brake torque and tire friction.
4. **Step steer** at low speed; fit steering rate, lag and deadband.
5. **Constant-radius turn**; fit tire slip and understeer.
6. **Lidar static test** facing a wall at known distances; fit noise and offset.
7. **Ultrasonic static test** at known distances and angles.

## Odometry calibration

1. Keep the car still for 5–10 s at boot and average the gyro yaw rate. Use that
   value as the initial bias and verify the estimator converges to it.
2. Push the car straight for a measured distance. Compare encoder distance with
   the tape; the fractional error is `encoders.wheel_radius_error`.
3. Drive a constant-radius turn. Compare wheel-derived yaw rate with the gyro;
   adjust `encoders.track_width_error` if the two disagree consistently.
4. Compare left/right wheel speeds during a low-speed turn. A large persistent
   split suggests a dragging brake, tyre pressure mismatch or encoder mounting
   problem, not a tuning issue.

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
