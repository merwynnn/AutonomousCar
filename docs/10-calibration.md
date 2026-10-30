# Calibration against the real car

> This document becomes actionable when the physical car exists. Until then, the
> simulator uses generic defaults and must not be called a validated digital twin.

## 1. What to measure

### Vehicle geometry and mass

- length, width, height (competition limits: 45 × 20 × 20 cm);
- wheelbase, track width, wheel radius;
- ground clearance (≥5 mm);
- mass and centre of mass (front/rear, left/right, height);
- steering geometry (Ackermann, max angle, toe).

### Powertrain

- motor model, KV, max current, gear ratio;
- ESC settings and braking behaviour;
- battery voltage/capacity/internal resistance;
- measured top speed and acceleration on a straight;
- current draw vs speed.

### Tires and suspension

- tire compound and radius;
- suspension stiffness/damping (if adjustable);
- friction behaviour: straight-line braking distance, cornering limit;
- wheel slip under acceleration.

### Sensors

- lidar model, beams, rate, range, accuracy;
- lidar mount pose and height (critical: 16 cm walls);
- ultrasonic positions, beam width, range, rate;
- IMU model, rate, bias;
- encoder ticks/rev and wheel radius;
- measured latency of each sensor and the control loop.

## 2. Measurement procedures

### Straight-line test

- Mark a 5 m straight; measure time to cross at full throttle from rest;
- repeat 5 times; fit acceleration and top speed;
- log motor current if possible.

### Braking test

- Accelerate to a fixed speed, brake fully, measure stopping distance;
- repeat on the track surface;
- fit deceleration and brake lag.

### Cornering test

- Drive constant-radius circles at increasing speed;
- record the speed at which the car starts to slide;
- estimate lateral acceleration limit and tire friction.

### Lidar comparison

- Place the car at known poses in a simple room/track;
- record scans and compare range/angle against ground truth;
- estimate range bias, noise σ, angular offset and dropout rate.

### Latency measurement

- Command a step steering/throttle change and log when the car responds;
- measure sensor timestamp to controller receive time;
- measure control loop period and jitter.

### Odometry test

- Drive a known path (straight, circle, figure-eight);
- compare encoder odometry to ground truth;
- estimate wheel radius and track width errors, slip factors.

## 3. Mapping measurements to config

| Measurement | Config field |
|---|---|
| Geometry/mass | `configs/vehicles/<car>.yaml` |
| Motor/battery | `vehicle.powertrain.*` |
| Tires | `vehicle.tires.*` |
| Lidar | `configs/sensors/lidar.yaml` |
| Ultrasonic | `configs/sensors/ultrasonic.yaml` |
| IMU/encoders | `configs/sensors/imu.yaml`, `encoders.yaml` |
| Latency | `sensors.*.latency_ms`, `control.period_ms` |
| Noise/bias | `sensors.*.noise.*` |
| Sensor extrinsics | `sensors.*.extrinsic` |

## 4. Validation

- Replay a real log through the simulator’s SLAM; compare map and trajectory.
- Compare simulated and real 0–5 m acceleration, braking distance, turning radius.
- Compare lidar range statistics on the same scene.
- Run a 5-lap simulation with calibrated parameters and compare to a real run.
- Report remaining gaps; do not tune unrelated parameters to hide them.

## 5. Calibration checklist

- [ ] Car mass and geometry measured
- [ ] Motor/gear/battery parameters measured
- [ ] Straight-line acceleration and top speed matched
- [ ] Braking distance matched
- [ ] Cornering limit matched
- [ ] Lidar mount height/extrinsics measured
- [ ] Lidar noise/bias/dropout estimated
- [ ] Ultrasonic positions/beam measured
- [ ] IMU bias/noise estimated
- [ ] Encoder resolution and slip estimated
- [ ] Sensor/control latency measured
- [ ] Real sensor data compared with the simulated sensor model
- [ ] 5-lap sim vs real comparison documented
