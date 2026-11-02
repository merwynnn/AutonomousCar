# Simulation and vehicle model

## Fixed-step loop

- Physics: 240 Hz by default (`simulation.physics_hz`).
- Control: 60 Hz by default; each control tick is four physics steps.
- Sensor sampling is scheduled in control ticks from each sensor's configured rate.
- A run is deterministic for a fixed scenario, config and seed.

`Simulation.step(command)` applies the actuation, advances physics, samples the
raw sensors, runs the shared state estimator, updates lap progress and returns the
next `Observation`.

## Vehicle model

`VehicleConfig` drives everything:

- overall dimensions and competition limits;
- `structure`: body parts with shape, position, rotation, size, colour and
  collision flag;
- mass, centre of mass, wheelbase, track width;
- wheel radius, width, mass and vertical offset;
- drive layout, motor, battery, tires, suspension, aero and steering limits.

`simulator/mjcf.py` turns that description into the physics model loaded by
MuJoCo. The dashboard uses the same object, so wheel joints, collision geometry
and the car studio cannot silently disagree.
Physics applications:

- chassis mass and inertia from the configured mass and collision bounds;
- centre of mass from `center_of_mass_m`;
- wheel joint positions from wheelbase/track width/vertical offset;
- front/rear tire friction from `tires`;
- motor speed/torque limits and 2WD/4WD drive layout;
- steering angle and rate limits;

`steering.rate_deg_s` is a command slew-rate limit. It should match the rate
limiter in the real low-level firmware, not the servo's no-load datasheet speed.
A loaded servo usually follows the command slew limit more repeatably than its
open-loop speed specification.
- rolling resistance and aero damping.

## Frames

World: x forward, y left, z up. Body: x forward, y left, z up, origin at the
chassis centre. Sensor mounts use the body frame. MuJoCo reports the body-frame
pose directly, so raycast origins and sensor heights use the same convention as
the vehicle definition.

## Physics world

- **MuJoCo** is the digital twin. It uses a generated MJCF with contact wheels,
  steering hinges, position-servo actuators and per-wheel torque motors.
- It implements the `World` protocol in `simulator/world_api.py`, so the same
  lidar, ultrasonic, encoder, IMU, steering and dashboard code runs against it.

## State estimation

`simulator/simulation.py` produces raw sensor frames and feeds them to
`onboard.estimator.WheelImuEstimator`, the same estimator used on the Pi:

- rear-wheel encoder ticks -> forward speed and wheel-derived yaw rate;
- gyro yaw rate with online bias correction;
- steering angle -> kinematic yaw-rate sanity check;
- zero-velocity update when the car is still;
- slip flag when wheel and gyro yaw rates disagree.

Estimator tuning is in `configs/estimator.yaml`. Wheel radius, wheelbase,
track width and encoder resolution come from the shared vehicle/sensor YAML.

## Lap timing

Progress is projected onto the track centreline. A lap is armed only after the
car has travelled at least half a lap so a small roll at the start line does not
count. Race completion is based on the configured lap count.
