# Simulation and vehicle model

## Fixed-step loop

- Physics: 240 Hz by default (`simulation.physics_hz`).
- Control: 60 Hz by default; each control tick is four physics steps.
- Sensor sampling is scheduled in control ticks from each sensor's configured rate.
- A run is deterministic for a fixed scenario, config and seed.

`Simulation.step(command)` applies the actuation, advances physics, updates
odometry, schedules lidar/ultrasonic samples, updates lap progress and returns the
next `Observation`.

## Vehicle model

`VehicleConfig` drives everything:

- overall dimensions and competition limits;
- `structure`: body parts with shape, position, rotation, size, colour and
  collision flag;
- mass, centre of mass, wheelbase, track width;
- wheel radius, width, mass and vertical offset;
- drive layout, motor, battery, tires, suspension, aero and steering limits.

`simulator/car_urdf.py` and `simulator/mjcf.py` turn that description into the
physics models loaded by the PyBullet and MuJoCo backends. The dashboard uses the
same object, so wheel joints, collision geometry and the car studio cannot
silently disagree.
Physics applications:

- chassis mass and inertia from the configured mass and collision bounds;
- centre of mass from `center_of_mass_m`;
- wheel joint positions from wheelbase/track width/vertical offset;
- front/rear tire friction from `tires`;
- motor speed/torque limits and 2WD/4WD drive layout;
- steering angle and rate limits;
- rolling resistance and aero damping.

## Frames

World: x forward, y left, z up. Body: x forward, y left, z up, origin at the
chassis centre. Sensor mounts use the body frame. PyBullet reports the base
centre-of-mass pose, so the PyBullet world converts it back to the body-frame
origin; MuJoCo reports the body frame directly. Raycast origins and sensor heights
therefore use the same convention in both backends.

## Backends

- **MuJoCo** is the default digital twin. It uses a generated MJCF with contact
  wheels, steering hinges, position-servo actuators and per-wheel torque motors.
- **PyBullet** is kept as a comparison and debugging backend.
- Both implement the `World` protocol in `simulator/world_api.py`, so the same
  lidar, ultrasonic, odometry and dashboard code runs on either.

## Lap timing

Progress is projected onto the track centreline. A lap is armed only after the
car has travelled at least half a lap so a small roll at the start line does not
count. Race completion is based on the configured lap count.
