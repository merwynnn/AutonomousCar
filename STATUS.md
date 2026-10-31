# Project status

> Last updated: 2026-10-31
> Branch: `sim-core` — standalone simulator + dashboard car lab

## What works

- Data-driven car definition in YAML or Python:
  - mass, centre of mass, dimensions, wheelbase, track width, wheels;
  - body structure with box/cylinder/sphere parts and collision flags;
  - motor, battery, tires, suspension, aero and steering limits;
  - sensor mounts with position and roll/pitch/yaw;
  - lidar/ultrasonic/IMU/encoder characteristics.
- Runtime URDF generation from `VehicleConfig`; PyBullet loads the generated car.
- 240 Hz PyBullet physics, 60 Hz control, lap timing and deterministic seeded
  sensors.
- 720-beam 3D-ray-traced lidar with non-ideal effects, three ultrasonic sensors,
  IMU/odometry drift.
- Sensor-only algorithm boundary and two lidar-only demo controllers
  (furthest-gap and wall-follow).
- Dashboard at `dashboard/`:
  - live track scene with lidar returns, trail and odometry;
  - **Car studio** showing only the car, with body parts, wheels and sensor
    mounts from the same config;
  - desktop engineering UI inspired by the provided reference;
  - outliner/details inspector, run controls, manual driving, speed limit,
    config reload.
- REST/WebSocket backend and generated-URDF download.

## How to run

```bash
uv sync --extra dev
uv run python -m dashboard.server
# second terminal
cd dashboard/web && npm install && npm run dev
```

Open `http://localhost:5173` (dev) or `http://localhost:8000` after
`npm run build`.

## Known limitations

- No SLAM, MCL, racing line or map in this branch.
- Dashboard auto controllers are simple lidar-only demos, not a racing stack.
- Suspension tuning is approximated through damping/friction rather than a full
  spring/damper model.
- The car studio is an inspection view; it does not edit YAML in place.
- Headless browser screenshots need WebGL/EGL support.

## Next tasks

1. Add live numeric editing for selected car/sensor properties.
2. Add a proper suspension model if lap-time sensitivity requires it.
3. Add brake/regen and motor torque-speed curves to the vehicle model.
4. Add recorded replay for dashboard sessions.
