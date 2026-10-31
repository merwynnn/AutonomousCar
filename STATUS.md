# Project status

> Last updated: 2026-10-31
> Branch: `sim-core` — standalone simulator + dashboard car lab

## What works

- **MuJoCo digital twin** as the default physics backend, generated from the same
  vehicle definition as the PyBullet comparison backend.
  - contact wheels, 4WD torque motors, steering hinges and position servos;
  - 240 Hz physics, 60 Hz control, lidar raycasts from MuJoCo geometry.
- Reference AWAKE 2027 car profile and sensor package in
  `simulator/configs/awake_2027.yaml`.
- Data-driven car definition in YAML or Python:
  - mass, centre of mass, dimensions, wheelbase, track width, wheels;
  - body structure with box/cylinder/sphere parts and collision flags;
  - motor, battery, tires, suspension, aero and steering limits;
  - sensor mounts with position and roll/pitch/yaw;
  - lidar/ultrasonic/IMU/encoder characteristics.
- Runtime URDF/MJCF generation from `VehicleConfig`; both backends load the
  generated car.
- 240 Hz MuJoCo/PyBullet physics, 60 Hz control, lap timing and deterministic
  seeded sensors.
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

Reference hardware: [docs/10-hardware.md](docs/10-hardware.md).
Calibration: [docs/11-calibration.md](docs/11-calibration.md).

## Known limitations

- No SLAM, MCL, racing line or map in this branch.
- MuJoCo tyre behaviour is contact-based; it is not yet a Pacejka/brush tyre
  model calibrated against the real car.
- Suspension stiffness is represented at the joint/contact level rather than a
  full spring/damper linkage.
- The car studio is an inspection view; it does not edit YAML in place.
- Headless browser screenshots need WebGL/EGL support.

## Next tasks

1. Add a Pacejka/brush tyre model and calibrated motor torque-speed curve.
2. Add live numeric editing for selected car/sensor properties.
3. Add a full suspension linkage model if lap-time sensitivity requires it.
4. Add a real-car serial adapter and recorded replay.
5. Measure the selected BOM and fit the calibration protocol in `docs/11-calibration.md`.
