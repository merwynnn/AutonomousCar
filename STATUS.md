# Project status

> Last updated: 2026-11-02
> Branch: `sim-core` — MuJoCo-only simulator + portable onboard runtime

## What works

- **Portable onboard boundary** in `onboard/`:
  - `onboard/api.py` owns `Observation`, `Actuation`, `SensorFrame`,
    `StateEstimate` and `RobotAPI`;
  - controllers receive raw sensor frames and shared-estimator output;
  - `algorithms/` imports only `onboard.api` and NumPy;
  - `simulator/bridge.py` exposes `SimRobotAPI`;
  - an import-boundary test fails if `onboard/` or `algorithms/` imports host,
    physics or dashboard code.
- **Shared wheel + IMU estimator** in `onboard/estimator/`:
  - rear encoder ticks, gyro yaw rate and steering fusion;
  - zero-velocity gyro-bias learning;
  - slip detection when wheel and gyro disagree;
  - conservative diagnostic covariance.
- **Raw simulated sensor chain**:
  - encoder model with wheel-radius/track-width calibration errors, 50 Hz
    ticks, jitter and dropouts;
  - BNO085-class IMU with bias, random walk, scale, noise, cross-axis coupling
    and latency;
  - steering-angle sensor with noise, quantization and latency;
  - lidar and ultrasonic models unchanged and still timestamp-filtered.
- **Shared configuration layout**:
  - `configs/vehicle*.yaml` and `configs/sensors*.yaml` are portable;
  - `configs/estimator.yaml` holds estimator tuning only;
  - `configs/sim/` holds simulator-only scenarios.
- **MuJoCo digital twin** generated from the vehicle definition.
  - contact wheels, 4WD torque motors, steering hinges and position servos;
  - 240 Hz physics, 60 Hz control, lidar raycasts from MuJoCo geometry.
- Reference AWAKE 2027 car profile and sensor package.
- Data-driven car definition in YAML or Python:
  - mass, centre of mass, dimensions, wheelbase, track width, wheels;
  - body structure with box/cylinder/sphere parts and collision flags;
  - motor, battery, tires, suspension, aero and steering limits;
  - sensor mounts with position and roll/pitch/yaw;
  - lidar/ultrasonic/IMU/encoder characteristics.
- Runtime MJCF generation from `VehicleConfig`; MuJoCo loads the generated car.
- Sensor-only algorithm boundary and two lidar-only demo controllers
  (furthest-gap and wall-follow).
- Dashboard at `dashboard/`:
  - live track scene with lidar returns, trail and odometry;
  - **Car studio** showing only the car, with body parts, wheels and sensor
    mounts from the same config;
  - **Tuning console** for all vehicle/sensor parameters, live apply,
    save-to-YAML and undo/redo;
  - **gap-follow debug** with 3D opening overlay and live clearance profile;
  - **random track generator** with object density and default-track restore;
  - desktop engineering UI inspired by the provided reference;
  - outliner/details inspector, run controls, manual driving, speed limit,
    config reload.
- REST/WebSocket backend and generated-MJCF download.

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
- The estimator is a complementary wheel/IMU filter, not a full EKF, and does
  not use lidar scan matching.
- MuJoCo tyre behaviour is contact-based; it is not yet a Pacejka/brush tyre
  model calibrated against the real car.
- Suspension stiffness is represented at the joint/contact level rather than a
  full spring/damper linkage.
- Hardware drivers are protocol/protocol-skeleton level; board-specific serial
  and lidar setup still has to be implemented on the Pi.
- The car studio is an inspection view; it does not edit YAML in place.
- Headless browser screenshots need WebGL/EGL support.

## Next tasks

1. Implement the board-specific `SensorSource` / `ActuatorSink` drivers and
   record real encoder/IMU/lidar logs.
2. Validate the shared estimator against recorded real-car runs.
3. Add a 2D EKF and optional lidar correction behind the same `StateEstimator`
   protocol if the controller needs better pose.
4. Add a Pacejka/brush tyre model and calibrated motor torque-speed curve.
5. Measure the selected BOM and fit the calibration protocol in
   `docs/11-calibration.md`.
