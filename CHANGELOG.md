# Changelog

## 2026-11-01 — Developer startup script

- Added `run.bat` to launch the backend and dashboard in two WSL windows and
  open the web UI.

## 2026-11-01 — Portable onboard runtime and shared state estimator

- Added `onboard/` as the Raspberry Pi-deployable package:
  - `onboard.api` owns the observation/actuation contract, raw sensor frames,
    fused state estimate and `RobotAPI`;
  - `onboard.estimator.WheelImuEstimator` fuses rear encoders, gyro and steering;
  - `onboard.runtime.SensorRobot` / `ControllerRuntime` provide a shared loop;
  - `onboard.drivers` defines hardware protocols and a JSON-lines board link.
- Added `simulator/bridge.py` with `SimRobotAPI`; the simulator is now one
  provider of the portable robot interface.
- Controllers receive both `observation.sensors` (raw) and `observation.state`
  (fused), with compatibility properties for older code.
- Added simulated encoder, IMU and steering-angle measurement models, including
  wheel-radius/track-width calibration errors, gyro bias/random walk, sensor
  latency and dropouts.
- Replaced the simulator-only odometry error model with the shared estimator.
- Moved shared configuration to root `configs/`:
  - `vehicle*.yaml`, `sensors*.yaml` and `estimator.yaml` are portable;
  - simulator-only scenarios live in `configs/sim/`.
- Added import-boundary tests for `onboard/` and `algorithms/`.
- Fixed the CLI gap-follow controller default speed limit.

## 2026-10-31 — Tuning console, gap debug and random tracks

- Added a dashboard tuning console for the full vehicle and sensor configs.
  - Apply live or save to YAML; comments are preserved where possible.
  - Quick-tune strip for steering rate/latency/max angle/deadband.
  - `Ctrl+Z` / `Ctrl+Y` undo/redo draft edits.
- Lowered the reference steering rate to a more realistic 180 °/s and documented
  the real-car slew-rate limiter.
- Added gap-follow debug data: clearance profile, chosen bearing, opening edges,
  clearance, throttle and min clearance.
- Added a 3D gap fan/line overlay and a live clearance-vs-bearing chart.
- Added a random track generator with seed, lane width and object density.
  - Default test track can always be restored from the dashboard.
  - Generated tracks stay inside the 15 × 8 m arena.
- Switched manual driving to ZQSD; physical WASD and arrow keys also work.

## 2026-10-31 — MuJoCo digital twin and AWAKE reference car

- Added a generated MuJoCo MJCF backend with contact wheels, steering hinges,
  wheel torque motors and position-servo steering.
- Made MuJoCo the default dashboard/CLI backend; PyBullet remains selectable.
- Added a shared `World` protocol so sensors and simulation run unchanged on
  either backend.
- Added the reference AWAKE 2027 car/sensor scenario:
  one 4WD motor, VESC-class limits, LD19-class lidar at 14 cm, four ultrasonic
  sensors, BNO085-class IMU and encoders.
- Added reference hardware BOM and calibration protocol docs.
- Added MuJoCo backend tests for model compilation, driving, sensor boundary and
  wall raycasts.

## 2026-10-31 — Car lab dashboard on the simulator core

- Added a FastAPI + WebSocket dashboard under `dashboard/`.
- Added a React/Three.js car studio that renders only the vehicle, with body
  parts, wheels and sensor mounts from the live configuration.
- Redesigned the UI as a flat desktop engineering application with an outliner,
  details inspector, toolbar, viewport HUD and status bar.
- Made the car definition drive the runtime URDF, physics and dashboard model.
- Added body structure parts, wheel width/mass/offset, sensor mount RPY and IMU
  mount to the configuration models.
- Added 3D lidar raycasts so lidar mount rotation affects measurements.
- Added lidar-only gap-follow and wall-follow demo controllers.
- Added code-defined car example and generated-URDF download endpoint.
- Added car-definition and URDF tests, dashboard docs and screenshots.
