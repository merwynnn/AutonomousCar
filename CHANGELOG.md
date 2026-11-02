# Changelog

## 2026-11-02 — Non-fatal algorithm errors

- Controller import/reload failures and runtime exceptions no longer stop the
  simulation loop. The run pauses and the dashboard shows a persistent
  controller-error strip.
- Failed reloads keep the previous controller instances; a successful reload
  clears the error.
- `debug_frame()` failures are contained and shown as `DEBUG ERROR` in the debug
  dock instead of breaking telemetry.
- Added regression tests for import failures, runtime errors and debug errors.

## 2026-11-02 — Live algorithm reload

- Added a **Reload algo** toolbar button and `controller.reload` WebSocket
  command.
- The backend re-imports `algorithms/*.py` (debug contract first, registry last),
  rebuilds controller instances and keeps the current simulation, track and lap
  state. The new controller takes over on the next tick.
- Reload failures are reported in the dashboard without replacing the working
  controllers.
- Clarified in the docs that **Reload config** reloads YAML and rebuilds physics
  and sensors; it does not re-import Python code.

## 2026-11-02 — Generic controller debug and gap solver candidates

- Added a controller-neutral `DebugFrame` contract (`algorithms/debug.py`) for
  readouts, plots, candidate tables and 3D overlays.
- Added `algorithms/registry.py`; the dashboard controller list is now served
  by the backend instead of being hardcoded in the UI.
- Replaced the gap-only WebSocket payload and panel with `state.debug` and a
  shared `DebugDock`.
- Gap-follow debug now shows raw/smoothed/clearance/score curves, the scoring
  terms, scored candidates with the selected one highlighted, validity reasons
  and a compact readout grid.
- The 3D debug layer draws the opening, selected/clamped bearings and three
  well-separated candidate labels using generic overlay primitives.

## 2026-11-02 — AutoBench dashboard cleanup

- Renamed the dashboard wordmark and browser title from AWAKE to **AutoBench**.
- Made the world outliner and details panels collapsible to narrow rails; the
  viewport expands when a panel is retracted.
- Removed duplicate Track / Car only controls from the toolbar and outliner.
  The viewport header now owns the single view switch.

## 2026-11-02 — Shared actuator layer and hardware-ready serial path

- Added `configs/hardware.yaml` for servo/VESC/board calibration shared by
  simulation and the Pi.
- Added `onboard.actuation` with steering rate/deadband/backlash/latency and a
  throttle-to-speed PI with battery sag and torque lag.
- `Actuation.throttle` is now a normalized target speed; controllers can read
  static limits from `Observation.limits`.
- Added `MotorSample` telemetry and a simulator motor measurement model.
- Updated the JSON-lines protocol with protocol version, sequence numbers,
  calibrated steering/speed targets and motor telemetry.
- Added `MockBoard` for hardware-free end-to-end tests of `SensorRobot` and the
  serial drivers.
- Added `awake-selftest` and an optional `hardware` extra for pyserial.

## 2026-11-02 — MuJoCo-only core

- Removed the PyBullet backend, its URDF generator and the generated-URDF
  download endpoint. MuJoCo is now the only physics world.
- Removed `--backend`, `--gui`, `AWAKE_BACKEND` and the backend fields from the
  dashboard/WebSocket payloads.
- Regenerated `uv.lock`; `pybullet` is no longer a dependency.
- Updated the example, tests, architecture docs and dashboard copy to the
  MuJoCo-only model.

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
