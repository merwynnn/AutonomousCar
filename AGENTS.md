# AGENTS.md — Working rules for the AWAKE 2027 simulator core

This file is the entry point for any human or AI agent working in this repository.
Read it before changing code. Keep it up to date.

## 1. Mission

Build a physically credible, browser-driven simulator and strategy lab for the
AWAKE Challenge 2027 mini autonomous vehicle. The core must make it easy to:

- define and modify the car entirely from configuration/code (mass, body parts,
  dimensions, wheels, motor, tires, sensors, mounts and sensor characteristics);
- test controllers behind a sensor-only boundary with non-ideal sensors;
- inspect the vehicle in isolation from the track in the dashboard;
- compare controllers and conditions reproducibly;
- later port the same sensor models and controller contract to the Raspberry Pi.

The competition specification is `Cahier des charges 2027.pdf`. It is read-only.
The standalone core is intentionally smaller than the research branches: no SLAM,
no MCL, no racing line and no occupancy grid live here.

## 2. Architecture at a glance

```text
Browser (React + Three.js)
    │  REST + WebSocket (JSON)
FastAPI dashboard (dashboard/, host only)
    │
Simulator (simulator/, host only)
    ├── generated MJCF (simulator/mjcf.py)
    ├── MuJoCo digital twin, track and raw sensor measurement models
    ├── simulator/bridge.py: SimRobotAPI
    └── fixed-step loop, latency filtering and lap timing
Portable onboard runtime (onboard/, deployable to the Pi)
    ├── api.py: Observation, SensorFrame, StateEstimate, RobotAPI
    ├── actuation.py: shared actuator calibration and command shaping
    ├── estimator/: shared wheel + IMU estimator
    ├── runtime.py: SensorRobot / ControllerRuntime
    └── drivers/: serial protocol, mock board and hardware protocols
Algorithms (algorithms/)
    ├── registry.py: dashboard-selectable controller list
    ├── debug.py: optional DebugFrame contract for custom debug views
    ├── learned_gap.py: CMA-ES-evolved gap policy + weights loader
    └── Observation -> Actuation controllers (onboard.api + NumPy only)
Training (training/, host only)
    └── CMA-ES rollout search, warm start, export and progress reporting
```

- The dashboard is a renderer and operator console. It never runs physics.
- `simulator/` owns physics, simulated sensor measurements, latency, the track
  and lap timing. It is host-only.
- `onboard/` is the code that is copied to the Raspberry Pi. It owns the
  portable contract, the shared estimator and the hardware drivers.
- `algorithms/` owns the driving policy and imports only `onboard.api` and
  NumPy.
- `Observation` has no true pose and no future scan. Controllers receive both
  raw sensor frames (`observation.sensors`) and the shared estimator output
  (`observation.state`). The only pose is the drifting estimate.
- The dashboard may display ground truth for debugging, but controllers must
  never receive it.
- The simulator implements `onboard.api.RobotAPI` through `SimRobotAPI`; the
  real car implements the same protocol with `onboard.runtime.SensorRobot`.

### Portable onboard boundary

These rules are non-negotiable when adding code:

- `onboard/` is the code intended to run on the Raspberry Pi.
- `onboard/` and `algorithms/` must never import `simulator.*` or
  `dashboard.*`, nor a physics backend or FastAPI.
- `algorithms/` import only `onboard.api`, their own package modules and NumPy.
  `algorithms/debug.py` defines the optional, host-neutral debug snapshot
  contract; the dashboard must not import a controller's internal debug class.
- `onboard/api.py` owns `Observation`, `Actuation`, `SensorFrame`,
  `StateEstimate` and `RobotAPI`.
- `algorithms/interface.py` is a compatibility re-export only; new code imports
  `onboard.api` directly.
- `simulator/bridge.py` implements `RobotAPI` for the simulator.
- `simulator/sensors/` produce raw frames using the types in `onboard/api.py`.
- The shared estimator lives in `onboard/estimator/` and is used by both
  simulation and hardware.
- New sensors are added through a typed `SensorFrame` field, with a simulator
  model in `simulator/sensors/` and a hardware driver in `onboard/drivers/`.
  `SensorFrame.extra` is for short experiments, not the normal path.
- Raw sensor data and the fused `StateEstimate` are both exposed to controllers.
- Ground truth never appears in `Observation`.
- Keep the import-boundary test in `tests/test_boundary.py` green.

Detailed references:

- `README.md` — quick start and configuration.
- `docs/02-architecture.md` — components and data flow.
- `docs/03-simulation.md` — physics, vehicle model and timing.
- `docs/04-sensors.md` — non-ideal sensor models.
- `docs/08-dashboard.md` — dashboard views, protocol and UX rules.
- `docs/09-car-definition.md` — the single-source car specification.
- `docs/10-hardware.md` — reference BOM, wiring and packaging.
- `docs/11-calibration.md` — measurement and validation protocol.

## 3. Quick start

```bash
# Python environment (uv-managed Python 3.11)
uv sync --extra dev

# backend (terminal 1)
uv run python -m dashboard.server

# frontend (terminal 2)
cd dashboard/web
npm install
npm run dev
```

Open the Vite URL (usually `http://localhost:5173`). The backend runs on
`http://localhost:8000`.

Useful commands:

```bash
uv run pytest
uv run ruff check simulator algorithms dashboard tests
uv run mypy simulator algorithms dashboard
cd dashboard/web && npm run build
uv run python -m simulator.cli --laps 1 --max-time 5
```

## 4. Car definition rules

The car is data, not a hand-written physics model.

- One `VehicleConfig` is the only source of truth for chassis geometry, mass,
  centre of mass, wheelbase, track width, wheel size/position, motor, tires and
  steering limits.
- `simulator/mjcf.py` generates the MJCF used by the MuJoCo world. Never fork a
  second vehicle model in the frontend or physics code.
- Sensor mounts live in `SensorsConfig` in the vehicle body frame:
  x forward, y left, z up, origin at the chassis centre.
- Body parts are `structure` entries. Box, cylinder and sphere parts may be
  visual-only or collidable.
- Shared car and sensor files live in the root `configs/` directory:
  `vehicle*.yaml`, `sensors*.yaml`, `estimator.yaml`, `hardware.yaml`.
  Simulator-only scenarios live in `configs/sim/`. Do not put shared
  definitions back under `simulator/configs/`.
- `configs/hardware.yaml` owns servo/VESC/board calibration. Simulation and the
  Pi must load the same values through `onboard.actuation`; never duplicate
  throttle or steering mappings in a backend.
- `configs/estimator.yaml` contains estimator tuning only. Wheel radius,
  wheelbase, track width and encoder resolution are read from the shared
  vehicle/sensor files by `onboard.config.load_onboard_config`.
- A code-defined car is valid: construct `VehicleConfig` and `SensorsConfig` in
  Python and pass them to `Simulation(vehicle=..., sensors=...)`.
- After changing YAML, use the dashboard **Reload config** action (or restart
  the server). A reload must rebuild the MJCF, sensors, estimator and simulation.
- The dashboard's car studio must show only the car, never track walls, surface,
  lidar returns or trails.

## 5. Non-negotiable technical rules

### 5.1 Sensor realism

- Default mode is `realistic`. `ideal` exists only for debugging and must be
  clearly visible in the UI and logs.
- Controllers must only receive messages whose `t_available` is in the past.
  Never leak ground truth, future scans or perfect odometry.
- Every sensor effect must be seeded and configurable. New realism features need
  a unit test (analytic scene or statistical check).
- Lidar includes range/angle noise, quantization, per-beam timestamps and motion
  distortion, dropouts, beam divergence, intensity, wall-height occlusion,
  mount RPY, latency/jitter and calibration errors.
- Ultrasonic includes cone integration, blind zone, max range, noise, low update
  rate, latency, cross-talk, specular dropouts and missed thin objects.
- The fused state estimate must drift; encoders must see wheel spin and slip
  from the physics state; gyro bias must be learned, not read from ground truth.
- If a controller performs suspiciously well, check for ground-truth leakage
  before celebrating.

### 5.2 Determinism

- Physics uses a fixed timestep (default 1/240 s).
- Each sensor has its own seeded RNG stream.
- Rendering and wall-clock time never affect simulation results.
- Headless runs must be reproducible for a fixed seed/config.

### 5.3 Performance

- Lidar raycasts are batched. Profile before increasing sub-ray counts.
- The dashboard uses one sub-ray per beam in live mode; batch/CLI runs keep the
  configured fidelity.
- Keep telemetry throttled and avoid rebuilding large Three.js objects every
  frame.
- Dispose of Three.js geometries, materials and textures on unmount or rebuild.

### 5.4 Real-car portability

- Keep controller code hardware-agnostic behind `RobotAPI`.
- Keep `algorithms/` and `onboard/` free of `simulator.*` imports.
- Use the same estimator (`onboard/estimator/`) and the same timestamped
  `SensorFrame` types in simulation and on the Pi.
- Hardware drivers implement `SensorSource` / `ActuatorSink` and must timestamp
  when the sample was taken, not when the Pi received it.
- Keep actuator saturation, steering rate and latency limits in the shared car
  config so simulation and firmware cannot silently disagree.
- Document ARM/Pi build steps when the port is attempted. Do not claim Pi
  performance before benchmarking it.

## 6. Coding conventions

- Python: 3.11, `ruff` for lint/format, `mypy` for types, `pytest` for tests.
  Prefer dataclasses/pydantic for configs and type hints on public functions.
- TypeScript/React: strict TypeScript, functional components, Three.js scene
  code isolated in `dashboard/web/src/scene/`, no `any` unless justified.
- Configs are YAML, validated by pydantic models. No hard-coded vehicle/track
  constants in algorithms or the dashboard.
- Units are SI: metres, seconds, radians, kilograms, volts, amperes. Angles in
  the UI may be degrees, always labelled.
- Coordinate frames: world (x forward, y left, z up), body and sensor. Document
  every transform and use explicit frame suffixes.
- The dashboard is an engineering tool: dense, flat, legible, no decorative
  gradients, neon glows or oversized rounded cards. Follow the visual language
  in `UI-reference.webp` (desktop engineering application) rather than a generic
  web dashboard.

## 7. Testing and definition of done

A change is done only when:

1. It works in the dashboard or CLI, not just in a unit test.
2. Relevant unit/integration tests pass and `ruff`/`mypy` are clean.
3. Sensor realism is preserved or improved, with a test.
4. The generated car model and dashboard car view still agree with the config.
5. `docs/` is updated, plus `STATUS.md` and a devlog entry for significant work.
6. A screenshot/GIF is added for visible UI changes.
7. The change is committed with a clear message.

Test layers:

- `pytest` — configs, car generation, sensors, estimator, timing, API.
- `tests/test_boundary.py` — fails if `onboard/` or `algorithms/` imports host,
  physics or dashboard code.
- Golden runs — fixed seed/config produces the same metrics within tolerance.
- `npm run build` — frontend type-check and production build.
- Playwright screenshots when a visible UI change lands.

## 8. Documentation and monitoring rules

- `AGENTS.md` (this file): stable rules and architecture map.
- `STATUS.md`: current one-page state, updated every work session.
- `docs/devlog/YYYY-MM-DD.md`: what was done, decisions, blockers, next steps.
- `docs/13-decisions/ADR-*.md`: significant technical decisions and trade-offs.
- `docs/experiments/EXP-*.md`: experiment config, seed, metrics, plots, conclusion.
- `docs/assets/`: screenshots and GIFs referenced by docs.
- `CHANGELOG.md`: user-visible changes.
- Telegram: send a screenshot when a visible feature lands.

## 9. Git rules

- `main` in the wider repository holds the research branch; this branch is the
  transferable core. Keep changes focused and self-contained.
- Use short-lived feature branches for risky work.
- Conventional commits: `feat:`, `fix:`, `docs:`, `test:`, `refactor:`, `chore:`.
- Never commit `data/`, recordings, model weights, `.venv/`, `node_modules/`,
  build artefacts or secrets.
- Do not modify `Cahier des charges 2027.pdf`.
- Update docs in the same commit as the feature.

## 10. No-go zones

- Do not use GPS (competition rule).
- Do not let controllers read ground truth or future sensor data.
- Do not silently replace realistic sensors with perfect ones to make a test pass.
- Do not duplicate car geometry in the dashboard or a second physics model.
- Do not put track geometry in the car-only studio view.
- Do not add ROS 2, Gazebo or heavy GPU dependencies without an ADR; the MVP is
  deliberately lightweight.
- Do not commit large recordings; store them under `data/` and link them in docs.

## 11. Directory map

```text
onboard/            portable code copied to the Raspberry Pi
  api.py            Observation, SensorFrame, StateEstimate, RobotAPI
  actuation.py      shared actuator calibration and command shaping
  config.py         estimator tuning + shared geometry/hardware loading
  estimator/        wheel + IMU estimator used by sim and hardware
  scan_motion.py    estimated-odometry lidar scan motion compensation
  runtime.py        SensorRobot / ControllerRuntime
  drivers/          JSON-lines protocol, mock board and hardware protocols
simulator/          host-only physics and sensor core
  config.py         pydantic config models + YAML loading
  mjcf.py           generated MuJoCo MJCF from VehicleConfig
  bridge.py         SimRobotAPI implementation
  mujoco_world.py   MuJoCo world and raycasts
  sensors/          lidar, ultrasonic, encoder, IMU and steering models
  simulation.py     fixed-step loop, latency filtering, lap timing
  track.py          test track and procedural obstacles
configs/            shared, portable configuration
  vehicle*.yaml     car geometry, structure, motor, tires, steering
  sensors*.yaml     sensor mounts and non-ideal characteristics
  estimator.yaml    shared wheel + IMU estimator tuning
  hardware.yaml     servo/VESC/board calibration shared with the Pi
  sim/              simulator-only scenarios
algorithms/         portable controllers (onboard.api + NumPy only)
  learned_gap.py    CMA-ES-evolved B1/B2 gap policy
  weights/          committed evolved policy (.npz), explicit exception
training/           host-only CMA-ES search, warm start, export, progress
dashboard/          host-only FastAPI backend and React/Three.js car lab
  server.py         REST + WebSocket adapter
  spec.py           car/sensor/track JSON payloads
  web/              Vite dashboard
tests/              Python tests and fixtures
docs/               architecture, requirements, runbook, devlog, ADRs
```
