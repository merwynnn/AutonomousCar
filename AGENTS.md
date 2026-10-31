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
FastAPI dashboard (dashboard/)
    │
Simulation core (simulator/, 240 Hz physics, 60 Hz control)
    ├── generated URDF (simulator/car_urdf.py)
    ├── PyBullet world, track, car and timing
    ├── non-ideal lidar, ultrasonic and odometry models
    └── scenario/vehicle/sensor YAML configs
Algorithms (algorithms/)
    └── Observation -> Actuation controllers (NumPy only)
```

- The dashboard is a renderer and operator console. It never runs physics.
- `simulator/` owns physics, sensor models, latency, the track and lap timing.
- `algorithms/` owns the driving policy and imports only
  `algorithms.interface` and NumPy.
- `Observation` has no true pose and no future scan. The only pose is drifting
  wheel/IMU odometry.
- The dashboard may display ground truth for debugging, but controllers must
  never receive it.
- The real car can implement `RobotAPI` and run the same controllers unchanged.

Detailed references:

- `README.md` — quick start and configuration.
- `docs/02-architecture.md` — components and data flow.
- `docs/03-simulation.md` — physics, vehicle model and timing.
- `docs/04-sensors.md` — non-ideal sensor models.
- `docs/08-dashboard.md` — dashboard views, protocol and UX rules.
- `docs/09-car-definition.md` — the single-source car specification.

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

The car is data, not a hand-written URDF.

- One `VehicleConfig` is the only source of truth for chassis geometry, mass,
  centre of mass, wheelbase, track width, wheel size/position, motor, tires and
  steering limits.
- `simulator/car_urdf.py` generates the URDF used by PyBullet. Never fork a
  second vehicle model in the frontend or physics code.
- Sensor mounts live in `SensorsConfig` in the vehicle body frame:
  x forward, y left, z up, origin at the chassis centre.
- Body parts are `structure` entries. Box, cylinder and sphere parts may be
  visual-only or collidable.
- A code-defined car is valid: construct `VehicleConfig` and `SensorsConfig` in
  Python and pass them to `Simulation(vehicle=..., sensors=...)`.
- After changing YAML, use the dashboard **Reload config** action (or restart
  the server). A reload must rebuild the URDF, sensors and simulation.
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
- Odometry must drift; encoders must see slip from the physics state.
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
- Keep `algorithms/` free of `simulator.*` imports.
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

- `pytest` — configs, car generation, sensors, timing, API.
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
- Do not duplicate car geometry in the dashboard or a second URDF.
- Do not put track geometry in the car-only studio view.
- Do not add ROS 2, Gazebo or heavy GPU dependencies without an ADR; the MVP is
  deliberately lightweight.
- Do not commit large recordings; store them under `data/` and link them in docs.

## 11. Directory map

```text
simulator/          standalone PyBullet core
  config.py         pydantic config models + YAML loading
  car_urdf.py       generated URDF from VehicleConfig
  world.py          world, walls, car loading and raycasts
  sensors/          lidar and ultrasonic models
  simulation.py     fixed-step loop, latency filtering, lap timing
  track.py          test track and procedural obstacles
  configs/          scenario, vehicle and sensor YAML
  assets/           reference URDF only; runtime URDF is generated
algorithms/         sensor-only controller contract + demo controllers
dashboard/          FastAPI backend and React/Three.js car lab
  server.py         REST + WebSocket adapter
  spec.py           car/sensor/track JSON payloads
  web/              Vite dashboard
tests/              Python tests and fixtures
docs/               architecture, requirements, runbook, devlog, ADRs
```
