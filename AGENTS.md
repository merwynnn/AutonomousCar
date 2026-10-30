# AGENTS.md — Working rules for the AWAKE 2027 simulator

This file is the entry point for any human or AI agent working in this repository.
Read it before changing code. Keep it up to date.

## 1. Mission

Build a physically credible, browser-driven simulator and strategy lab for the
AWAKE Challenge 2027 mini autonomous vehicle. The simulator must make it easy to:

- test controllers, racing lines and speed profiles for the fastest safe 5-lap run;
- simulate non-ideal sensors, especially a realistic 2D lidar;
- develop and visualize a from-scratch 2D lidar SLAM pipeline;
- compare strategies over many generated tracks and controlled conditions;
- later calibrate the same models against the real Raspberry Pi car.

The competition specification is `Cahier des charges 2027.pdf`. It is read-only.
`docs/01-requirements.md` is the traceable interpretation of that document.

## 2. Current status

Always read `STATUS.md` first. It contains the current milestone, what works, how
to run it, what is next and known issues. `docs/devlog/` contains dated work logs.

## 3. Quick start

> These commands are updated as the project is built. If a command fails, check
> `docs/12-runbook.md` and `STATUS.md` before changing dependencies.

```bash
# one-time system toolchain (user has already installed it)
sudo apt install -y build-essential cmake ninja-build libeigen3-dev

# Python environment (uv-managed Python 3.11)
uv sync --all-extras

# C++ SLAM extension (editable build)
uv run python -m awake_sim.cli build-cpp

# backend
uv run uvicorn awake_sim.server.app:app --reload --port 8000

# frontend
cd web && npm install && npm run dev

# all checks
make check
```

## 4. Architecture at a glance

```text
Browser (React + Three.js)
    │  REST + WebSocket (JSON commands, binary telemetry)
FastAPI server
    │
Simulation manager / worker (Python, 240 Hz physics)
    ├── PyBullet world, generic 4WD Ackermann car, tracks
    ├── sensor models (lidar, ultrasonic, IMU, encoders) + latency bus
    ├── controllers, planning, racing line, speed profile
    ├── C++ SLAM core (pybind11): CSM/ICP, occupancy grid, pose graph, MCL
    └── recorder + experiment database
Batch runner (headless subprocesses) → SQLite/CSV → dashboard
```

- Python orchestrates simulation, experiments and visualization.
- Performance-critical SLAM lives in `cpp/` and is exposed through `src/awake_sim/slam/`.
- The browser never runs physics; it renders server state and sends commands.
- The real car interface is `RobotAPI`; the simulation adapter and a future serial
  adapter must be interchangeable.

Detailed docs:

- `docs/02-architecture.md` — components, data flow, interfaces.
- `docs/03-simulation.md` — physics, vehicle model, timing.
- `docs/04-sensors.md` — non-ideal sensor models and validation.
- `docs/06-slam.md` — SLAM math, pipeline and evaluation.
- `docs/07-strategies.md` — controllers, racing line, speed profile, experiments.
- `docs/08-dashboard.md` — UI views, protocol and UX rules.

## 5. Non-negotiable technical rules

### 5.1 Sensor realism

- Default mode is `realistic`. `ideal` exists only for debugging and must be
  clearly visible in the UI and logs.
- Controllers and SLAM must only receive messages whose `t_available` is in the
  past. Never leak ground truth, future scans or perfect odometry.
- Every sensor effect must be seeded and configurable. New realism features need
  a unit test (analytic scene or statistical check).
- Lidar simulation must include, at minimum: range/angle noise, quantization,
  per-beam timestamps and motion distortion, dropouts, beam divergence/mixed
  pixels, intensity, wall-height occlusion, self-occlusion, mount tilt/vibration,
  latency/jitter and calibration errors.
- Ultrasonic simulation must include cone integration, blind zone, max range,
  noise, low update rate, latency, cross-talk, specular dropouts and missed thin
  objects.
- IMU and odometry must drift; encoders must see slip from the physics state.
- If a controller performs suspiciously well, check for ground-truth leakage
  before celebrating.

### 5.2 Determinism

- Physics uses a fixed timestep (default 1/240 s).
- Each sensor has its own seeded RNG stream.
- Every run records its config hash and seed.
- Rendering and wall-clock time never affect simulation results.
- Batch experiments must be reproducible run-to-run.

### 5.3 Performance

- SLAM hot path is C++17 (Eigen). Python reference code is for tests/teaching only.
- Target: <20 ms per 720-beam scan on desktop; <50 ms on the Pi later.
- Lidar raycasts are batched. Profile before increasing sub-ray counts.
- Telemetry to the browser is binary for arrays and throttled/decimated as needed.

### 5.4 Real-car portability

- Keep the C++ SLAM core free of desktop-only dependencies.
- Keep controller code hardware-agnostic behind `RobotAPI`.
- Document ARM/Pi build steps when the port is attempted. Do not claim Pi
  performance before benchmarking it.

## 6. Coding conventions

- Python: 3.11, `ruff` for lint/format, `mypy` for types, `pytest` for tests.
  Prefer dataclasses/pydantic for configs, type hints on public functions.
- C++: C++17, `clang-format` style (4 spaces, 100 cols), Eigen for linear algebra,
  pybind11 bindings, CTest for unit tests.
- TypeScript/React: strict TypeScript, ESLint + Prettier, functional components,
  Three.js scene code isolated from UI state, no `any` unless justified.
- Configs are YAML, validated by pydantic models. No hard-coded vehicle/track
  constants in algorithms.
- Units are SI: metres, seconds, radians, kilograms, volts, amperes. Angles in
  the UI may be degrees, always labelled.
- Coordinate frames: world (x forward, y left, z up), track, body, sensor.
  Document every transform and use explicit frame suffixes.

## 7. Testing and definition of done

A change is done only when:

1. It works in the dashboard or CLI, not just in a unit test.
2. Relevant unit/integration tests pass; `make check` is green.
3. Sensor realism is preserved or improved, with a test.
4. Performance is measured if it is on the hot path.
5. `docs/` is updated, plus `STATUS.md` and a devlog entry for significant work.
6. A screenshot/GIF is added for visible UI changes.
7. The change is committed with a clear message.

Test layers:

- `pytest` — configs, geometry, sensors, timing, SLAM reference, API.
- `ctest` — C++ math, grid, matcher, pose graph.
- `vitest`/`playwright` — frontend logic and smoke/screenshot tests.
- Golden runs — fixed seed/config produces the same metrics within tolerance.

## 8. Documentation and monitoring rules

- `AGENTS.md` (this file): stable rules and architecture map.
- `STATUS.md`: current one-page state, updated every work session.
- `docs/devlog/YYYY-MM-DD.md`: what was done, decisions, blockers, next steps.
- `docs/13-decisions/ADR-*.md`: significant technical decisions and trade-offs.
- `docs/experiments/EXP-*.md`: experiment config, seed, metrics, plots, conclusion.
- `docs/assets/`: screenshots and GIFs referenced by docs.
- `CHANGELOG.md`: user-visible changes.
- Telegram: send a screenshot at each milestone and when a visible feature lands.
- The dashboard “Project” panel renders `STATUS.md` so the user can monitor work
  without leaving the app.

## 9. Git rules

- `main` is the working branch; use short-lived feature branches for risky work.
- Conventional commits: `feat:`, `fix:`, `docs:`, `test:`, `refactor:`, `chore:`.
- Never commit `data/`, recordings, model weights, `.venv/`, `node_modules/`,
  build artefacts or secrets.
- Do not modify `Cahier des charges 2027.pdf`.
- Keep commits focused; update docs in the same commit as the feature.

## 10. No-go zones

- Do not use GPS (competition rule).
- Do not let controllers or SLAM read ground truth or future sensor data.
- Do not silently replace realistic sensors with perfect ones to make a test pass.
- Do not claim digital-twin accuracy before calibration against the real car.
- Do not add ROS 2, Gazebo or heavy GPU dependencies without an ADR; the MVP is
  deliberately lightweight.
- Do not commit large recordings; store them under `data/` and link them in docs.

## 11. Directory map

```text
configs/            vehicle, sensor, track and scenario YAML
src/awake_sim/      Python package
  sim/              PyBullet world, vehicle, timing, sensor bus
  sensors/          lidar, ultrasonic, IMU, encoders
  tracks/           test track, generator, editor serialization
  control/          controllers, racing line, speed profile
  slam/             Python bindings + reference implementation + evaluation
  experiments/      batch runner, metrics, storage
  server/           FastAPI app, WebSocket protocol, simulation manager
  cli/              command-line entry points
cpp/                C++17 SLAM core + pybind11 bindings
web/                React/Three.js dashboard
tests/              Python tests + fixtures
docs/               architecture, requirements, runbook, devlog, ADRs
data/               local recordings and experiment artefacts (gitignored)
```
