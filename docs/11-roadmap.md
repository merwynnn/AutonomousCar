# Roadmap

> Current milestone is tracked in `STATUS.md`. Each milestone is complete only when
> its acceptance criteria are met and documented.

## M0 — Bootstrap ✅

**Deliverables**

- git repository, `.gitignore`, README, AGENTS, STATUS;
- docs scaffold (architecture, requirements, sensors, SLAM, strategies, dashboard);
- Python 3.11 environment via `uv`, PyBullet smoke test;
- C++17 SLAM binding skeleton with a test;
- FastAPI backend shell + React/Three.js dashboard shell;
- `make dev`, `make check`;
- first screenshot and devlog entry.

**Acceptance**

- Dashboard opens and shows a live placeholder scene.
- `/api/health` responds.
- C++ binding builds and a smoke test passes.
- `make check` is green.
- Docs index and STATUS are current.

## M1 — Driveable vertical slice ✅

**Deliverables**

- PDF-like test track with 16 cm pipe walls and start/finish line.
- Generic 1/10 4WD Ackermann car with PyBullet physics.
- Realistic lidar, 3 ultrasonic, IMU and odometry with latency-aware bus.
- Manual driving from the browser.
- Simple auto controller (wall follow or pure pursuit).
- 5-lap race manager, lap times, penalties.
- Live 3D view, lidar/sensor overlays and charts.
- Tests and docs for all of the above.

**Acceptance**

- Car completes 5 autonomous laps with `realistic` sensors.
- Noise, dropouts, latency and motion distortion are visible and configurable.
- Controllers cannot see ground truth or future scans (tested).
- Seeded run is deterministic.
- Screenshot/GIF and STATUS/devlog updated.

## M2 — Strategy lab (core done; generator/editor pending)

**Deliverables**

- Controllers: wall follow, gap follow, pure pursuit, Stanley, MPC.
- Centreline extraction, racing line optimization, time-optimal speed profile.
- Track generator with obstacles + simple editor.
- Strategy comparison report using `awake-sim compare` and documented experiments.

**Acceptance**

- At least 4 strategies compared on generated tracks.
- Measurable lap-time improvement over the baseline.
- Crash/penalty rates reported, not just lap times.

## M3 — SLAM (front-end + local MCL done; reliable loop closure/global MCL pending)

**Deliverables**

- C++17 SLAM core: preprocessing/deskew, CSM, occupancy grid, pose graph.
- Reliable place recognition and loop-closure verification.
- MCL local localization ✅; global relocalization pending.
- pybind11 bindings + Python reference implementation.
- Live SLAM visualizations and diagnostics.
- Evaluation: ATE/RPE, map IoU, loop closure, runtime.
- Mapping/race/online modes.
- Map export.

**Acceptance**

- 10 Hz processing on desktop (target <20 ms/scan for 720 beams).
- Front-end ATE and drift reported honestly; loop closure tested with a
  false-positive suite.
- Visually accurate map and trajectory after loop correction.
- SLAM-localized 5-lap run.
- Benchmark and docs.

## M4 — Robustness and optimization

**Deliverables**

- `ideal`/`realistic`/`harsh` presets, latency/dropout/vibration/miscalibration.
- Collision/penalty model refinement.
- Randomized track/obstacle robustness suite.
- Hardware what-if and calibration guide.

**Acceptance**

- Speed-vs-robustness summary across presets.
- Hardware sensitivity report.
- Harsh-preset results and failure analysis.
- Calibration workflow documented and tested on synthetic data.

## M5 — Polish and handoff

**Deliverables**

- Complete docs, runbook, demo video/GIF, final report.
- Reproducible fresh install.
- Full test suite and benchmarks.
- Clean code and commit history.

**Acceptance**

- Fresh clone installs and runs from the runbook.
- All checks pass.
- Demo recorded.
- Handoff session completed.

## Later (optional)

- Raspberry Pi port and onboard benchmark.
- Camera simulation and line detection.
- ROS 2 bridge.
- MuJoCo or higher-fidelity tire model.
- Reinforcement learning experiments.
