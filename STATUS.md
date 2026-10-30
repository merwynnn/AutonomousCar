# Project status

> Last updated: 2026-10-30
> Current milestone: **M3 in progress — SLAM front-end live; loop closure detector disabled**

![M3 SLAM dashboard](docs/assets/2026-10-30-m3-slam.png)

## Headline results

Test track, 5 laps, `realistic` sensors:

| Controller | Total (s) | Best lap (s) | Localization |
|---|---:|---:|---|
| Wall-follow (lidar) | 148.34 | 29.46 | lidar only |
| **Pure pursuit + racing line** | **46.12** | **9.20** | ground truth (upper bound) |

SLAM front-end (local tracking, 8 s / 81 scans, low-noise odometry):

| Metric | Result |
|---|---:|
| ATE mean | **2.89 cm** |
| ATE max | 5.27 cm |
| Scan time mean | **14.2 ms** |
| Scan time p95 | **18.9 ms** |
| Occupied / free cells | 2,255 / 9,179 |

The live dashboard shows the SLAM occupancy map, blue SLAM ghost pose/trail,
ATE and scan time. With realistic drifting odometry the map accumulates drift;
the loop-closure detector is currently disabled because scan-only place
recognition produced false matches on the repetitive pipe walls. The SE(2)
pose-graph back-end is implemented and unit-tested, but no forced start-pose
reset is used.

## What works today

- PyBullet 1/10 4WD Ackermann car, 240 Hz physics, 16 cm pipe walls.
- 720-beam non-ideal lidar: noise, quantization, motion distortion, dropouts,
  dead beams, intensity, wall-height occlusion and latency.
- Three non-ideal ultrasonic sensors; IMU/odometry hooks.
- Latency-aware control: the last command is held while a scan is in flight.
- Wall-follow (lidar only), manual WASD driving.
- Racing line + time-optimal speed profile; pure pursuit (GT upper bound).
- Race manager: 5-lap timing, start-line arming, progress/lateral offset.
- **C++17 SLAM front-end**: per-beam deskew, occupancy grid, chamfer likelihood,
  coarse/fine correlative matching with motion prior, pybind11 bindings.
- **SE(2) pose graph** with keyframes, sequential edges and a tested optimizer.
- Loop-closure detection is **disabled** until a reliable descriptor/landmark is
  added; no forced reset or referee-driven pose correction.
- Live SLAM visualization: occupancy map texture, SLAM ghost/trail, ATE and
  scan-time diagnostics.
- Dashboard: live 3D, lidar, sonars, steering, speed trace, strategy benchmarks,
  controller switch, auto/manual, target speed, camera modes.
- CLI: `awake-sim race`, `compare`, `slam-bench`.
- 24 Python tests + C++ CTest green; ruff/mypy clean; frontend builds.

## Milestone status

- **M0 ✅** — bootstrap, docs, dashboard, C++ binding.
- **M1 ✅** — 5 autonomous wall-follow laps, realistic sensors.
- **M2 core ✅** — racing line, pure pursuit, strategy benchmark; generator and
  editor still pending.
- **M3 front-end ✅** — C++ SLAM tracking + live map; pose graph implemented,
  loop-closure detection disabled pending reliable place recognition; MCL later.

## Next tasks

1. M3b: reliable place recognition (Scan Context or a visible start-line
   landmark), loop-closure verification, MCL/global relocalization, map IoU and
   ATE/RPE reporting.
2. M2 rest: seeded track generator with obstacles and a simple track editor.
3. Calibration against the real car when hardware is chosen.
4. Pi port and benchmark (deferred).

## Known issues

- SLAM has no active loop closure or MCL yet. The front-end drifts with realistic
  wheel odometry; the map is local, not globally corrected.
- Scan-only loop detection is ambiguous on the repetitive pipe walls; it is
  disabled rather than allowed to corrupt the map.
- Some map artefacts appear outside the lane at high speed; robust map updates
  are planned.
- Pure pursuit uses ground truth; the deployable result needs SLAM/MCL pose.
- Live mode uses one lidar sub-ray per beam; high-fidelity mode uses three.
- No track generator/editor yet.
- Wall-follow is conservative and slow on the waist corners.

## How to run

```bash
uv run uvicorn awake_sim.server.app:app --port 8000
cd web && npm run dev -- --port 5173
# open http://localhost:5173
```

Headless checks and benchmarks:

```bash
uv run awake-sim compare --laps 5
uv run awake-sim slam-bench --duration 20
uv run pytest -q
uv run python -m awake_sim.cli test-cpp
```

## Monitoring

- Devlog: [`docs/devlog/2026-10-30.md`](docs/devlog/2026-10-30.md)
- Experiments: [`docs/experiments/EXP-0001`](docs/experiments/EXP-0001-strategy-baseline.md),
  [`EXP-0002`](docs/experiments/EXP-0002-slam-frontend.md)
- Architecture: [`docs/02-architecture.md`](docs/02-architecture.md)
- SLAM: [`docs/06-slam.md`](docs/06-slam.md)
- Sensors: [`docs/04-sensors.md`](docs/04-sensors.md)
- Roadmap: [`docs/11-roadmap.md`](docs/11-roadmap.md)
