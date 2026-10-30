# Project status

> Last updated: 2026-10-30
> Current milestone: **M3 in progress — MCL drive + map-once/race experiments**

![M3 SLAM/MCL dashboard](docs/assets/2026-10-30-m3-slam.png)

## Headline results

### Localization comparison (1 lap, 1.2 m/s, safe line)

| Pose source | Lap time | ATE mean | Status |
|---|---:|---:|---|
| GT | 20.12 s | 0.0 cm | OK |
| Odom | DNF | 36.4 cm | hit wall |
| **MCL** | **20.57 s** | **23.0 cm** | **OK** |

**MCL drives the car with no ground truth in the control loop**, only ~2%
slower than GT at this conservative speed. Odometry-only fails, worsened by the
procedural landmarks.

### Map once, race with MCL

| Metric | Result |
|---|---:|
| Mapping lap (MCL, no GT) | 20.68 s |
| Map seam (start vs end pose) | 11.9 cm |
| 5-lap race at 0.9 m/s | **5/5, 136.00 s** |
| 5-lap best lap | 27.03 s |
| Lap times | 27.13, 27.32, 27.24, 27.28, 27.03 s |
| 5-lap MCL ATE mean / max | 22.4 / 80.5 cm |
| Tuned MCL | 400 particles, rank_keep 0.99, stride 3 |

Mapping and racing are both sensor-only (MCL, no GT). The five-lap race
**completes at 0.9 m/s** with global relocalization off. At 1.2 m/s it still
sticks near the map seam; speed is currently the safety margin. Procedural
wall-side crates/poles add distinctive lidar features and make odometry-only
fail earlier.

## What works today

- PyBullet 1/10 4WD Ackermann car, 240 Hz physics, 16 cm pipe walls.
- Procedural wall-side crates/poles: collidable, lidar-visible SLAM landmarks.
- 720-beam non-ideal lidar, three ultrasonic sensors, drifting wheel odometry.
- Wall-follow, pure pursuit and manual driving.
- Racing line + speed profile with a speed-limit slider that applies to both
  controllers.
- Localization modes: **GT / Odom / MCL**; MCL drives pure pursuit with no GT.
- 400-particle MCL with ESS/spread diagnostics, on/off/reset.
- C++ SLAM front-end: deskew, occupancy grid, correlative matching; map freeze
  and save.
- SE(2) pose graph with keyframes; loop-closure detector experimental/off.
- Dashboard: live 3D, SLAM/MCL/odometry ghosts and trails, particles, map,
  debug layers, **Car view = Actual / Controller / SLAM**.
- CLI: `race`, `compare`, `localization-bench`, `map-and-race`, `slam-bench`.
- 24 Python tests + 2 C++ tests; frontend builds.

## Milestone status

- **M0 ✅** — bootstrap, docs, dashboard, C++ binding.
- **M1 ✅** — realistic sensors, wall-follow, 5-lap acceptance.
- **M2 core ✅** — racing line, pure pursuit, strategy benchmark; generator/editor
  still pending.
- **M3 front-end ✅** — SLAM tracking, map, MCL, localization modes, map-once race
  experiments; reliable loop closure/global MCL pending.

## Next tasks

1. Global MCL relocalization / periodic correction at the start line.
2. Reliable loop closure or a visible start-line landmark to remove the ~12 cm
   map seam.
3. More robust controller (Stanley/MPC) for MCL at higher speed.
4. M2 rest: track generator/editor.
5. Calibration against the real car; Pi port later.

## Known issues

- 5-lap MCL race completes only at the conservative 0.9 m/s; at 1.2 m/s it
  sticks near the seam.
- Global relocalization is experimental; on this symmetric track it can converge
  to the wrong place, so it is off by default.
- Loop closure is experimental and off; scan-only detection false-positives on
  repetitive walls.
- MCL is local, not global; no kidnapped-robot recovery.
- Odom-only pure pursuit fails at the tested settings.
- Safety/recovery is minimal; the current MCL race uses a conservative 1.2 m/s.
- Map artefacts near the seam remain.

## How to run

```bash
uv run uvicorn awake_sim.server.app:app --port 8000
cd web && npm run dev -- --port 5173
# open http://localhost:5173
```

In the dashboard, to drive with **no GT in the control loop**:

```text
Driving mode: Auto
Controller:   Pursuit
Localization: MCL
MCL:          On
Loop closure: Off
Speed limit:  1.2 m/s
```

CLI:

```bash
uv run awake-sim localization-bench --laps 1
uv run awake-sim map-and-race --laps 1
uv run awake-sim map-and-race --laps 5
uv run awake-sim slam-bench --duration 20 --mcl
```

## Reports

- [`EXP-0001` strategy baseline](docs/experiments/EXP-0001-strategy-baseline.md)
- [`EXP-0002` SLAM front-end](docs/experiments/EXP-0002-slam-frontend.md)
- [`EXP-0003` MCL](docs/experiments/EXP-0003-mcl.md)
- [`EXP-0004` GT vs odom vs MCL](docs/experiments/EXP-0004-localization-bench.md)
- [`EXP-0005` map once, race with MCL](docs/experiments/EXP-0005-map-and-race.md)
- Devlog: [`docs/devlog/2026-10-30.md`](docs/devlog/2026-10-30.md)
