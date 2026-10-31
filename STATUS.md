# Project status

> Last updated: 2026-10-30
> Current milestone: **M3 in progress — MCL drive + map-once/race experiments**

![M3 SLAM/MCL dashboard](docs/assets/2026-10-30-m3-slam.png)

## Headline results

### Localization comparison (1 lap, 1.2 m/s, growing SLAM map)

| Pose source | Lap time | ATE mean | Status |
|---|---:|---:|---|
| GT | 20.12 s | 0.0 cm | OK |
| Odom | DNF | 20.0 cm | hit wall |
| **MCL** | **27.56 s** | **39.2 cm** | **OK** |

**MCL completes the lap with no ground truth in the control loop**, while
odometry drifts into a wall. The on-track landmarks make the growing-map test
harder for MCL; the frozen-map five-lap race below is the stronger result.

### Map once, race with MCL

| Metric | Result |
|---|---:|
| Mapping lap (MCL, no GT) | 24.04 s |
| Map seam (start vs end pose) | 20.5 cm |
| 5-lap race at 0.9 m/s | **5/5, 132.94 s** |
| 5-lap best lap | 26.31 s |
| Lap times | 26.40, 26.84, 26.99, 26.41, 26.31 s |
| 5-lap MCL ATE mean / max | 25.7 / 91.0 cm |
| Tuned MCL | 400 particles, rank_keep 0.99, stride 3 |

Mapping and racing are both sensor-only (MCL, no GT). The five-lap race
**completes at 0.9 m/s** with global relocalization off. At higher speed it
still loses lock near the map seam. The eight on-track landmarks are now
outside-of-corner crates/poles, placed inside the clear floor boundary rather
than embedded in the wall pipes.

### Furthest-gap benchmark (lidar only)

| Metric | Result |
|---|---:|
| 5 laps at 3.2 m/s | **5/5, 85.65 s** |
| Best lap | **15.72 s** |
| Lap times | 18.32, 17.40, 17.25, 16.97, 15.72 s |
| Inputs | one lidar scan; no map, no pose, no GT |

The controller scores a 60° clearance cone around every forward direction,
steers toward the most open line and holds the speed limit. It is fast but
local and greedy; 2.8 m/s actually DNF while 3.2 m/s completed, so the result is
not a safety proof. See
[`EXP-0006`](docs/experiments/EXP-0006-gap-follow.md).

## What works today

- PyBullet 1/10 4WD Ackermann car, 240 Hz physics, 16 cm pipe walls.
- Eight procedural on-track crates/poles (outside of corners): collidable,
  lidar-visible SLAM landmarks, rendered in the dashboard.
- 720-beam non-ideal lidar, three ultrasonic sensors, drifting wheel odometry.
- Wall-follow, pure pursuit, furthest-gap and manual driving.
- Racing line + speed profile with a speed-limit slider.
- Localization modes: **GT / Odom / MCL**; MCL drives pure pursuit with no GT.
- 400-particle MCL with ESS/spread diagnostics, on/off/reset.
- C++ SLAM front-end: deskew, occupancy grid, correlative matching; map freeze
  and save.
- SE(2) pose graph with keyframes; loop-closure detector experimental/off.
- Dashboard: live 3D, lidar, SLAM/MCL/odometry ghosts and trails, particles,
  map, debug layers, **Car view = Actual / Controller / SLAM**, gap vector and
  gap readout.
- CLI: `race`, `compare`, `gap-bench`, `localization-bench`, `map-and-race`,
  `slam-bench`.
- 27 Python tests + 2 C++ tests; frontend builds.

## Milestone status

- **M0 ✅** — bootstrap, docs, dashboard, C++ binding.
- **M1 ✅** — realistic sensors, wall-follow, 5-lap acceptance.
- **M2 core ✅** — racing line, pure pursuit, strategy benchmark; generator/editor
  still pending.
- **M3 front-end ✅** — SLAM tracking, map, MCL, localization modes, map-once race
  experiments; reliable loop closure/global MCL pending.

## Next tasks

1. Add a local obstacle-avoidance safety layer so pure pursuit tolerates
   on-track landmarks with MCL pose error.
2. Improve mapping/MCL consistency with on-track landmarks (seam is 20.5 cm).
3. Global MCL relocalization / periodic correction at the start line.
4. Reliable loop closure or a visible start-line landmark.
5. More robust controller (Stanley/MPC) for MCL at higher speed.
6. M2 rest: track generator/editor.
7. Calibration against the real car; Pi port later.

## Known issues

- 5-lap MCL race completes only at the conservative 0.9 m/s; at 1.2 m/s it
  sticks near the seam.
- Growing-map MCL ATE is 39.2 cm with the on-track landmarks (1 lap, 1.2 m/s);
  the frozen-map race is better at 25.7 cm mean but 91.0 cm max.
- Global relocalization is experimental; on this symmetric track it can converge
  to the wrong place, so it is off by default.
- Loop closure is experimental and off; scan-only detection false-positives on
  repetitive walls.
- MCL is local, not global; no kidnapped-robot recovery.
- Odom-only pure pursuit fails at the tested settings.
- Safety/recovery is minimal.
- Furthest-gap is a local greedy strategy; 3.2 m/s is not a proven safe limit.
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
Speed limit:  0.9 m/s
```

For the lidar-only **furthest-gap benchmark**:

```text
Driving mode: Auto
Controller:   Gap
Speed limit:  3.2 m/s (set automatically by the Gap button)
```

CLI:

```bash
uv run awake-sim gap-bench --laps 5 --speed 3.2
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
- [`EXP-0006` furthest-gap benchmark](docs/experiments/EXP-0006-gap-follow.md)
- Devlog: [`docs/devlog/2026-10-30.md`](docs/devlog/2026-10-30.md)
