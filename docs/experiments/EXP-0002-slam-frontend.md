# EXP-0002 — SLAM front-end baseline

- Date: 2026-10-30
- Code: M3 front-end working tree
- Track: PDF-like test track
- Vehicle: generic 1/10 4WD Ackermann
- Controller: pure pursuit at 0.65 speed scale
- Sensors: 720-beam lidar, `realistic`, motion distortion, dropouts, latency
- Odometry: ground truth + 1 cm / 0.003 rad Gaussian noise
- SLAM: C++ occupancy grid (5 cm) + chamfer likelihood + coarse/fine correlative
  scan matching with a motion prior + per-beam deskew

## Question

Does the from-scratch 2D lidar SLAM front-end track the car accurately and fast
enough for 10 Hz operation?

## Results

`uv run awake-sim slam-bench --duration 8` (81 scans):

| Metric | Result |
|---|---:|
| Scans | 81 |
| ATE mean | **2.89 cm** |
| ATE max | 5.27 cm |
| Scan time mean | **14.2 ms** |
| Scan time p95 | **18.9 ms** |
| Occupied cells | 2,255 |
| Free cells | 9,179 |

A longer 20 s offline run with a 0.65-speed profile measured 1.8 cm mean ATE,
5.7 cm max ATE and 12.3 ms mean scan time (13.9 ms p95).

Live dashboard screenshot: [`assets/2026-10-30-m3-slam.png`](../assets/2026-10-30-m3-slam.png).

## Interpretation

- The front-end meets the 10 Hz desktop target (<20 ms p95) with margin.
- Deskewing the spinning-lidar scan before matching was essential; without it,
  the map smeared and the matcher drifted by metres.
- A motion prior around odometry prevents jumps between repetitive wall
  segments. With the prior, ATE stays at centimetre level.
- The live dashboard shows the occupancy map, the SLAM ghost pose and the SLAM
  trail; ATE in the screenshot was 0.8 cm.

## Caveats

- **No active loop closure.** The SE(2) pose graph exists and is unit-tested,
  but scan-only place recognition was not reliable on the repetitive pipe walls.
  A forced start-pose closure was tried and rejected as cheating. The map is
  built by the front-end only until a trustworthy descriptor/landmark is added.
- **No MCL yet.** Localization is scan matching against the running map, seeded
  by odometry.
- Odometry is ground truth + noise, not a realistic drifting wheel odometry.
  A drift/loop-closure test is the next step.
- At high speed some map artefacts appear outside the lane; map filtering and
  robust updates are planned.
- This is a simulator-only result; Pi benchmarking is later.

## Reproduction

```bash
uv run awake-sim slam-bench --duration 20
uv run pytest tests/test_slam_pipeline.py -q
```

## Decision

The front-end is good enough to drive the live SLAM visualization and to become
the M3 base. Next:

1. reliable place recognition (Scan Context or a visible start-line landmark);
2. loop-closure verification and pose-graph activation;
3. MCL / global relocalization;
4. map quality (IoU) and ATE/RPE reports on generated tracks;
5. Pi port and benchmark.
