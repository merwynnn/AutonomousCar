# EXP-0001 — Wall-follow vs pure pursuit

- Date: 2026-10-30
- Code: M2 working tree
- Track: PDF-like test track, 0.9 m lane, 16 cm pipe walls
- Vehicle: generic 1/10 4WD Ackermann
- Laps: 5
- Realism: `realistic`
- Localization: wall-follow uses lidar only; pure pursuit uses ground-truth pose
- Lidar: 720 beams, live mode (1 sub-ray), noise/dropouts/latency enabled

## Question

How much lap time is available from a racing line and a path-tracking controller,
compared with a reactive lidar wall-follower?

## Setup

| Item | Value |
|---|---|
| Simulator | PyBullet, 240 Hz physics, 60 Hz control |
| Sensors | 720-beam lidar + 3 ultrasonic + IMU/odometry |
| Wall-follow | lidar sector controller, base speed 2.4 m/s |
| Pure pursuit | racing line + time-optimal speed profile, 0.55 m lookahead |
| Start | bottom-centre start/finish line |
| Penalties | none recorded |

## Results

| Controller | Total 5 laps (s) | Best lap (s) | Mean lap (s) | Max speed (m/s) | Crashes |
|---|---:|---:|---:|---:|---:|
| Wall-follow (lidar) | 148.34 | 29.46 | 29.67 | ~1.9 | 0 |
| Pure pursuit (GT) | **46.12** | **9.20** | **9.22** | ~3.2 | 0 |

Pure pursuit is **3.22× faster** on total time. The racing line stayed within
0.173 m of the centreline (lane half-width 0.45 m) and the car remained on the
floor (z ≈ 0.09 m) for the whole run.

## Interpretation

- Most of the gain comes from cornering at the friction limit instead of crawling
  through the waist corners.
- The wall-follower is conservative because it slows whenever the front sector
  sees a wall, which is almost always true on a 0.9 m lane.
- Pure pursuit with a good line is smooth and consistent (9.20–9.29 s), but this
  result uses the ground-truth pose and is therefore an **upper bound**.
- The realistic deployable result must use SLAM/MCL localization and will be
  slower; M3 will measure that gap.

## Caveats

- No obstacles were present.
- Pure pursuit did not use the lidar for control; the sensor stack was still
  simulated but not consumed by the controller.
- The racing line is a heuristic corner-cutting path, not a full minimum-curvature
  optimization.
- The vehicle model is generic; real-car calibration is pending.

## Reproduction

```bash
uv run awake-sim race --controller wall_follow --laps 5
uv run awake-sim race --controller pure_pursuit --laps 5
# or both:
uv run awake-sim compare --laps 5
```

## Decision

Keep both controllers:

- wall-follow as the sensor-only fallback / recovery behaviour;
- pure pursuit + racing line as the performance baseline.

Next experiments:

1. pure pursuit with odometry-only and SLAM/MCL localization;
2. minimum-curvature racing line vs the heuristic line;
3. generated tracks with obstacles;
4. MPC vs pure pursuit at the limit;
5. parameter sweep of lookahead, speed scale, friction and latency.
