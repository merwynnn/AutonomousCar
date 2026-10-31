# Strategies and experiments

## 1. Goal

Find the fastest **safe** strategy for five consecutive laps on an unknown track,
using only non-ideal onboard sensors. The lab compares controllers, racing lines,
speed profiles and localization modes under controlled conditions.

## 2. Controllers

| Controller | Input | Strength | Weakness |
|---|---|---|---|
| Wall follow (PID) | Side lidar ranges | Simple, robust | Slow, sensitive to gaps |
| Gap follow | Lidar gap angles | Fast on open corners | Can cut corners, needs tuning |
| Pure pursuit | Centreline/racing line + pose | Smooth, tunable | Needs localization, corner cutting |
| Stanley | Centreline + heading error | Good tracking | Sensitive to noisy heading |
| MPC | Vehicle model + reference | Speed/comfort constraints | Compute cost, model mismatch |

All controllers implement the same interface and receive only latency-filtered
sensor data. Controller parameters live in YAML and can be tuned live.

## 3. Global planning

### 3.1 Centreline extraction

From the occupancy map:

1. compute a distance transform;
2. extract a skeleton / Voronoi path;
3. prune spurs and close the loop;
4. resample by arc length;
5. compute tangent, normal and curvature.

A ground-truth centreline is available for baseline tests; the SLAM/map-derived
centreline is used for realistic runs.

### 3.2 Racing line

Find a smooth closed path inside the track boundaries that minimizes curvature
while keeping a safety margin from walls:

- quadratic-programming or iterative smoothing formulation;
- constraints: distance to walls ≥ half car width + margin;
- objective: curvature² + length penalty;
- validate against the car’s minimum turning radius.

### 3.3 Time-optimal speed profile

Given the racing line curvature `κ(s)`:

- lateral limit: `v ≤ sqrt(a_lat_max / |κ|)`;
- friction circle: `a_long² + a_lat² ≤ (μ g)²`;
- forward pass limited by motor power/traction;
- backward pass limited by braking;
- steering-rate and actuator limits;
- optional safety margin from sensor latency and localization uncertainty.

The result is a speed target per arc-length station; the controller tracks it.

## 4. Localization modes

| Mode | Pose source | Purpose |
|---|---|---|
| `gt` | Ground truth | Upper bound; debug only |
| `odom` | Wheel odometry + IMU | Shows drift cost |
| `slam_mcl` | SLAM map + MCL/scan matching | Realistic race mode |

Experiments report results per mode so controller quality is separated from
localization quality.

## 5. Experiment matrix

Dimensions:

- track preset (PDF-like) + generated seeds;
- controller and parameters;
- localization mode;
- sensor realism preset (`ideal` / `realistic` / `harsh`);
- latency multiplier;
- friction and battery presets;
- random seed.

Metrics:

- 5-lap total time, per-lap times, best lap;
- mean/std across seeds;
- crash/DNF rate, wall contacts, deformation penalties;
- intervention count;
- off-track time;
- maximum speed;
- SLAM ATE/RPE and map quality (when SLAM is active);
- compute time per step and per scan.

Outputs:

- controller comparison table from `awake-sim compare`;
- lap times and penalty summary in the console;
- Markdown report in `docs/experiments/`.

## 6. Headless runs

- Each `race` / `compare` run is a headless simulation with a fixed seed.
- Same config + seed → same metrics.
- The report records the command, seed and code revision.
- Run comparisons one at a time to avoid CPU contention with the dashboard.

## 7. Hardware what-if

Because the car is not chosen yet, the lab can sweep:

- mass, centre of mass, wheelbase, track width;
- motor torque/current limit, gear ratio, battery voltage;
- tire friction front/rear;
- lidar height/mount and sensor quality;
- steering limits and latency.

Outputs: lap-time sensitivity, speed-vs-robustness summary, and warnings
(e.g., lidar too high for the walls). This is a comparative tool, not a validated
digital twin until the real car is measured.

## 8. Race rules

The simulator implements the competition essentials:

- 5 consecutive laps; total time is the sum;
- start/finish crossing and complete-loop validation;
- wall grazing tolerated; deformation-level impact penalized;
- intervention/repositioning penalized;
- incomplete lap not counted.

## 9. Recommended experiment sequence

1. **Baseline:** wall follow and gap follow with `ideal` sensors; verify the track
   and timing.
2. **Realism:** repeat with `realistic` sensors and latency; quantify the lap-time
   cost of imperfection.
3. **Localization:** compare `gt`, `odom`, `slam_mcl` with pure pursuit.
4. **Racing line:** centreline vs optimized racing line with the speed profile.
5. **Controllers:** pure pursuit, Stanley, MPC on the best line.
6. **Robustness:** `harsh` sensors, latency, dropouts, friction changes.
7. **Generalization:** 20+ generated tracks; check overfitting.
8. **Hardware:** sensitivity sweeps to guide the real build.

## 10. Baseline result (2026-10-30)

Test track, 5 laps, `realistic` sensors:

| Controller | Total (s) | Best lap (s) | Localization |
|---|---:|---:|---|
| Wall-follow (lidar) | 148.34 | 29.46 | lidar only |
| Pure pursuit + racing line | 46.12 | 9.20 | ground truth (upper bound) |

Pure pursuit is 3.22× faster, but the result is not yet deployable because it
uses the ground-truth pose. See
[`experiments/EXP-0001-strategy-baseline.md`](experiments/EXP-0001-strategy-baseline.md).
The next comparison must repeat this with odometry-only and SLAM/MCL
localization, generated tracks and obstacles.

### 10.1 Furthest-gap benchmark (lidar only, 2026-10-30)

The furthest-gap controller uses one lidar scan, no map and no pose. It finds the
direction whose surrounding 60° cone has the largest minimum range, steers
toward it, and holds the configured speed limit. Five laps at **3.2 m/s**:

| Metric | Result |
|---|---:|
| Completed | **5 / 5** |
| Total | **85.65 s** |
| Best lap | **15.72 s** |
| Lap times | 18.32, 17.40, 17.25, 16.97, 15.72 s |
| Inputs | lidar only (no map, no GT) |

It is faster than the validated 0.9 m/s MCL race (132.94 s) but is a local,
greedy strategy with no route memory. The 2.8 m/s run DNF while 3.2 m/s
completed, so the speed result is not a safety proof. See
[`experiments/EXP-0006-gap-follow.md`](experiments/EXP-0006-gap-follow.md).

## 11. Interpreting results

- Compare distributions, not single runs.
- A faster strategy that crashes is not better; report crash rate and penalties.
- A strategy that only works with ground truth is not deployable.
- A strategy that only works on the PDF track may overfit; check generated tracks.
- Differences smaller than run-to-run noise are not meaningful.
