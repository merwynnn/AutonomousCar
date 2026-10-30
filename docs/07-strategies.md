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

- SQLite experiment database;
- CSV export;
- dashboard leaderboard and Pareto plots;
- Markdown report in `docs/experiments/`.

## 6. Batch execution

- Each run is a headless simulation process with a fixed seed.
- A worker pool runs N processes in parallel (default: 4, configurable).
- Progress and cancellation are exposed in the dashboard.
- Results are reproducible: same config + seed → same metrics.
- A batch manifest records code version, config hash and environment.

## 7. Hardware what-if

Because the car is not chosen yet, the lab can sweep:

- mass, centre of mass, wheelbase, track width;
- motor torque/current limit, gear ratio, battery voltage;
- tire friction front/rear;
- lidar height/mount and sensor quality;
- steering limits and latency.

Outputs: lap-time sensitivity, Pareto front of speed vs robustness, and warnings
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

## 10. Interpreting results

- Compare distributions, not single runs.
- A faster strategy that crashes is not better; report crash rate and penalties.
- A strategy that only works with ground truth is not deployable.
- A strategy that only works on the PDF track may overfit; check generated tracks.
- Differences smaller than run-to-run noise are not meaningful.
