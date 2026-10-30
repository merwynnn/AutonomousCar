# EXP-0003 — MCL on the front-end map

- Date: 2026-10-30
- Track: PDF-like test track
- Controller: pure pursuit (GT pose for driving)
- Sensors: realistic 720-beam lidar, drifting wheel odometry
- SLAM: front-end only (loop closure off)
- MCL: 400 particles, 5 cm occupancy map, 0.015 m / 0.012 rad motion noise

## Question

Can a particle-filter MCL localize on the map produced by the SLAM front-end,
and how does it compare with the front-end pose?

## Results

`uv run awake-sim slam-bench --duration 12 --mcl`:

| Metric | SLAM front-end | MCL |
|---|---:|---:|
| ATE mean | 22.60 cm | 21.95 cm |
| ATE max | 31.45 cm | 41.13 cm |
| Scan/update time | 13.0 ms mean / 17.1 ms p95 | included |
| Effective particles (last) | — | 281 / 400 |
| Particle spread (last) | — | 0.20 m |
| Odom drift (last) | 0.42 m | — |

The live dashboard snapshot at 94 scans showed SLAM ATE 8.3 cm and MCL ATE
5.8 cm; the difference is mainly which part of the map had been observed.

## Interpretation

- MCL tracks the map and produces a smooth pose with a healthy particle cloud
  (ESS 200–340, spread 0.1–0.3 m).
- It does **not** fix map drift. Both estimators are limited by the front-end map;
  their ATE grows together as the map drifts.
- MCL is initialized around the first SLAM pose, so this is local localization,
  not global relocalization.
- The particle filter adds only a few milliseconds; scan matching dominates the
  cost.

## Debug controls

The dashboard exposes:

- MCL off/on;
- loop closure off/on (experimental, default off);
- reset MCL;
- layer toggles for map, SLAM, MCL, particles, odometry and GT trail;
- metrics for SLAM ATE, MCL ATE, ESS, spread, odom drift, loop closures and
  graph size.

## Caveats

- MCL uses a map built by the drifting front-end. A loop-corrected map would
  improve it.
- No global relocalization / kidnapped-robot test yet.
- The loop-closure detector is disabled because scan-only place recognition
  produced false positives on the repetitive walls.
- MCL is not yet used by the racing controller; pure pursuit still uses GT pose.

## Reproduction

```bash
uv run awake-sim slam-bench --duration 20 --mcl
```

## Decision

Keep MCL as a localization layer and expose the toggle for experiments. Next:

1. global relocalization (spread particles over the map, recover from a wrong
   initial pose);
2. reliable loop closure, then re-evaluate MCL on the corrected map;
3. feed MCL pose into pure pursuit instead of ground truth.
