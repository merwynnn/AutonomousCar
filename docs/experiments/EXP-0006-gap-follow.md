# EXP-0006 — Furthest-gap lidar benchmark

- Date: 2026-10-30
- Track: PDF-like test track with eight on-track procedural landmarks
- Controller: `FurthestGapController` (`src/awake_sim/control/gap_follow.py`)
- Inputs: one 720-beam lidar scan, measured speed for the recovery layer
- No map, no localization pose, no ground truth
- Command: `uv run awake-sim gap-bench --laps 5 --speed 3.2`

## Question

How fast can a deliberately simple, lidar-only strategy complete five laps:
find the most open direction in the scan, steer toward it, and hold the
configured speed limit?

## Controller

Every scan:

1. keep valid beams in a ±100° front sector;
2. treat invalid/no-return beams as open space;
3. smooth ranges with a short circular 7-beam average;
4. for every direction, compute the minimum range in a 60° window centred on
   that direction;
5. choose the direction with the largest clearance, with a small bias toward
   straight ahead;
6. steer toward it with first-order smoothing;
7. command the configured speed limit, braking only when the clearance is below
   0.8 m (and harder below 0.5 m).

The 60° window is the key safety term. A single-beam "farthest point" selector
oscillated, and a narrow ±10° window clipped a crate at +12–17° before seeing it.
The wide window is conservative but keeps the original idea: pick the most open
line, then go fast. The existing stuck-recovery layer reverses the car if it is
pinned; it uses measured speed only, not ground truth.

## Tuning and reliability

All runs are deterministic and use the final on-track obstacle layout. Five-lap
runs:

| Speed limit | Clearance window | Result | Total | Best lap |
|---:|---:|---|---:|---:|
| 2.4 m/s | 60° | **5/5** | 88.81 s | 16.98 s |
| 2.8 m/s | 60° | DNF 1/5 | 18.13 s | 18.13 s |
| 3.2 m/s | 60° | **5/5** | **85.65 s** | **15.72 s** |
| 3.6 m/s | 60° | DNF 1/5 | 16.57 s | 16.57 s |
| 4.0 m/s | 60° | DNF 0/5 | — | — |

The 3.2 m/s run was repeated and produced the identical lap times. The 2.8 m/s
DNF is not a monotonic speed effect: contact dynamics and the chosen gap differ
enough that it can get pinned while 3.2 m/s completes. This is a reminder that a
single pass/fail run is not a safety proof.

The 60° window was selected after these one-lap checks:

| Window | 1.2 m/s | 2.4 m/s |
|---:|---|---|
| 20° | flip | flip |
| 30° | flip | flip |
| 40° | flip | flip |
| 50° | 1 lap | flip |
| 60° | 1 lap | 1 lap |

## Final five-lap run (3.2 m/s)

```text
Status:        OK (5/5)
Total:         85.65 s
Best lap:      15.72 s
Lap times:     18.32, 17.40, 17.25, 16.97, 15.72 s
Controller:    furthest gap, lidar only
Map / pose:    none
```

## Interpretation

- A controller with one scan, no map and no pose completes five laps at
  3.2 m/s. It is **1.55× faster than the validated 0.9 m/s MCL race**
  (132.94 s), but it is not a deployable race strategy yet: it has no route
  memory and no global consistency.
- The controller cuts corners and relies on the local opening. On this track it
  eventually finds the start/finish line because the corridor is closed, but it
  has no concept of a lap or a racing line.
- The gap vector is a useful visualization: it shows exactly which opening the
  controller is committing to, and the clearance/opening readout explains why it
  brakes or turns.

## Limitations

- Local, greedy strategy: a single bad gap choice can pin the car.
- No map, no loop closure, no global relocalization.
- Speed-limit reliability is not monotonic; 3.2 m/s completed while 2.8 m/s
  did not.
- Obstacles are on the outside of corners. More central obstacles would require
  a stronger local avoidance layer or a wider clearance window.
- The benchmark is a one-track result, not a generalization claim.

## Reproduction

```bash
uv run awake-sim gap-bench --laps 5 --speed 3.2
uv run awake-sim race --controller gap_follow --laps 5 --speed 3.2
uv run awake-sim compare --laps 5 --speed 2.4
```

Dashboard: choose **Auto → Gap**. The button sets the 3.2 m/s benchmark speed,
draws the selected opening as a magenta vector and shows gap bearing, range,
clearance and opening width.

## Decision

Keep the furthest-gap controller as a **simple lidar-only benchmark**, not as the
main race controller. It provides a fast, easy-to-explain lower bound and a
useful debugging view for the lidar and the local planner.
