# EXP-0004 — GT vs odometry vs MCL pure pursuit

- Date: 2026-10-30
- Track: PDF-like test track
- Controller: pure pursuit
- Speed limit: 1.2 m/s
- Racing line: centred/safe line (lateral margin 0.35 m)
- Realism: `realistic`, 720-beam lidar, drifting wheel odometry
- Loop closure: off

## Question

Can pure pursuit drive without ground truth, and what does localization
quality cost in lap time?

## Results (1 lap)

| Pose source | Lap time | ATE mean | ATE max | Status |
|---|---:|---:|---:|---|
| GT | 20.12 s | 0.0 cm | 0.0 cm | OK |
| Odom | DNF | 36.4 cm | 75.1 cm | hit wall |
| MCL | **20.57 s** | **23.0 cm** | 38.8 cm | OK |

## Interpretation

- **MCL is only ~2% slower than ground truth** at this conservative speed.
- **Odometry-only fails**: with the procedural landmarks the run drifts to
  36.4 cm mean / 75.1 cm max and hits a wall.
- MCL is not perfect, but it completes the lap and is clearly more robust than
  odometry.
- The safe line matters: with the aggressive line, MCL/odometry crashed at the
  right lobe even with the same localization.
- No ground truth enters the MCL control loop. GT is only the comparison mode.

## Caveats

- One lap only; the 5-lap map-and-race run (EXP-0005) shows MCL drift over
  multiple laps.
- The safe line is slower than the aggressive racing line.
- Loop closure is off.
- The car uses a reverse recovery for stuck conditions.

## Reproduction

```bash
uv run awake-sim localization-bench --laps 1
```

## Decision

MCL-driven pure pursuit is viable at conservative speed. Next: global MCL
relocalization or periodic map/pose correction before increasing speed to 2+ m/s.
