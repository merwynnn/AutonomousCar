# EXP-0005 — Map once, race with MCL

- Date: 2026-10-30
- Mapping: one slow lap, **MCL control** on the growing SLAM map, 1.2 m/s, safe line, lidar 1 sub-ray
- Race: frozen one-lap map, MCL control, 1.2 m/s, safe line
- Loop closure: off
- Track: PDF-like test track

## Question

Does a slow mapping lap produce a map good enough to race five laps with MCL
and no ground truth in the control loop?

## Results

| Metric | Result |
|---|---:|
| Mapping lap (MCL control, no GT) | 20.81 s |
| Map seam (start vs end pose) | **8.3 cm** |
| One-lap MCL race | 20.15 s |
| One-lap MCL ATE mean / max | 23.8 / 47.6 cm |
| 5-lap attempt: completed | **3 / 5** |
| 5-lap attempt: total completed time | 61.44 s |
| 5-lap attempt: lap times | 20.72, 20.35, 20.37 s |
| 5-lap attempt: MCL ATE mean / max | 45.3 / 69.7 cm |
| 5-lap attempt: status | **DNF** (stuck after lap 3) |

## Interpretation

- A sensor-only MCL mapping lap produced a **8.3 cm seam** (slower speed and a
  safer line than the previous GT mapping test). It is good enough for one MCL
  lap at 1.2 m/s, but not yet proven for five consecutive laps.
- MCL error grows across laps (22.6 cm mean in the one-lap comparison, 45.3 cm
  in the 5-lap attempt). Without a global correction, particles slowly lock onto
  the drifting local map.
- The car got stuck after lap 3 despite reverse recovery. This is a localization
  failure, not a controller tuning issue.
- Mapping is faster than the race only because it was configured at 1.5 m/s and
  a shorter line; at equal settings GT is 20.12 s, MCL 20.40 s.

## What this means

A **one-lap frozen map is promising but not sufficient** for a five-lap MCL race.
The next fixes are:

1. global MCL relocalization or periodic localization reset at the start line;
2. reliable loop closure (or a visible start-line landmark) to remove the map
   seam;
3. lower race speed until MCL drift is bounded;
4. a more robust controller (Stanley/MPC) that tolerates larger pose error.

The map seam is the key diagnostic: the previous 19.2 cm seam created the local
ambiguity that eventually grew into a 0.7 m MCL error. The sensor-only mapping
lap reduced the seam to 8.3 cm; whether that is enough for five laps is being
measured.

## Reproduction

```bash
uv run awake-sim map-and-race --laps 1
uv run awake-sim map-and-race --laps 5
```

## Decision

Keep the map-once/race-with-MCL workflow as an experiment, but do not claim a
successful five-lap MCL race yet. Add global relocalization or a proper loop
closure before increasing race speed.
