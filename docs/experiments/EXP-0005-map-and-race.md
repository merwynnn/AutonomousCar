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
| Race speed | 0.9 m/s |
| Global relocalization | off |
| MCL tuning | 400 particles, rank_keep 0.99, stride 3, softmax 10 |
| 5-lap attempt: completed | **5 / 5** |
| 5-lap attempt: total | **134.32 s** |
| 5-lap attempt: lap times | 27.12, 26.62, 27.13, 26.80, 26.65 s |
| 5-lap attempt: MCL ATE mean / max | 23.3 / 60.8 cm |
| 5-lap attempt: status | **OK** |

## Interpretation

- A sensor-only MCL mapping lap produced a **8.3 cm seam** (slower speed and a
  safer line than the previous GT mapping test). It is good enough for one MCL
  lap at 1.2 m/s, but not yet proven for five consecutive laps.
- MCL error grows across laps (22.6 cm mean in the one-lap comparison, 45.3 cm
  in the 5-lap attempt). Without a global correction, particles slowly lock onto
  the drifting local map.
- At 1.2 m/s the car got stuck on lap 2. At **0.9 m/s** all five laps completed,
  with MCL ATE 23.3 cm mean / 60.8 cm max. Speed is currently the safety margin
  that keeps MCL tracking through the seam.
- Tuning result: aggressive rank pruning (0.95/0.9) improved one lap but failed
  the 5-lap run. A mild **rank_keep 0.99** with 400 particles and stride 3 is the
  reliable setting. Ranking is applied only when resampling is triggered.
- Global relocalization was tested and made the run worse on this symmetric
  track: the scan-only scatter can converge to the wrong place. It is now
  experimental, off by default, and triggered only manually or when stuck.
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

The map seam is the key diagnostic: the sensor-only mapping lap reduced it to
8.3 cm. At 0.9 m/s, five laps complete with MCL ATE staying at tens of
centimetres. At 1.2 m/s the car sticks near the seam. The next gains must come
from global relocalization done properly, loop closure, or a more robust
controller.

## Reproduction

```bash
uv run awake-sim map-and-race --laps 1
uv run awake-sim map-and-race --laps 5
```

## Decision

Keep the map-once/race-with-MCL workflow as an experiment, but do not claim a
successful five-lap MCL race yet. Add global relocalization or a proper loop
closure before increasing race speed.
