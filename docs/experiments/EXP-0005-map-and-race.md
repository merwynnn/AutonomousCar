# EXP-0005 — Map once, race with MCL

- Date: 2026-10-30
- Mapping: one slow lap, **MCL control** on the growing SLAM map, 1.2 m/s, safe line, lidar 1 sub-ray
- Race: frozen one-lap map, MCL control, 0.9 m/s, safe line
- Loop closure: off
- Global relocalization: off
- Track: PDF-like test track with eight on-track landmarks

## Question

Does a slow mapping lap produce a map good enough to race five laps with MCL
and no ground truth in the control loop?

## Results

| Metric | Result |
|---|---:|
| Mapping lap (MCL control, no GT) | 24.04 s |
| Map seam (start vs end pose) | **20.5 cm** |
| One-lap MCL race (first race lap) | 26.40 s |
| Race speed | 0.9 m/s |
| MCL tuning | 400 particles, rank_keep 0.99, stride 3, softmax 10 |
| 5-lap attempt: completed | **5 / 5** |
| 5-lap attempt: total | **132.94 s** |
| 5-lap attempt: lap times | 26.40, 26.84, 26.99, 26.41, 26.31 s |
| 5-lap attempt: MCL ATE mean / max | 25.7 / 91.0 cm |
| 5-lap attempt: status | **OK** |

## Interpretation

- A sensor-only MCL mapping lap produced a **20.5 cm seam**. It is good enough
  for five MCL laps at 0.9 m/s, but not for faster runs.
- MCL error is not monotonically growing: the frozen-map race averages 25.7 cm
  but reaches 91.0 cm near the seam. Without a global correction, particles can
  lock onto the drifting local map.
- The on-track landmarks add distinctive scan features. They also make the
  growing-map one-lap test harder: MCL ATE rises to 39.2 cm and odometry drifts
  into a wall. The frozen-map race remains the stronger result.
- Tuning result: aggressive rank pruning (0.95/0.9) improved one lap but failed
  the 5-lap run. A mild **rank_keep 0.99** with 400 particles and stride 3 is the
  reliable setting. Ranking is applied only when resampling is triggered.
- Global relocalization was tested and made the run worse on this symmetric
  track: the scan-only scatter can converge to the wrong place. It is now
  experimental, off by default, and triggered only manually or when stuck.
- The mapping lap is slower than the race because it uses the stable no-rank
  MCL settings and a conservative speed profile. That trade is intentional:
  mapping quality first, then race reliability.

## What this means

A **one-lap frozen map is sufficient for five MCL laps at 0.9 m/s**, but the
20.5 cm seam and 91.0 cm maximum error are still the limiting factors. The next
fixes are:

1. a local obstacle-avoidance safety layer so pure pursuit tolerates on-track
   landmarks with MCL pose error;
2. global MCL relocalization or periodic correction at the start line;
3. reliable loop closure (or a visible start-line landmark) to remove the map
   seam;
4. lower race speed until MCL drift is bounded;
5. a more robust controller (Stanley/MPC) that tolerates larger pose error.

## Reproduction

```bash
uv run awake-sim map-and-race --laps 1
uv run awake-sim map-and-race --laps 5
```

## Decision

Keep the map-once/race-with-MCL workflow as the main sensor-only experiment. It
completes five laps at 0.9 m/s; higher speeds and stronger on-track landmarks
still need a better map/seam or a local avoidance layer before they can be
claimed as reliable.
