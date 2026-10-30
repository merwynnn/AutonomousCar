# Requirements traceability

Source of truth: `Cahier des charges 2027.pdf` (AWAKE Group, 2027 edition).
This document translates the specification into testable simulator requirements.
If the PDF and this table disagree, the PDF wins; update this table.

## Competition requirements

| ID | Requirement | Source | Simulator implication | Verification |
|---|---|---|---|---|
| R-01 | 1/10 scale miniature vehicle; chassis may be bought pre-assembled | §2.1, §8.1 | Vehicle model is a 1/10 RC car; chassis is a configurable preset | Vehicle config validation |
| R-02 | Max dimensions: length 45 cm, width 20 cm, height 20 cm | §8.1 | Geometry and collision shape constrained; dashboard warns if exceeded | Config test |
| R-03 | Electric motorisation; 4WD or 2WD; 4WD recommended | §8.1, §8.2 | Motor/ESC model; 4WD default, 2WD configurable | Physics test |
| R-04 | Ground clearance ≥ 5 mm | §4.1 | Chassis height parameter and floor-contact check | Config/physics test |
| R-05 | At least 3 ultrasonic sensors and one lidar (LD06-class reference) | §8.2 | 3 ultrasonic + 2D spinning lidar simulated by default | Sensor test |
| R-06 | Camera optional; IMU/accelerometer allowed; GPS forbidden | §8.2 | Camera interface optional; IMU simulated; no GPS module | Config/API test |
| R-07 | Indoor gym, track area 15 × 8 m, dark floor | §4.1 | World bounds, dark floor material, indoor lighting preset | Track test |
| R-08 | Track width 80–100 cm | §4.1 | Track generator and validator enforce width range | Track generator test |
| R-09 | Walls 16 cm high, grey VMC pipes | §4.1 | Pipe-wall geometry with height/radius; lidar sees over if mounted too high | Geometry/sensor test |
| R-10 | Training track is ring/peanut-like; competition track similar with extra obstacles, unknown until the day | §4.1, §4.1 | PDF-like preset + seeded generator + obstacle variants; no hard-coded competition layout | Track generation test |
| R-11 | Five consecutive laps; final time = sum of the five laps | §4.2 | Race manager counts 5 laps, records per-lap and total time | Race integration test |
| R-12 | Wall contact that deforms the track → intervention + 2-point penalty; light grazing tolerated | §5.2 | Contact impulse threshold; penalty event; optional visual displacement | Race rules test |
| R-13 | Team intervention/repositioning → 2-point penalty | §5.2 | Reposition command with penalty event; stuck detection | Race rules test |
| R-14 | An incomplete lap is not counted | §5.2 | Lap validation requires full loop and start/finish crossing | Race rules test |
| R-15 | Free practice before the run; competition order drawn at random | §4.1, §4.2 | Practice mode and race mode; no dependence on run order | UI/CLI test |
| R-16 | Budget 900 €; reports, kakémono and final presentation are part of the score | §2.1, §6, §7 | Simulator can include hardware cost in what-if presets; docs support reports | Optional cost report |

## Simulator requirements derived from the goals

| ID | Requirement | Rationale | Verification |
|---|---|---|---|
| S-01 | Default sensors are non-ideal: noise, bias, dropouts, latency, motion distortion | User requirement; realistic strategy ranking | Sensor realism tests |
| S-02 | Controllers/SLAM never receive ground truth or future messages | Fairness and realism | Latency/leakage test |
| S-03 | Fixed timestep, seeded RNG, recorded config hash | Reproducible experiments | Determinism test |
| S-04 | 2D lidar SLAM from scratch, C++ hot path, 10 Hz target | User requirement; Python too slow | Benchmark + ATE/RPE test |
| S-05 | Live 3D visualization of car, sensors, map, trajectories | Human monitoring | Screenshot/Playwright test |
| S-06 | Replay, experiment comparison, live tuning, track editor | User selected features | UI acceptance test |
| S-07 | Batch experiments over tracks/seeds/controllers | Strategy optimization | Batch integration test |
| S-08 | Portable C++ SLAM and hardware-agnostic controllers | Pi later | Build/API review |
| S-09 | English docs, AGENTS.md, STATUS.md, devlog, ADRs | User requirement | Docs review |
| S-10 | Git history and clean commits | Monitoring/reproducibility | Git log review |

## Test mapping

- `tests/test_config.py` — R-01..R-06, R-08 config validation.
- `tests/test_track.py` — R-07..R-10 generator/geometry.
- `tests/test_sensors.py` — R-05, R-09, S-01, S-02.
- `tests/test_race.py` — R-11..R-14.
- `tests/test_slam_reference.py` + `cpp/tests/` — S-04.
- `tests/test_api.py` — S-05/S-06 server contract.
- `docs/experiments/` — S-07 results.
- `docs/` + `AGENTS.md` — S-09/S-10.

## Open points

- The exact competition track and obstacle types are unknown; generated variants
  are used to avoid overfitting.
- Real car dimensions, mass, motor and sensors are not chosen yet; defaults are
  generic and will be replaced by measured values.
- The official race timing method is not specified; the simulator uses a
  ground-truth start/finish crossing for lap timing (referee role), while the
  controller only sees sensors.
