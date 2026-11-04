# EXP-2026-11-04 — CMA-ES lidar gap policy

## Question

Can evolutionary policy search produce a sensor-only controller that beats the
hand-tuned furthest-gap baseline on the reference AWAKE 2027 lap, with zero
wall/obstacle contact and a reproducible one-lap time?

## Setup

| Item | Value |
| --- | --- |
| Scenario | `configs/sim/awake_2027.yaml` |
| Sensors | realistic |
| Laps per rollout | 1 |
| Physics / control | 240 Hz / 60 Hz |
| Lidar sub-rays | 1 (training), 1 and 3 (verification) |
| Seed | 42 |
| Contact gate | every physics substep; wall/obstacle geoms only |
| Workers | 6 |
| Fitness | clean finish `1e6 + (32 - lap_time) - penalties`; dirty runs negative, ranked by progress |

Training is in `training/`; the runtime policy is
`algorithms/learned_gap.py` with weights in `algorithms/weights/policy.npz`.

## Baselines

| Controller | Lap | Contacts | Notes |
| --- | ---: | ---: | --- |
| `gap_follow`, full throttle, sub-rays 1 | 25.55 s | 0 | hand-tuned reference |
| `gap_follow`, full throttle, sub-rays 3 | 25.82 s | 0 | higher lidar fidelity |
| `gap_follow`, 1.5 m/s limit, sub-rays 3 | 32.73 s | 0 | default dashboard speed |

## B1 — structured gap controller

23 parameters: front sector, clearance window, gap threshold, smoothing,
scoring weights, aim blend, steering gain/rate, and a clearance/curvature speed
schedule. CMA-ES starts from the hand-tuned defaults.

| Run | Generations | Pop | Lap | Contacts | Top / mean speed |
| --- | ---: | ---: | ---: | ---: | --- |
| Exploration | 4 | 12 | 14.55 s | 0 | — |
| Refinement | 50 | 16 | **9.93 s** | **0** | 2.74 / 2.56 m/s |

The exported B1 policy was verified at sub-rays 1 and 3. It is stable and
clearly beats the 25.55 s baseline.

## B2a — linear beam scorer

36 downsampled beams over the front sector, 12 features per beam (range,
clearance, free fraction, side clearance, bearing, temporal consistency), a
linear beam score and a linear target-speed head. The policy steers toward the
highest-scoring beam.

Warm start: B1 is rolled out as an expert and the B2 features are regressed
onto B1's full-resolution score surface, not a single action. A direct
action-imitation fit collapsed to “drive straight” because most expert ticks
happen on straights. The warm-started policy completes a clean lap at a reduced
speed cap and then CMA-ES tunes all 45 parameters.

| Policy | Lap (sub-rays 1) | Lap (sub-rays 3) | Contacts | Top / mean speed | Min wall clearance |
| --- | ---: | ---: | ---: | --- | ---: |
| B2a warm start, speed cap 2.7 m/s | 10.38 s | — | 0 | — | — |
| B2a after 60 generations | **9.33 s** | **9.35 s** | **0** | 2.93 / 2.74 m/s | 0.215 m |

Command:

```bash
uv run python -m training.warmstart --laps 8 --iterations 0
uv run python -m training.cma_opt --stage b2a --generations 60 --popsize 16 \
  --sigma0 0.18 --workers 6 --init data/training/b2a_init.npz \
  --out algorithms/weights/policy.npz
```

The run also found a 9.32 s candidate at generation 59 that was not persisted
by the host filesystem; the exported 9.33 s policy is the verified record.

## B2b — residual hidden layer

B2a is expanded to a 16-unit residual hidden layer (605 parameters, sep-CMA-ES).
The expansion is numerically identical to B2a at initialisation, so it can only
improve the search without changing the runtime contract.

Result: **rejected**. The search found a 9.03 s candidate from a random start
offset, but the same vector contacted a wall at 3.84 m from the reference
start; the end-of-run reference-start validation also reported DNF with one
contact. Random-start offsets are useful for robustness, but candidates must be
re-validated at the reference pose before they can be promoted or exported.
B2a remains the shipped policy.

The runner now performs that reference-start validation on every promoted
candidate, so a future B2b run can only export a clean reference lap.

## Reproduction and monitoring

```bash
# preflight: starting lap, contacts and a rough total ETA
uv run python -m training.cma_opt --stage b2a --preflight ...

# live progress, elapsed, ETA and best lap
uv run python -m training.watch --stage b2a

# verify one lap at both lidar fidelities
uv run python -m training.evaluate --policy algorithms/weights/policy.npz \
  --subrays 1 3
```

## Observations

- The dominant gain comes from speed: the tuned policy averages 2.74 m/s where
  the baseline averages about 1.0 m/s.
- Contact detection at physics resolution was essential; a control-tick check
  misses brief wall touches and lets unsafe candidates look clean.
- Random start offsets during search prevent overfitting the launch pose, but
  the final candidate must be re-validated at the reference start. A B2b
  candidate that was fast from a random offset was dirty at the reference pose,
  which is why the runner now validates every promoted candidate there.
- The learned policy ignores the dashboard speed slider: it owns its speed
  plan, bounded by `observation.limits.max_speed_mps`.
