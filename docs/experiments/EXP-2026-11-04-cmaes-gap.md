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

The policy input is exactly `Observation`: lidar (raw or estimated-odometry
compensated), the wheel+IMU `StateEstimate`, actuator limits and `dt`. Training
fitness uses simulator progress, lap time and physics contacts, but those values
are never passed to the controller. `tests/test_learned_gap.py` includes a
regression test that falsifies the estimated pose and asserts the actuation is
unchanged.

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
by the host filesystem; the exported 9.33 s policy is the verified record. A
later random-track refinement improved it to **9.10 s** (see below).

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

## Random-track generalisation

The reference-trained B2a policy was evaluated on six held-out random tracks
before any random-track training. All runs used the same policy, seed and
sensor model; only the track changed.

| Track seed | Object density | Object size | Width | Result | Contacts |
| ---: | ---: | ---: | ---: | --- | ---: |
| 1 | 0.0 | 1.4 | 0.90 m | DNF at 14.5/24.7 m | 1 |
| 2 | 0.3 | 1.4 | 0.90 m | DNF at 7.9/22.1 m | 1 |
| 3 | 0.6 | 1.4 | 0.90 m | **7.93 s** | 0 |
| 4 | 0.9 | 1.5 | 0.90 m | DNF at 6.1/24.3 m | 1 |
| 5 | 0.5 | 1.6 | 0.88 m | **8.97 s** | 0 |
| 6 | 0.5 | 1.2 | 1.00 m | DNF at 10.7/24.2 m | 1 |

Only 2/6 clean: the reference policy is a strong but track-specific reactive
controller. This is the motivation for randomised-track training. A candidate
is still exported only after it validates on the reference track.

Random-track training command:

```bash
uv run python -m training.cma_opt --stage b2a --generations 40 --popsize 12 \
  --sigma0 0.08 --workers 6 --init algorithms/weights/policy.npz \
  --random-track --reference-track-prob 0.25 \
  --object-density-range 0.15 0.75 --object-scale-range 1.10 1.60 \
  --track-width-range 0.85 1.00 --out data/training/policy_random.npz
```

Robust random-track B1 recipe (new track every generation, no reference
generations; 60 s allows slow clean laps):

```bash
uv run python -m training.cma_opt --stage b1 --generations 40 --popsize 12 \
  --sigma0 0.04 --workers 6 --init data/training/b1_robust_init.npz \
  --out data/training/policy_robust.npz \
  --random-best-out data/training/policy_robust_random.npz \
  --random-track --reference-track-prob 0.0 --max-time 60 \
  --object-density-range 0.0 0.9 --object-scale-range 1.0 1.8 \
  --track-width-range 0.85 1.0 --object-scale-jitter 0.40 --track-seed 2027
```

Held-out success-rate sweep:

```bash
uv run python -m training.evaluate --policy algorithms/weights/policy.npz \
  --random-sweep 10 --max-time 60 --track-seed 123
```

The generator scales crate/pole footprints and clamps each object so its
footprint stays inside the lane. Per-object jitter mixes small and larger
obstacles.

The first random-track B2 run improved the reference-best clean lap to
**9.10 s**, but it was clean on only **3/6** held-out tracks. A later B1
random-track run reached **9.95 s** on the reference yet was **0/10** on a fresh
sweep: with one track per generation, CMA-ES can fit the training sequence
instead of learning a robust policy. Both were rejected for the default
controller.

The shipped policy is instead a **robustness-first B1**: the tuned B1 gap
controller with a 2.2 m/s base, a curvature speed cap (0.6), and clearance
slowdown thresholds of 0.6 m / 1.1 m. It is selected on held-out tracks, not on
training-track lap time.

| Policy | Reference sub-rays 1 | Reference sub-rays 3 | Fixed tracks | Fresh sweeps |
| --- | ---: | ---: | ---: | ---: |
| Old reference-only B2a | 9.33 s | 9.35 s | 2/6 | — |
| Fast B2 random-trained | 9.10 s | 9.12 s | 3/6 | — |
| B1 random-trained | 9.95 s | — | — | 0/10 |
| **Robust B1 (shipped)** | **15.63 s** | **15.67 s** | **6/6** | **27/30** |

The robust B1 is slower but it actually completes unseen tracks. Its throttle
varies from 0.36 to 0.47 (target speed 1.53–2.01 m/s) as clearance and
curvature change, so it is not a constant-speed controller.

The generator now also caps the centreline curvature at 1.6 1/m so it does not
produce near-undrivable hairpins, and every object footprint is clamped inside
the lane. Remaining random-track failures are dominated by track/obstacle
geometry rather than the controller.

**Next step for random training:** evaluate each candidate on 2–3 generated
tracks per generation (worst-case or mean fitness) instead of one. That prevents
the sequence-fitting seen above and should let a random-trained policy keep the
robustness while recovering speed.

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
