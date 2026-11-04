# ADR-004 — CMA-ES-evolved lidar gap policy

- Status: accepted
- Date: 2026-11-04
- Branch: `algorithms-tests`

## Context

The hand-tuned furthest-gap controller completes the reference AWAKE 2027 lap
in about 25.5 s at full throttle with no contact. The simulator already exposes
a sensor-only `Observation`, so a faster policy must be learned from lidar,
wheel/IMU state and actuator limits alone. The policy must remain portable to
the Raspberry Pi: NumPy and `onboard.api` only, with no map and no host imports.

The goal is a single reference lap (`configs/sim/awake_2027.yaml`, realistic
sensors, seed 42) with **zero wall or obstacle contact**. Implicit track
memorisation is acceptable; privileged training information is not required.

## Decision

1. Optimise controller parameters with **CMA-ES** over one-lap simulator
   rollouts, using per-physics-substep contact detection so a brief wall touch
   cannot pass.
2. Search two policy families:
   - **B1**: a structured gap controller with 23 tunable parameters (scan
     window, scoring weights, steering gain/smoothing, speed schedule);
   - **B2**: a per-beam linear (and optionally residual-hidden) scorer with a
     speed head, warm-started from B1.
3. Keep the runtime in `algorithms/learned_gap.py`; it imports only NumPy and
   `onboard.api` and loads `algorithms/weights/policy.npz`.
4. Put all training code in the host-only `training/` package. Training may
   import MuJoCo and the simulator; the runtime may not.
5. Export a new policy only when it is a **clean** lap faster than the policy
   already on disk. Validate the winner at the reference start before export.
6. Use `cma` as an optional `train` dependency; it is not required to run the
   simulator, dashboard or car.
7. Support domain-randomised training on procedural tracks with random shape,
   lane width, object density and object size. Keep the reference track in a
   fraction of generations and always re-validate before export, so a lucky
   random circuit cannot silently replace the reference lap.

## Consequences

- The dashboard lists the controller as **Learned gap (CMA-ES)** and **Reload
  algo** picks up a newly exported `policy.npz` without restarting.
- The controller ignores the dashboard speed slider by design: the evolved
  policy owns its speed plan, bounded by `observation.limits.max_speed_mps`.
- The weights are tuned for the reference track. That is intentional for the
  competition, but a new track needs a new training run.
- CMA-ES is sample-hungry. Six workers is the measured throughput knee on the
  development machine; the runner reports ETA and writes machine-readable
  status files so long runs are observable.
- The policy receives only `Observation`. Training fitness uses true progress,
  lap time and physics contacts for supervision/evaluation; none of these are
  fed into the policy. The estimator starts at the known start pose, then runs
  on wheel/IMU data like the real car.

## Alternatives considered

- **Hand-tuning the gap controller**: tried first; plateaus around 25 s.
- **Behaviour cloning / DAgger from a privileged planner**: needs a teacher and
  adds a second policy stack; B1 already provides a strong warm start.
- **Deep RL (PPO/SAC)**: heavier dependency, less reproducible on CPU and
  harder to bound for safety than evolution strategies on ~10–600 parameters.
- **Model-predictive control**: strong, but a larger runtime and tuning burden
  for the Pi; not needed to beat the reference baseline.
