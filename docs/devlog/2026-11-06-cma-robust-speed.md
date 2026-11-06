# CMA-ES robustness + speed campaign series (B1/B2)

Goal: beat the 9.5 s reference lap while staying clean on any generated track
(density 0–0.9, object scale 1.0–1.8, width 0.85–1.0), any sensor-noise seed.

## Workflow changes (all pushed)

- Multi-track generations (`--tracks-per-generation K`): every candidate is
  evaluated on the reference track plus K−1 fresh random tracks.
- `--fitness-aggregation worst`: worst-of-K lexicographic objective — a
  candidate only scores well if it is clean and fast on ALL K tracks.
- `--vary-sensor-seed`: sensor-noise seed resampled every generation
  (common random numbers kept within a generation). The shipped 9.93 s B1
  was only 10/12 clean across sensor seeds; this fixed that failure mode.
- `--stop-after-stall N`: early stop when no promotion for N generations.
- Objective-winner export: `data/training/<stage>_objective_best.npz` always
  holds the multi-track objective winner (the lap-gated export used to
  silently discard robust-but-slightly-slower candidates).
- B1 genome extensions (append-only, old weight files stay valid):
  `clearance_speed_gain`, `steer_anticipation_gain`,
  `free_path_speed_gain` + `corridor_half_width_m`.
- B2 safety envelope (`min_clearance_m`, `clearance_speed_gain`,
  `steer_slow_gain`) is now evolved instead of hard-coded.
- Worker auto-detection is cgroup/affinity aware; remote notebook runner
  (`training/notebooks/remote_cma_training.ipynb`) runs training in the
  foreground for hosted environments (Kaggle-safe).

## Findings

1. **Speed and robustness trade off hard in the clearance-based policy.**
   Campaign 1–2 (mean fitness, fixed seed, narrow track distribution)
   reached 8.88 s but only 1–2/10 clean on hard random tracks.
2. **Worst-of-K selection is the robustness lever**: campaign 3 (worst-of-4)
   → 9.37 s at 4/12; campaign 4 (worst-of-6) → **9.57 s at 8/12 + 7/10**.
3. **Why the car never accelerates**: every speed term was a function of
   beam clearance, which is ~1 m everywhere on a 0.9 m lane. The champion
   cruises at a uniform ~2.8 m/s; the grip limit allows ~5 m/s at median
   curvature. Added a free-path-length speed term (how far the aimed
   corridor is open) to represent "there is room ahead".
4. **Large-sigma restarts destroy converged genomes** (campaign 5: 11.32 s).
   Refinement with sigma 0.06 instead (campaign 6: 9.55 s promoted, adopted
   `free_path_speed_gain=5.06`).
5. Physics: grip-limit lap ≈ 5.2 s, actuator-capped ≈ 6.7 s → 8 s is
   feasible; the remaining gap is driving skill, not physics.

## Champion so far

`algorithms/weights/policy_robust_b1.npz` (campaign 4 objective best):
**9.57 s reference lap, clean, 8/12 and 7/10 clean on hard held-out sweeps,
6/6 sensor seeds** on the 12-seed sweep.

## In flight

- Campaign 7: CMA-ES over the learned B2a beam scorer (48 dims, evolved
  safety envelope), warm-started from the champion B1 by DAgger imitation
  of its score surface, trained with worst-of-6.
