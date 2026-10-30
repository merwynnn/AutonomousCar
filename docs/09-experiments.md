# Experiment protocol and reporting

## 1. Principles

- Every experiment is reproducible from a config + seed.
- Compare distributions, not single runs.
- Report crashes/penalties, not only lap time.
- Separate controller quality from localization quality.
- Test on the PDF track and on generated tracks to detect overfitting.

## 2. Experiment definition

An experiment is a set of runs defined by:

```yaml
name: pure_pursuit_vs_mpc
tracks:
  - test_track
  - generated: {count: 20, seed: 1000}
controllers:
  - pure_pursuit
  - stanley
  - mpc
localization: slam_mcl
realism: realistic
latency_ms: 35
seeds: [1, 2, 3]
laps: 5
```

The batch runner expands this matrix, runs each combination headless, and stores
the results.

## 3. Metrics

| Metric | Definition |
|---|---|
| `total_time_s` | Sum of 5 lap times |
| `best_lap_s` | Fastest lap |
| `lap_times_s` | Per-lap times |
| `completed_laps` | Laps validated by the race manager |
| `crash_count` | Deformation-level wall impacts |
| `penalty_points` | Competition-like penalty total |
| `interventions` | Repositioning events |
| `off_track_time_s` | Time outside track limits |
| `max_speed_mps` | Maximum body speed |
| `slam_ate_m` | Absolute trajectory error vs ground truth |
| `slam_rpe_m` | Relative pose error |
| `map_iou` | Occupancy agreement vs ground truth |
| `scan_ms_p95` | 95th percentile SLAM time per scan |
| `sim_realtime_factor` | Simulated time / wall time |

## 4. Storage

```text
data/experiments/<experiment_id>/
  manifest.json        config, seed, code version, environment
  metrics.json         per-run metrics
  runs/<run_id>.jsonl  state/events/commands
  runs/<run_id>.npz    scans, poses, map (optional)
  plots/*.png
```

A SQLite database (`data/experiments.db`) indexes experiments and runs for the
dashboard.

## 5. Report template

`docs/experiments/EXP-<id>-<slug>.md`:

```markdown
# EXP-0001 — Pure pursuit vs MPC

## Question
...

## Setup
- tracks:
- controllers:
- localization:
- realism:
- seeds:
- code version / config hash:

## Results
| Controller | Total time (s) | Best lap (s) | Crashes | Penalties |
...

## Plots
![lap times](assets/...png)

## Interpretation
...

## Decision
...

## Reproduction
```bash
uv run awake-sim batch configs/experiments/EXP-0001.yaml
```
```

## 6. Statistical hygiene

- Use at least 3 seeds per track for noisy comparisons.
- Report mean and standard deviation, or median and IQR.
- Do not compare runs with different realism/localization settings as if they were
  the same experiment.
- Flag runs that ended early (DNF) and include them in the crash rate.
- Keep the raw data; regenerate plots from it rather than editing images.

## 7. Batch performance

- Default worker count: 4 (configurable), leaving headroom for the dashboard.
- Each worker runs headless PyBullet; no rendering.
- A batch can be cancelled; completed runs are kept.
- Progress and partial results are visible in the Lab view.
