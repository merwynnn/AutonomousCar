# Experiment protocol and reporting

## 1. Principles

- Every experiment is reproducible from a config + seed.
- Compare distributions, not single runs.
- Report crashes/penalties, not only lap time.
- Separate controller quality from localization quality.
- Test on the PDF track and on generated tracks to detect overfitting.

## 2. Experiment definition

Experiments are run with the CLI and recorded as Markdown reports. A typical
A/B comparison:

```bash
uv run awake-sim compare --laps 5
uv run awake-sim race --controller pure_pursuit --laps 5
uv run awake-sim slam-bench --duration 20 --mcl
```

Repeat a run with different seeds when the result is noisy, and record the
command, seed and config in the report.

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
| `slam_rpe_m` | Relative pose error (when implemented) |
| `map_iou` | Occupancy agreement vs ground truth (when implemented) |
| `scan_ms_p95` | 95th percentile SLAM time per scan |
| `sim_realtime_factor` | Simulated time / wall time |

## 4. Storage

- Reports and plots: `docs/experiments/`.
- Screenshots: `docs/assets/`.
- Optional raw logs: `data/` (gitignored).
- Each report states the command, seed, code revision and config used.

## 5. Report template

`docs/experiments/EXP-<id>-<slug>.md`:

```markdown
# EXP-0001 — Pure pursuit vs wall-follow

## Question
...

## Setup
- track:
- controllers:
- localization:
- realism:
- seed:
- code revision / config:

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
uv run awake-sim compare --laps 5
```
```

## 6. Statistical hygiene

- Use at least 3 seeds per track for noisy comparisons.
- Report mean and standard deviation, or median and IQR.
- Do not compare runs with different realism/localization settings as if they were
  the same experiment.
- Flag runs that ended early (DNF) and include them in the crash rate.
- Keep the raw data; regenerate plots from it rather than editing images.

## 7. Running a comparison

- `awake-sim compare` runs wall-follow and pure pursuit back-to-back.
- `awake-sim race` runs one controller.
- `awake-sim slam-bench` reports SLAM ATE and scan time.
- Close the dashboard during long headless runs to avoid CPU contention.
- Record the wall time as well as simulated time when reporting performance.
