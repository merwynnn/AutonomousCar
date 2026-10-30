# Documentation index

This directory is the project’s engineering memory. Start with `AGENTS.md` and
`STATUS.md`, then use the documents below.

| Document | Purpose |
|---|---|
| [`00-overview.md`](00-overview.md) | One-page project overview and goals |
| [`01-requirements.md`](01-requirements.md) | Traceable interpretation of the AWAKE 2027 specification |
| [`02-architecture.md`](02-architecture.md) | Components, data flow, interfaces and frames |
| [`03-simulation.md`](03-simulation.md) | Physics, vehicle model, timing and determinism |
| [`04-sensors.md`](04-sensors.md) | Non-ideal lidar, ultrasonic, IMU and odometry models |
| [`05-tracks.md`](05-tracks.md) | Test track, generator, editor and race rules |
| [`06-slam.md`](06-slam.md) | SLAM pipeline, math, performance and evaluation |
| [`07-strategies.md`](07-strategies.md) | Controllers, racing line, speed profile and experiments |
| [`08-dashboard.md`](08-dashboard.md) | UI views, WebSocket protocol and UX rules |
| [`09-experiments.md`](09-experiments.md) | Experiment protocol, metrics and reporting |
| [`10-calibration.md`](10-calibration.md) | Calibrating the simulator against the real car |
| [`11-roadmap.md`](11-roadmap.md) | Milestones and acceptance criteria |
| [`12-runbook.md`](12-runbook.md) | Install, run, test and troubleshoot |
| [`13-decisions/`](13-decisions/) | Architecture decision records (ADRs) |
| [`devlog/`](devlog/) | Dated work logs for monitoring progress |
| [`experiments/`](experiments/) | Experiment reports and results |
| [`assets/`](assets/) | Screenshots, diagrams and GIFs |

## Documentation rules

- Keep documents short enough to be read; link to code and configs instead of
  duplicating constants.
- Update the relevant document in the same commit as the code change.
- Record significant decisions as ADRs, not only in chat.
- Every experiment report must include the config, seed, metrics and conclusion.
- Screenshots go in `docs/assets/` with descriptive names.
