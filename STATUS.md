# Project status

> Last updated: 2026-10-30
> Current milestone: **M1 complete — M2 next**

![M1 PyBullet dashboard](docs/assets/2026-10-30-m1-pybullet.png)

## M1 acceptance result

Headless 5-lap run, `realistic` sensors, 720-beam lidar, 3 ultrasonic sensors,
wall-follow controller:

| Metric | Result |
|---|---|
| Laps completed | 5 / 5 |
| Lap times | 29.52, 29.89, 29.68, 29.46, 29.79 s |
| Total time | **148.34 s** |
| Penalties | 0 |
| Wall time for the run | 94.8 s |

The wall-follow controller is deliberately conservative. M2 replaces it with a
racing line and time-optimal speed profile for real lap-time optimization.

## What works today

- PyBullet headless world with a custom 1/10 4WD Ackermann URDF.
- Floor and 16 cm pipe-wall collision geometry (768 boxes), fixed 240 Hz physics.
- Wheel friction, steering servo model, 4WD velocity control.
- 720-beam raycast lidar with:
  - range/angle noise, bias, quantization;
  - per-beam timestamps and motion distortion;
  - dropouts vs incidence/range, dead beams, dropped scans;
  - intensity, wall-height occlusion, self-filtering, latency.
- Three ultrasonic sensors with cone sampling, blind zone, noise, latency,
  cross-talk/specular dropouts and real ranges.
- Latency-aware control: commands are held while a scan is in flight; the
  controller never sees a future scan.
- Lidar wall-follow + manual keyboard driving (W/S throttle, A/D steering).
- Race manager: start-line arming, 5-lap timing, progress/lateral offset.
- Dashboard: live 3D track/car/lidar/trail, race metrics, sonars, steering,
  speed chart, auto/manual modes, target speed, camera modes.
- 20 Python tests + C++ CTest green; `ruff`, `mypy` and the frontend build pass.

## How to run

```bash
# backend (WSL)
uv run uvicorn awake_sim.server.app:app --port 8000

# dashboard (second terminal)
cd web
npm run dev -- --port 5173
```

Open `http://localhost:5173`. The car auto-starts in wall-follow mode; press
Pause/Reset or switch to Manual.

## Next tasks (M2)

1. Centreline extraction, minimum-curvature racing line and time-optimal speed
   profile.
2. Pure pursuit, Stanley and MPC controllers; compare with wall-follow.
3. Seeded track generator with obstacles and a simple track editor.
4. Batch experiment runner, SQLite/CSV results, leaderboard and Pareto plots.
5. Full replay/scrubbing and live parameter tuning.

## Known issues

- Wall-follow is slow (~0.9 m/s average) and conservative on the waist corners.
- Live mode uses one lidar sub-ray per beam; batch/high-fidelity mode restores 3.
- SLAM is still the C++ binding skeleton; algorithms start in M3.
- No replay yet; runs are not persisted to disk.
- The simulation runs at roughly 0.85–1.0× real time under the live dashboard.
- Vite uses polling because inotify is unreliable on `/mnt/c` under WSL2.

## Monitoring

- Devlog: [`docs/devlog/2026-10-30.md`](docs/devlog/2026-10-30.md)
- Architecture: [`docs/02-architecture.md`](docs/02-architecture.md)
- Sensor model: [`docs/04-sensors.md`](docs/04-sensors.md)
- Roadmap: [`docs/11-roadmap.md`](docs/11-roadmap.md)
- Screenshots: [`docs/assets/`](docs/assets/)
