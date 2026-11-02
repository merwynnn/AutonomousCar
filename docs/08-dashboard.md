# Dashboard

## Views

### Track

Live 3D view of the car, track surface, pipe walls, obstacles, lidar returns,
path trail and optional odometry trail. The viewport header shows run state,
controller, lap and timing. Manual driving uses Z/Q/S/D.

### Car studio

The **Car only** view hides track surface, walls, obstacles, lidar returns and
trails. It renders the vehicle, body parts, wheels and sensor mounts from the
same JSON payload used by the rest of the dashboard. Optional layers show sensor
coverage and labels.

The studio is intended for inspecting mass/geometry/sensor changes without the
track distracting from the car.

## Outliner and details

The left outliner exposes scenario, vehicle, structure parts, wheels, sensors,
track and race session. Clicking a node fills the right details panel with the
live configuration values. Sensor details include body-frame position, RPY,
height, range and model characteristics.

### Gap-solver debug

When the gap-follow controller is active, the viewport shows the chosen opening
as a 3D fan and a live clearance-vs-bearing chart. The chart shows the selected
bearing, opening edges, clearance and throttle. The **Gap solver** layer toggle
hides it.

### Tuning console and track generator

The **Tune** action opens a modal that edits the raw vehicle and sensor configs.
It supports `Ctrl+Z`/`Ctrl+Y` undo/redo, live apply and save-to-YAML. The steering
quick-tune strip exposes `rate_deg_s`, latency, max angle and deadband.

The **Test track** details panel generates random smooth tracks with seed, lane
width and object density. **Default** restores the reference test track.

![Random track generator](assets/dashboard-random-track.png)

![Tuning console](assets/dashboard-tuning.png)

![Gap solver debug](assets/dashboard-gap-debug.png)

## Toolbar

- Run, pause, step and reset.
- Track / Car only.
- Orbit, top and chase cameras; turntable in car studio.
- Auto/manual drive, demo controller and speed limit.
- Reload config.
- Layer toggles: lidar, trail, odometry, sensors, coverage, labels.

Keyboard shortcuts: `Space` run/pause, `R` reset, `C` view, `1`/`2`/`3` camera,
`T` turntable, `Z/Q/S/D` manual drive (WASD also works).

## Design language

The UI is a flat desktop engineering application, closer to an Unreal/CAE tool
than a consumer web dashboard:

- square panels, 1 px separators, dense type;
- neutral blue-grey chrome with a restrained amber accent;
- monospaced numeric values;
- no decorative gradients, glowing cards or oversized rounded containers.

## Protocol

REST: `/api/health`, `/api/status`, `/api/car`, `/api/car/mjcf`, `/api/track`, `/api/config`.

WebSocket `/ws/telemetry`:

- server: `hello`, `state`, `scan`, `config.result`, `track.result`;
- client: `run.start`, `run.pause`, `run.reset`, `run.step`,
  `run.set_speed`, `control.mode`, `control.controller`, `drive.command`,
  `config.reload`, `config.apply`, `track.generate`, `track.default`, `ping`.

The simulator runs MuJoCo and the dashboard displays `MUJOCO` in the title bar.

The backend throttles state to 30 Hz and forwards new lidar scans at the sensor
rate. It drops stale frames rather than delaying physics.
