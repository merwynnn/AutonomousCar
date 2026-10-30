# Dashboard and monitoring

## 1. Purpose

The dashboard is the human window into the simulator. It must answer three
questions at a glance:

1. **What is the car doing?** — 3D pose, speed, steering, lidar, map.
2. **Why is it doing that?** — sensor data, localization, planned path, metrics.
3. **Which strategy is better?** — replay, experiment comparison, Pareto plots.

## 2. Views

### Race view

Minimal, large 3D scene for demos and driving:

- car and track;
- lap timer, current/best/total time;
- speed and steering;
- next-turn hint / racing line;
- penalties and collisions;
- start/pause/reset/step.

### Engineering view

Everything needed to debug perception, SLAM and control:

- lidar points and rays, raw vs deskewed, dropouts, intensity;
- ultrasonic cones and hit points;
- odometry, IMU drift and estimated pose;
- occupancy grid, likelihood field, matched scan;
- GT/odom/SLAM trajectories, covariance, particles;
- pose graph nodes/edges and loop closures;
- planned path, racing line, speed heatmap;
- charts: speed, steering, yaw, acceleration, lap/sector times, latency,
  sensor rates, SLAM timing/ATE;
- sensor health panel with warnings (e.g., lidar above wall height).

### Lab view

- experiment table with filters and sorting;
- multi-run comparison and overlay plots;
- CSV/JSON export;
- batch launcher and progress;
- live tuning sliders;
- track generator controls and editor;
- replay with timeline scrubbing.

### Project view

- renders `STATUS.md` (current milestone, what works, next tasks);
- links to docs, devlog and experiment reports;
- latest screenshots and known issues;
- build/version information.

## 3. 3D scene layers

| Layer | Rendering |
|---|---|
| Track floor | Dark plane with track boundaries |
| Walls | Instanced pipe cylinders, 16 cm high |
| Obstacles | Boxes/cones/barriers |
| Car | Simple 1/10 car mesh with wheels; orientation axes |
| Lidar | `THREE.Points` for hits + optional ray lines |
| Ultrasonic | Cone meshes + hit markers |
| Occupancy map | DataTexture on the floor, log-odds colour map |
| Trajectories | Lines: GT, odometry, SLAM, planned path |
| Racing line | Thick line with speed colour |
| Covariance | Ellipse at the estimated pose |
| Particles | Small points for MCL |
| Collisions | Flash/marker at contact point |
| Start line | Checkered strip |

Camera modes: chase, orbit, top-down, first-person, lidar/map view. Keyboard
shortcuts: WASD manual driving, space pause, R reset, C camera, L lidar, M map.

## 4. Controls

- Start / pause / step / reset.
- Simulation speed (real time, slow motion, fast).
- Scenario and track selection.
- Controller and localization mode.
- Realism preset (`ideal`, `realistic`, `harsh`).
- Seed.
- Live tuning: controller gains, speed limit, lookahead, friction, latency,
  noise multipliers.
- Record/replay controls.
- Batch launcher.

## 5. WebSocket protocol

### Transport

- REST for CRUD/config/results.
- WebSocket `/ws/telemetry` for run state, commands and telemetry.
- JSON for control/state; binary frames for arrays.

### Binary frame layout

```text
byte 0      : message type (uint8)
byte 1      : flags (uint8)
bytes 2-3   : header length (uint16, little endian)
bytes 4..   : UTF-8 JSON header
remaining   : payload (Float32/Uint8 arrays)
```

Types:

| ID | Type | Payload |
|---|---|---|
| 1 | `lidar` | angle[], range[], intensity[], timestamp[] |
| 2 | `map_delta` | cell indices + log-odds values |
| 3 | `trajectory` | x[], y[] per path |
| 4 | `particles` | x[], y[], weight[] |
| 5 | `scan_match` | correspondences or score grid |

JSON messages:

- `hello`
- `state`
- `metrics`
- `event`
- `experiment`
- `error`

### Throttling

- State: 30 Hz.
- Lidar: 10 Hz (or every other scan in slow/busy mode).
- Map: deltas at 1–2 Hz; full map on request.
- Trajectories: 10 Hz.
- Charts: 10 Hz.
- The server drops frames rather than delaying physics.

## 6. UX rules

- The default view is Race; engineering detail is one click away.
- Units are always labelled (m/s, °, s, m).
- Red = danger/penalty, amber = warning, green = nominal.
- No hidden mode: the active realism preset and localization mode are always
  visible.
- Warnings are actionable (e.g., “lidar at 0.18 m is above the 0.16 m wall”).
- The UI remains usable at 1080p and with a single screen.
- Connection/FPS/sim-speed indicators are always visible.
- The app must survive a backend restart without a full page reload.

## 7. Screenshots and Telegram

- Playwright captures the dashboard at milestones.
- Screenshots are stored in `docs/assets/` with descriptive names.
- Telegram receives a screenshot and a short status at each milestone.
- `STATUS.md` links the latest screenshot and devlog entry.

## 8. Implementation notes

- React + TypeScript + Vite; Three.js for the scene; uPlot for charts.
- Keep Three.js scene code in `web/src/scene/`, UI panels in `web/src/panels/`.
- Simulation state is received through a typed WebSocket client; no physics in
  the browser.
- Use `requestAnimationFrame` for rendering and throttle React state updates.
- Dispose of Three.js geometries/materials on unmount.
- Test with Playwright smoke/screenshot tests.
