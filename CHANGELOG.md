# Changelog

All notable changes to this project are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- Initial repository, documentation scaffold and working rules (`AGENTS.md`).
- Architecture, requirements traceability, sensor realism, SLAM and strategy docs.
- Python 3.11 / `uv` project skeleton with FastAPI backend and React/Three.js dashboard.
- C++17 SLAM binding skeleton and test harness.
- Devlog and ADR structure for monitoring progress.
- M1 PyBullet 4WD Ackermann car, 16 cm pipe walls, 720-beam non-ideal lidar,
  three ultrasonic sensors, wall-follow and manual driving.
- Live dashboard with race metrics, sonars, steering, progress and speed trace.
- M1 5-lap acceptance run: 148.34 s total, 29.46–29.89 s laps, zero penalties.
- M2 racing line, time-optimal speed profile and pure pursuit controller.
- Pure pursuit 5-lap benchmark: 46.12 s total, 9.20 s best lap (GT upper bound).
- Dashboard controller switch and strategy benchmark panel.
- `awake-sim race` and `awake-sim compare` CLI commands.
- C++17 2D lidar SLAM front-end: per-beam deskew, occupancy grid, chamfer
  likelihood, coarse/fine correlative matching with motion prior.
- Live SLAM occupancy map, ghost pose/trail, ATE and scan-time diagnostics.
- `awake-sim slam-bench`: 2.89 cm mean ATE, 5.27 cm max, 14.2 ms/scan
  (low-noise odometry benchmark).
- Realistic drifting wheel/IMU odometry for SLAM evaluation.
- 400-particle MCL with ESS/spread diagnostics and runtime on/off/reset.
- Dashboard localization toggles (MCL, experimental loop closure) and debug
  layers for GT, odometry, SLAM, MCL, particles and map.
- `awake-sim slam-bench --mcl`: SLAM 22.6 cm vs MCL 22.0 cm mean ATE over 12 s;
  MCL tracks the map but cannot fix map drift.
- GT/odom/MCL localization modes; MCL drives pure pursuit with no GT.
- `localization-bench`: GT 20.12 s, MCL 20.40 s, odometry DNF on one lap.
- Map freeze/save and `map-and-race`: sensor-only MCL mapping, 20.81 s,
  sensor-only mapping; five-lap MCL race 5/5 in 136.00 s at 0.9 m/s
  (MCL ATE 22.4 cm mean, rank_keep 0.99). Global relocalization experimental/off.
- Procedural wall-side crates/poles as collidable, lidar-visible SLAM landmarks;
  included in the track hello message and 3D dashboard.
- Dashboard car-view selector: Actual / Controller / SLAM.
- Speed-limit slider now applies to pure pursuit as well as wall-follow.

### Changed
-

### Fixed
-
