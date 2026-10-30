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

### Changed
-

### Fixed
-
