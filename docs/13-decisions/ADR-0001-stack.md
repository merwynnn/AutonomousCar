# ADR-0001 — Python simulation core with a browser dashboard

- Status: accepted
- Date: 2026-10-30
- Deciders: project team

## Context

We need a physically credible 3D simulator, a strategy lab, realistic sensors, a
from-scratch SLAM pipeline, and a human-usable monitoring UI. The available
environment is WSL2 with an RTX 5060 (8 GB), 7.3 GB RAM and no ROS installation.
The real car will use Python on a Raspberry Pi.

Options considered:

1. Python + PyBullet + FastAPI + Three.js dashboard.
2. All-browser Three.js + Rapier.
3. ROS 2 + Gazebo.
4. Python desktop + Rerun/Foxglove.

## Decision

Use Python 3.11 + PyBullet for simulation, FastAPI/WebSocket for the backend, and
React/Three.js for the dashboard. C++ is used only for the SLAM hot path.

## Rationale

- Python has the best ecosystem for control, optimization, SLAM evaluation and
  experiments, and matches the real car’s Python stack.
- PyBullet provides a ready raycast vehicle, batched raycasts, deterministic
  fixed-step physics and headless operation.
- A browser dashboard is the most usable and portable monitoring interface; it
  works from Windows without WSL GUI issues.
- ROS 2/Gazebo is heavy for 7.3 GB RAM, uncertain on Ubuntu 26.04 and unnecessary
  for a strategy lab; an optional bridge can be added later.
- All-browser physics would weaken the algorithm ecosystem and SLAM tooling.

## Consequences

- We maintain a Python + TypeScript + C++ codebase (three languages).
- Python 3.11 is pinned because PyBullet 3.2.7 has no newer wheels.
- The browser does not run physics; the backend must stream telemetry efficiently.
- A ROS 2 bridge is a later optional adapter, not a core dependency.
