# Project overview

## What we are building

A local 3D simulator and strategy lab for the AWAKE Challenge 2027 mini autonomous
vehicle. The simulator reproduces the competition’s essential physics, sensors,
track constraints and race rules, then lets the team compare strategies by lap
time, crashes and robustness.

The tool has four jobs:

1. **Strategy lab** — compare controllers, racing lines and speed profiles on the
   PDF-like test track and many generated variants.
2. **Sensor realism lab** — show how non-ideal lidar, ultrasonic, IMU and odometry
   affect control and SLAM; no perfect sensors by default.
3. **SLAM workbench** — a from-scratch 2D lidar SLAM pipeline in C++, with live
   maps, trajectories, covariance, loop closures and ground-truth evaluation.
4. **Real-car bridge** — hardware-agnostic controllers and a portable SLAM core,
   so validated pieces can move to the Raspberry Pi later.

## Why a simulator

The competition track is unknown until the day and the car does not exist yet. A
simulator lets the team:

- choose hardware (mass, motor, gear ratio, tire friction, lidar height) using
  sensitivity analysis rather than guesswork;
- test strategies across many track layouts instead of overfitting one circuit;
- measure the cost of sensor latency, noise and dropouts before race day;
- develop SLAM and controllers without risking the physical car;
- keep a reproducible record of every experiment.

## Success criteria

- Open a browser, watch a 1/10 car drive the PDF-like track with realistic lidar
  imperfections, and see live sensor and SLAM views.
- Compare at least four strategies over at least twenty generated tracks and get
  repeatable 5-lap results.
- Run 2D lidar SLAM at 10 Hz on desktop, with map and trajectory diagnostics and
  ground-truth ATE/RPE.
- Export a map, config and controller that can be reused on the real car.
- A new team member can install, run and understand the project from the docs.

## Non-goals for the MVP

- ROS 2/Gazebo integration.
- GPS or outdoor simulation.
- Camera-based perception (optional later).
- Certified digital-twin accuracy before the real car is measured.
- Full deformable-wall physics (penalty events are modelled instead).

## Guiding principles

- **Realism over convenience:** default sensors are imperfect and latency-aware.
- **No cheating:** controllers never see ground truth or future data.
- **Reproducibility:** fixed timestep, seeds and config hashes everywhere.
- **Visual first:** if it matters, it should be visible in the dashboard.
- **Portable core:** C++ SLAM and Python controllers must be able to leave the
  simulator.
- **Document as you go:** STATUS, devlog, ADRs and experiment reports are part of
  the product, not an afterthought.
