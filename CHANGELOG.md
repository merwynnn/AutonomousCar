# Changelog

## 2026-10-31 — MuJoCo digital twin and AWAKE reference car

- Added a generated MuJoCo MJCF backend with contact wheels, steering hinges,
  wheel torque motors and position-servo steering.
- Made MuJoCo the default dashboard/CLI backend; PyBullet remains selectable.
- Added a shared `World` protocol so sensors and simulation run unchanged on
  either backend.
- Added the reference AWAKE 2027 car/sensor scenario:
  one 4WD motor, VESC-class limits, LD19-class lidar at 14 cm, four ultrasonic
  sensors, BNO085-class IMU and encoders.
- Added reference hardware BOM and calibration protocol docs.
- Added MuJoCo backend tests for model compilation, driving, sensor boundary and
  wall raycasts.

## 2026-10-31 — Car lab dashboard on the simulator core

- Added a FastAPI + WebSocket dashboard under `dashboard/`.
- Added a React/Three.js car studio that renders only the vehicle, with body
  parts, wheels and sensor mounts from the live configuration.
- Redesigned the UI as a flat desktop engineering application with an outliner,
  details inspector, toolbar, viewport HUD and status bar.
- Made the car definition drive the runtime URDF, physics and dashboard model.
- Added body structure parts, wheel width/mass/offset, sensor mount RPY and IMU
  mount to the configuration models.
- Added 3D lidar raycasts so lidar mount rotation affects measurements.
- Added lidar-only gap-follow and wall-follow demo controllers.
- Added code-defined car example and generated-URDF download endpoint.
- Added car-definition and URDF tests, dashboard docs and screenshots.
