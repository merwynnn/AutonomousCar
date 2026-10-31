# Changelog

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
