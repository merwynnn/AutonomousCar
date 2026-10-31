# Architecture

```text
Browser (React + Three.js)
    │ REST + WebSocket JSON
FastAPI dashboard (dashboard/server.py)
    │
DashboardSimulation (adapts simulator.Simulation)
    │
simulator/  (PyBullet, sensors, track, odometry, timing)
    │
algorithms/ (Observation -> Actuation, NumPy only)
```

## Boundaries

- `simulator/` owns physics and sensor simulation. It must not import dashboard
  code or React state.
- `algorithms/` owns driving policy. It imports only `algorithms.interface` and
  NumPy. It never imports `simulator.*`.
- `dashboard/` owns transport, operator commands and rendering. It may read
  ground truth for display but passes only sensor observations to controllers.
- The browser never runs physics.

## Data flow

1. `DashboardSimulation` advances the fixed-step simulator at the control rate.
2. The current controller returns `Actuation`.
3. The manager broadcasts `state`, `scan` and `hello` messages over WebSocket.
4. The browser renders messages. It never predicts or integrates physics.
5. Commands (`run.start`, `drive.command`, `config.reload`, etc.) travel back
   over the same WebSocket.

## Configuration

`scenario.yaml` references a vehicle and sensor YAML next to it. `Simulation`
can also receive `VehicleConfig` and `SensorsConfig` directly from Python.

The generated URDF, dashboard car studio and PyBullet collision geometry all use
the same `VehicleConfig`; `dashboard/spec.py` serializes it for the browser.
