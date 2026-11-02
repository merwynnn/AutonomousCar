# ADR-002 — MuJoCo-only physics core

## Status

Accepted — 2026-11-02.

## Context

The simulator originally carried two physics backends: PyBullet and MuJoCo.
Both were generated from the same `VehicleConfig`, but they interpreted the
actuation differently:

- PyBullet mapped throttle to a wheel-speed target;
- MuJoCo mapped throttle to a torque command with battery sag and speed
  derating.

Maintaining two backends doubled the actuator-model surface, the test surface
and the documentation while the project's actual goal is a credible MuJoCo
digital twin and a portable onboard runtime. The URDF generator and the
`/api/car/urdf` endpoint existed only to support the PyBullet path.

## Decision

- MuJoCo is the only physics world.
- `simulator/world.py` and `simulator/car_urdf.py` are removed.
- `pybullet` is removed from the project dependencies.
- `--backend`, `--gui`, `AWAKE_BACKEND` and backend fields in dashboard payloads
  are removed.
- `/api/car/urdf` is removed; `/api/car/mjcf` remains.
- The `World` protocol in `simulator/world_api.py` remains as the boundary
  between the simulator/sensor models and the MuJoCo implementation.
- `"pybullet"` stays in the import-boundary test's forbidden roots so it cannot
  be reintroduced accidentally.

## Consequences

- One command-to-actuation interpretation, one physics model, one test surface.
- The dashboard is the only visual front end. A native viewer, if needed later,
  should be an optional MuJoCo viewer tool, not a second physics backend.
- External tools that consumed the generated URDF must use the MJCF instead or
  generate their own model from `VehicleConfig`.
- The next actuator work can focus on one plant: a shared actuator model,
  low-level steering/speed loops and hardware calibration.
