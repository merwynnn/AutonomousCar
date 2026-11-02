# ADR-001 — Portable onboard boundary

- Status: accepted
- Date: 2026-11-01
- Branch: `sim-core`

## Context

The simulator, dashboard and algorithms evolved together. The controller
contract lived in `algorithms/interface.py`, odometry was a simulator-only error
model fed by ground-truth speed/yaw rate, and configuration lived under
`simulator/configs/`. That made it hard to copy the driving stack to the
Raspberry Pi and impossible to test the real estimator chain in simulation.

## Decision

Introduce a portable `onboard/` package that owns:

- the `Observation` / `Actuation` contract;
- typed raw sensor frames;
- the fused `StateEstimate`;
- the `RobotAPI` protocol;
- the shared wheel + IMU estimator;
- the sensor -> estimator -> controller runtime;
- hardware-facing protocols and the low-level board link.

Dependency direction:

```text
algorithms  -> onboard.api
onboard     -> onboard.api (never simulator/dashboard/physics)
simulator   -> onboard.api
dashboard   -> simulator + onboard.api
```

The simulator implements `RobotAPI` through `simulator/bridge.py`. Simulator
sensor models produce the same raw frames as hardware drivers, and both feed the
same estimator.

Shared car, sensor and estimator configuration moves to root `configs/`.
Simulator-only scenarios move to `configs/sim/`.

## Consequences

- The Pi deployment is `onboard/ + algorithms/ + configs/`; MuJoCo and the
  dashboard remain host-only.
- Controllers can use raw data and fused state without importing simulator code.
- The estimator can be tested deterministically against simulated encoder/IMU
  streams before hardware exists.
- `algorithms/interface.py` is only a compatibility re-export.
- A future EKF can replace the estimator behind `StateEstimator` without
  changing controllers.
- The boundary test adds a small amount of duplication in test constants but
  catches accidental host imports immediately.
