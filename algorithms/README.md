# Algorithms

This folder is the portable policy side of the car. It must not import
`simulator.*`, `dashboard.*`, a physics backend or any ground-truth state.

## The contract

`onboard/api.py` defines:

- `Observation` — timestamp, control `dt`, raw `SensorFrame` and fused
  `StateEstimate`.
- `SensorFrame` — lidar, ultrasonic, IMU, encoders, steering and an `extra`
  mapping for experiments.
- `StateEstimate` — estimated pose, speed, yaw rate, covariance, gyro bias and
  slip flag.
- `Actuation` — `throttle` in `-1..1`, `steering` in radians (positive left).
- `Controller` — `reset()` and `update(observation) -> Actuation`.
- `RobotAPI` — `reset()`, `step(command)`, `close()`; implemented by the
  simulator and by the Pi runtime.

`algorithms/interface.py` is a compatibility re-export. New controllers should
import `onboard.api` directly.

## Add a controller

Create `algorithms/my_controller.py`:

```python
from onboard.api import Actuation, Observation


class MyController:
    def reset(self) -> None:
        self.steering = 0.0

    def update(self, observation: Observation) -> Actuation:
        scan = observation.sensors.lidar
        if scan is None:
            return Actuation(throttle=0.0, steering=self.steering)
        # ... your policy ...
        return Actuation(throttle=0.4, steering=self.steering)
```

Run it from the repository root:

```bash
uv run python -m simulator.cli \
  --controller algorithms.my_controller:MyController \
  --laps 5
```

A factory is also accepted:

```bash
uv run python -m simulator.cli \
  --controller algorithms.my_controller:make_controller
```

## Included demo controllers

- `algorithms/gap_follow.py` — lidar-only furthest-gap steering.
- `algorithms/wall_follow.py` — lidar-only wall following.
- `algorithms/template.py` — safe stop placeholder.

The dashboard can drive with either demo controller. They are intentionally
simple local policies, not a racing stack.

## Rules

- Handle `observation.sensors.lidar is None` at startup and after dropped scans.
- Never use a scan whose `t_available` is greater than `observation.t`.
- `observation.state.pose` is intentionally drifting. There is no true pose.
- Keep controller state inside the controller; call `reset()` for every run.
- Prefer pure functions and NumPy arrays; keep the hot loop allocation-light.
- Do not import simulator internals. If you need a new sensor field, add it to
  `onboard.api.SensorFrame` on purpose and document it.

## Testing without the simulator

Construct `Observation`, `SensorFrame` and `LidarObservation` directly in a unit
test and call `update()`. The contract has no physics dependency, so a
controller can be tested on a laptop or the Pi with NumPy only.
