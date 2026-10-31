# Algorithms

This folder is the portable policy side of the car. It must not import
`simulator.*`, PyBullet or any ground-truth state.

## The contract

`algorithms/interface.py` defines:

- `Observation` — timestamp, control `dt`, drifting odometry, odometry speed/yaw
  rate, one lidar scan (or `None` while a scan is in flight), ultrasonic reads.
- `Actuation` — `throttle` in `-1..1`, `steering` in radians (positive left).
- `Controller` — `reset()` and `update(observation) -> Actuation`.
- `RobotAPI` — `reset()`, `step(command)`, `close()`; implemented by the
  simulator and by a future hardware adapter.

## Add a controller

Create `algorithms/my_controller.py`:

```python
from algorithms.interface import Actuation, Observation


class MyController:
    def reset(self) -> None:
        self.steering = 0.0

    def update(self, observation: Observation) -> Actuation:
        scan = observation.lidar
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

- Handle `observation.lidar is None` at startup and after dropped scans.
- Never use a scan whose `t_available` is greater than `observation.t`.
- `observation.odometry` is intentionally drifting. There is no true pose.
- Keep controller state inside the controller; call `reset()` for every run.
- Prefer pure functions and NumPy arrays; keep the hot loop allocation-light.
- Do not import simulator internals. If you need a new sensor field, add it to
  `Observation` on purpose and document it.

## Testing without PyBullet

Construct `Observation` and `LidarObservation` directly in a unit test and call
`update()`. The interface has no PyBullet dependency, so the controller can be
tested on a laptop or the Pi with NumPy only.
