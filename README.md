# AWAKE 2027 — standalone simulator core

This branch (`sim-core`) contains **only the reusable simulator and the
algorithm boundary**. It deliberately has no dashboard, no SLAM, no MCL, no
occupancy grid, no racing line and no reference controller.

The two folders intended for transfer are:

```text
simulator/     PyBullet world, car, sensors, track, odometry, timing, CLI
algorithms/    sensor-only controller contract + your controllers
```

Everything else (`tests/`, this README, `pyproject.toml`) is development
scaffolding.

## 1. Architecture

```text
                 Observation (sensors + odometry only)
 algorithms/  <-------------------------------------------  simulator/
 Controller.update()                                      Simulation.step()
      |                                                        |
      +------------------> Actuation (throttle, steering) -----+
```

- `simulator/` owns physics, sensor models, latency, the track and lap timing.
- `algorithms/` owns the driving policy. It imports **only**
  `algorithms.interface` and NumPy.
- `Observation` has no true pose and no future scan. The only pose is drifting
  wheel/IMU odometry.
- `Simulation` implements the `RobotAPI` protocol. A real serial/CAN adapter can
  implement the same protocol and run the same controllers unchanged.

### Folder layout

```text
simulator/
  config.py                 pydantic config models + YAML loading
  track.py                  peanut track, walls, procedural on-track landmarks
  vehicle.py                VehicleState / Actuation
  world.py                  PyBullet world and 1/10 4WD Ackermann car
  odometry.py               drifting wheel/IMU odometry
  sensors/
    lidar.py                non-ideal 720-beam 2D lidar
    ultrasonic.py           three non-ideal ultrasonic sensors
  simulation.py             fixed-step loop, latency filtering, lap timing
  cli.py                    headless runner
  configs/                  scenario, vehicle and sensor YAML
  assets/                   1/10 car URDF
algorithms/
  interface.py              Observation, Actuation, Controller, RobotAPI
  template.py               safe stop placeholder
  README.md                 algorithm contract and example
```

## 2. Quick start

```bash
uv sync --extra dev

# Safe placeholder: the car stays still. Expect DNF; use Ctrl-C or --max-time.
uv run python -m simulator.cli --laps 1 --max-time 5
```

After you add a controller:

```bash
uv run python -m simulator.cli \
  --controller algorithms.my_controller:MyController \
  --laps 5
```

Useful options:

```bash
--scenario PATH        scenario YAML
--controller SPEC      module:ClassOrFactory
--laps N               override scenario laps
--seed N               override scenario seed
--realism ideal|realistic|harsh
--lidar-subrays 1|3    speed vs mixed-pixel fidelity
--gui                  open the PyBullet GUI
--realtime             pace the loop in wall-clock time
--max-time SECONDS     simulation-time limit
```

## 3. Add an algorithm

1. Create `algorithms/my_controller.py`.
2. Implement `reset()` and `update(observation)`.
3. Return an `Actuation`.
4. Run it with `--controller algorithms.my_controller:MyController`.

Minimal example:

```python
from algorithms.interface import Actuation, Observation


class MyController:
    def reset(self) -> None:
        self.steering = 0.0

    def update(self, observation: Observation) -> Actuation:
        scan = observation.lidar
        if scan is None:
            return Actuation(throttle=0.0, steering=self.steering)

        # Example only: steer toward the longest valid beam in the front sector.
        import numpy as np

        front = np.abs(scan.angles_rad) < np.deg2rad(60.0)
        valid = front & scan.valid & np.isfinite(scan.ranges_m)
        if not np.any(valid):
            return Actuation(throttle=0.0, steering=self.steering)

        ranges = np.where(valid, scan.ranges_m, -1.0)
        best = int(np.argmax(ranges))
        target = float(scan.angles_rad[best])
        self.steering += 0.25 * (target - self.steering)
        return Actuation(throttle=0.35, steering=self.steering)
```

If your controller needs constructor arguments, expose a factory:

```python
def make_controller():
    return MyController(gain=1.2)
```

Then use `--controller algorithms.my_controller:make_controller`.

### Contract rules

- Handle `observation.lidar is None`: the first scan may still be in flight.
- Use `lidar.t_available` if you keep your own history. Never use data whose
  `t_available` is in the future.
- `observation.odometry` is the only pose. It drifts on purpose.
- Units are SI: metres, seconds, radians, m/s, rad/s.
- Keep controllers deterministic for a fixed observation sequence.
- Do not import `simulator.*` from `algorithms/*`. That is the portability
  boundary.
- `throttle` is `-1..1`; `steering` is radians, positive left.

## 4. Raspberry Pi transfer

Copy only the two deployable folders:

```bash
scp -r simulator algorithms pi@raspberrypi.local:~/
```

On the Pi:

```bash
cd ~
python3 -m venv .venv
. .venv/bin/activate
pip install numpy pydantic pyyaml pybullet

# Headless smoke test
python -m simulator.cli --laps 1 --max-time 5 --lidar-subrays 1

# With a controller
python -m simulator.cli \
  --controller algorithms.my_controller:MyController \
  --laps 5 --lidar-subrays 1
```

Notes:

- The algorithm contract only needs **NumPy**. You can test controllers without
  PyBullet by feeding recorded `Observation` objects.
- PyBullet on a Pi is optional and needs a working OpenGL/EGL setup. Use
  `--lidar-subrays 1` and headless mode for slow boards.
- On the real car, implement `RobotAPI` with your serial/CAN adapter. Populate
  `Observation` from the real lidar, ultrasonic, IMU and encoders; call the same
  `Controller.update()`.
- Do not copy `tests/` or the development README unless you want them.

## 5. Configuration

`simulator/configs/scenario.yaml` references `vehicle.yaml` and `sensors.yaml`
relative to itself. Paths are resolved next to the scenario file, so the folder
is relocatable.

```yaml
vehicle: vehicle.yaml
sensors: sensors.yaml
seed: 42
realism: realistic
race:
  laps: 5
simulation:
  physics_hz: 240
  control_hz: 60
  max_time_s: 240
```

The simulator uses a fixed physics timestep, seeded sensor RNG streams and a
fixed control period. Rendering never affects results.

## 6. Tests and checks

```bash
uv run pytest
uv run ruff check simulator algorithms tests
uv run mypy simulator algorithms
```

The tests verify that `Observation` contains no ground truth, that lidar and
ultrasonic data respect their `t_available` timestamps, that reset is
deterministic, and that procedural obstacles remain inside the clear track
boundary.

## 7. What is intentionally missing

- No SLAM, MCL, occupancy map or pose graph.
- No dashboard, WebSocket or React code.
- No racing line, pure pursuit, wall-follow or gap-follow implementation.
- No real-car calibration.

Those live on `main`. This branch is the transferable core and the clean place
to add a new algorithm.
