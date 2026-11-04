# AWAKE 2027 — simulator core & car lab

A physically credible standalone simulator and browser car lab for the AWAKE
Challenge 2027 mini autonomous vehicle. The physics core is a MuJoCo digital
twin driven by one vehicle definition, one sensor-only controller boundary and
one dashboard.

![Car studio](docs/assets/dashboard-car-studio.png)

Reference hardware and calibration:
[hardware BOM](docs/10-hardware.md) · [calibration protocol](docs/11-calibration.md) ·
[car definition](docs/09-car-definition.md) · [dashboard](docs/08-dashboard.md).

The core remains deliberately small: **no SLAM, MCL, occupancy grid, racing line
or telemetry cloud**. The dashboard is a local renderer and operator console for
the simulator.

## 1. Architecture

```text
Browser (React + Three.js)
    │  REST + WebSocket (JSON)
FastAPI dashboard (dashboard/, host only)
    │
Simulator (simulator/, host only)
    ├── generated MuJoCo MJCF from the vehicle config
    ├── MuJoCo digital twin, track and raw sensor measurement models
    └── SimRobotAPI adapter
Portable onboard runtime (onboard/, deployable to the Pi)
    ├── Observation / SensorFrame / StateEstimate / RobotAPI contract
    ├── shared wheel + IMU estimator
    ├── SensorRobot / ControllerRuntime loop
    └── hardware drivers and JSON-lines protocol
Algorithms (algorithms/)
    └── Observation -> Actuation controllers (onboard.api + NumPy only)
```

- `onboard/` is the code that can run unchanged on the Raspberry Pi. It never
  imports `simulator.*` or `dashboard.*`.
- `simulator/` owns physics, simulated sensor measurements, latency, the track
  and lap timing. It implements the portable `RobotAPI` through
  `simulator/bridge.py`.
- `dashboard/` owns the browser UI, telemetry transport and car inspection view.
- `algorithms/` owns the driving policy and imports **only** `onboard.api` and
  NumPy.
- `Observation` has no true pose and no future scan. Controllers get raw
  timestamped data in `observation.sensors` and the shared estimator output in
  `observation.state`. The only pose is the drifting estimate.
- The dashboard may show ground truth for debugging; controllers never receive it.
- A real serial/CAN adapter implements `RobotAPI` and runs the same controllers.

### Folder layout

```text
onboard/                    portable runtime for the Pi
  api.py                    Observation, SensorFrame, StateEstimate, RobotAPI
  actuation.py              shared actuator calibration and command shaping
  config.py                 estimator tuning + shared geometry/hardware loading
  estimator/                wheel + IMU estimator used by sim and hardware
  runtime.py                SensorRobot / ControllerRuntime
  drivers/                  JSON-lines protocol, mock board and hardware protocols
simulator/                  host-only physics and sensor core
  config.py                 pydantic config models + YAML loading
  mjcf.py                   generates the MuJoCo MJCF digital twin
  bridge.py                 SimRobotAPI implementation
  track.py                  peanut track, walls and obstacles
  vehicle.py                VehicleState / Actuation
  mujoco_world.py           MuJoCo world and raycasts
  transforms.py             body/sensor rotation helpers
  sensors/                  lidar, ultrasonic, encoder, IMU and steering models
  simulation.py             fixed-step loop, latency filtering, lap timing
  cli.py                    headless runner
configs/                    shared configuration
  vehicle*.yaml             reference/generic car definitions
  sensors*.yaml             sensor mounts and non-ideal characteristics
  estimator.yaml            shared estimator tuning
  hardware.yaml             servo/VESC/board calibration shared with the Pi
  sim/                      simulator-only scenarios
algorithms/
  interface.py              compatibility re-export of onboard.api
  debug.py                  controller-neutral debug snapshots
  registry.py               dashboard-selectable controllers
  gap_follow.py             lidar-only demo controller
  wall_follow.py            lidar-only demo controller
  learned_gap.py            CMA-ES-evolved gap controller
  weights/policy.npz        current best evolved policy
  template.py               safe stop placeholder
training/                   host-only CMA-ES rollout search and reporting
dashboard/
  server.py                 FastAPI REST + WebSocket adapter
  spec.py                   car, sensor and track JSON payloads
  web/                      React/Three.js dashboard
examples/
  custom_car.py             fully code-defined car example
docs/
 10-hardware.md             reference BOM and wiring
 11-calibration.md          measurement and validation protocol
tests/
```

## 2. Quick start

```bash
uv sync --extra dev

# optional: real serial transport
uv sync --extra hardware

# backend (terminal 1)
uv run python -m dashboard.server

# frontend (terminal 2)
cd dashboard/web
npm install
npm run dev
```

On Windows, double-click `run.bat` to start both servers in WSL and open the
dashboard.

Open the Vite URL (usually `http://localhost:5173`). The API runs on
`http://localhost:8000`. A production build is served by the backend after
`npm run build`. The dashboard defaults to the reference car in
`configs/sim/awake_2027.yaml` and the MuJoCo physics world.

Hardware-free self-test of the portable robot path:

```bash
uv run python -m onboard.tools.selftest --profile mock
```

Headless simulator:

```bash
uv run python -m simulator.cli --laps 1 --max-time 5
uv run python -m simulator.cli \
  --controller algorithms.gap_follow:FurthestGapController \
  --laps 1 --realism ideal --lidar-subrays 1
```

Switch the scenario with `AWAKE_SCENARIO=/path/to/scenario.yaml`.

The dashboard also lists **Learned gap (CMA-ES)**, which loads
`algorithms/weights/policy.npz`. To reproduce or improve it:

```bash
uv sync --extra dev --extra train

# warm-start B2 from the committed B1 policy
uv run python -m training.warmstart --laps 8 --iterations 0 --speed-cap 2.7 \
  --out data/training/b2a_init.npz

# preflight (lap, contacts, rough ETA), then the monitored search
uv run python -m training.cma_opt --stage b2a --preflight --workers 6 \
  --popsize 16 --init data/training/b2a_init.npz
uv run python -m training.cma_opt --stage b2a --generations 60 --popsize 16 \
  --sigma0 0.18 --workers 6 --init data/training/b2a_init.npz
uv run python -m training.watch --stage b2a

# randomised tracks, object density and object sizes
uv run python -m training.cma_opt --stage b2a --generations 40 --popsize 12 \
  --sigma0 0.08 --workers 6 --init algorithms/weights/policy.npz \
  --random-track --reference-track-prob 0.25 \
  --object-density-range 0.15 0.75 --object-scale-range 1.10 1.60 \
  --track-width-range 0.85 1.00 --out data/training/policy_random.npz
```

Every promoted candidate is still re-validated on the reference track before
it can be exported, so random-track training cannot silently replace the
reference lap with a lucky random circuit.

Training is host-only. Logs, checkpoints and status files go to the
git-ignored `data/training/`; only the chosen weights are committed. The
dashboard's **Reload algo** action picks up a newly exported `policy.npz`
without restarting the server.

## 3. The car is data

There is **one** vehicle definition. It generates the MuJoCo MJCF that the
physics world loads and the geometry that the dashboard draws. There is no
second hand-written car model.

`configs/vehicle_awake2027.yaml` defines the reference competition car;
`configs/vehicle.yaml` remains the generic example. `configs/hardware.yaml`
defines the shared servo/VESC/board calibration used by the simulator and the Pi.
The vehicle files define:

- mass, centre of mass and competition limits;
- chassis `structure` parts (box / cylinder / sphere, visual and/or collidable);
- wheelbase, track width, wheel radius, width, mass and vertical offset;
- drive layout, motor, battery, tires, suspension, aero and steering limits.

Example body part:

```yaml
structure:
  - name: main_body
    shape: box
    position_m: [0.0, 0.0, 0.005]
    size_m: [0.38, 0.18, 0.055]
    color: "#a53235"
    collision: true
```

Positions are in the vehicle body frame: **x forward, y left, z up**, origin at
the chassis centre. For a cylinder, `size_m` is `[diameter, diameter, height]`;
for a sphere it is `[diameter, diameter, diameter]`.

Sensor positions and rotations live in `configs/sensors.yaml` and use
the same body frame:

```yaml
lidar:
  mount:
    position_m: [0.10, 0.0, 0.05]
    rpy_deg: [0.0, 0.0, 0.0]
  beams: 720
  max_range_m: 12.0
  noise:
    range_sigma_m: 0.012
```

Other sensors follow the same pattern. IMU, encoder and lidar/ultrasonic
characteristics are all parsed and used by the sensor models.

### Build a car in code

```python
from simulator.config import BodyPart, load_sensors, load_vehicle
from simulator.simulation import Simulation

vehicle = load_vehicle("configs/vehicle.yaml")
sensors = load_sensors("configs/sensors.yaml")

vehicle.mass_kg = 2.35
vehicle.wheelbase_m = 0.27
vehicle.tires.friction_rear = 1.15
vehicle.structure.append(
    BodyPart(
        name="sensor_pod",
        shape="cylinder",
        position_m=(0.0, 0.0, 0.07),
        size_m=(0.04, 0.04, 0.02),
        color="#d9dde1",
        collision=False,
    )
)
sensors.lidar.beams = 540
sensors.lidar.mount.position_m = (0.105, 0.0, 0.06)

simulation = Simulation(vehicle=vehicle, sensors=sensors, lidar_subrays=1)
```

See [`examples/custom_car.py`](examples/custom_car.py) for a runnable example:

```bash
uv run python -m examples.custom_car
```

Download the generated physics model from the running dashboard:

```bash
curl -o car.xml http://localhost:8000/api/car/mjcf
```

## 4. Dashboard

### Track view

Live MuJoCo scene with the car, walls, obstacles, lidar returns, path trail and
odometry trail. Drive manually with `Z/Q/S/D` or use the two lidar-only demo
controllers.

### Car studio

Click **Car only** or press `C`. The studio renders the vehicle in isolation:
no track surface, walls, obstacles, lidar returns or trails. It shows the body
parts, wheels, sensor mounts and optional sensor coverage directly from the same
configuration that drives physics.

Useful controls:

- `Space` — run / pause
- `R` — reset
- `C` — track / car studio
- `1` / `2` / `3` — orbit / top / chase camera
- `T` — car studio turntable
- `Z/Q/S/D` — manual drive (WASD also works)

### Tuning console

Click **Tune** in the toolbar or details panel. The modal edits the raw vehicle
and sensor configs, including:

- mass, dimensions, structure parts, wheels, motor, battery, tires, suspension,
  aero and steering;
- all lidar/ultrasonic/IMU/encoder fields and body-frame mounts;
- steering `rate_deg_s` (the servo slew-rate limit), latency, deadband and max
  angle in the quick-tune strip.

Use **Apply live** to rebuild without touching disk, or **Apply & save YAML** to
persist the values in `vehicle_awake2027.yaml` / `sensors_awake2027.yaml`.
`Ctrl+Z` / `Ctrl+Y` undo and redo draft edits; `Esc` closes the console.

### Random track generator

Click **Test track** (or **Random track**) in the outliner. The details panel
exposes a seed, lane width and **object density** (0 = empty, 1 = maximum
crates/poles). Generated tracks stay inside the 15 × 8 m arena. The **Default**
button always restores the reference test track.

### Gap-solver debug

When the gap-follow controller is active, the debug dock shows a range profile,
a score breakdown and the scored candidate beams. The viewport adds a restrained
3D opening fan, the selected/clamped bearings and up to three well-separated
candidate labels. Use the **Debug** layer toggle to hide or show the overlay.

Controllers can expose the same kind of view by returning a `DebugFrame` from
`algorithms/debug.py`; see [docs/08-dashboard.md](docs/08-dashboard.md).

After editing YAML, click **Reload config**. The dashboard reloads the scenario,
vehicle and sensor files and rebuilds the MJCF, sensors, estimator and
simulation. It pauses the run and pushes a fresh `hello` message to the browser.
It does not re-import Python modules.

After editing a file under `algorithms/`, click **Reload algo**. The backend
re-imports the algorithm modules and registry, rebuilds controller instances and
leaves the simulation, track, lap timing and current vehicle position untouched.
The new controller takes over on the next control tick. This is the quick path
for iterating on a controller without restarting `dashboard.server`. It reloads
the portable `algorithms/` package only; changes under `onboard/` or
`simulator/` still require a server restart.

If an algorithm fails to import or raises during an update, the run pauses and a
persistent red controller-error strip shows the exception. The server keeps
running: fix the file, click **Reload algo**, then resume. The previous
controller instances stay alive after a failed reload.

### REST and WebSocket

- `GET /api/health`, `/api/status`, `/api/car`, `/api/car/mjcf`, `/api/track`, `/api/config`
- `WS /ws/telemetry` — hello, state and scan messages; commands include
  `run.start`, `run.pause`, `run.reset`, `run.step`, `run.set_speed`,
  `control.mode`, `control.controller`, `drive.command`, `config.reload`,
  `config.apply`, `controller.reload`, `track.generate`, `track.default`.

## 5. Add an algorithm

1. Create `algorithms/my_controller.py`.
2. Implement `reset()` and `update(observation, speed_limit)` (the second
   argument may be optional).
3. Return an `Actuation`.
4. Add it to `algorithms/registry.py` so the dashboard lists it.
5. Optionally add `debug_frame()` returning an `algorithms.debug.DebugFrame`.

A controller can still be run headlessly with
`--controller algorithms.my_controller:MyController`.

Minimal example:

```python
import numpy as np
from onboard.api import Actuation, Observation


class MyController:
    def reset(self) -> None:
        self.steering = 0.0

    def update(
        self,
        observation: Observation,
        speed_limit: float | None = None,
    ) -> Actuation:
        scan = observation.sensors.lidar
        if scan is None:
            return Actuation(throttle=0.0, steering=self.steering)

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

The controller receives both layers:

```text
observation.sensors   raw lidar / ultrasonic / IMU / encoders / steering
observation.state     shared estimator: pose, speed, yaw rate, covariance,
                      gyro bias and slip flag
```

Compatibility properties (`observation.lidar`, `observation.odometry`,
`observation.speed_mps`, ...) still work for older controllers.

Contract rules:

- Handle `observation.sensors.lidar is None`: the first scan may still be in
  flight.
- Only use data whose `t_available` is in the past.
- `observation.state.pose` is the only pose. It drifts on purpose.
- Units are SI: metres, seconds, radians, m/s, rad/s.
- Keep controllers deterministic for a fixed observation sequence.
- Do not import `simulator.*` from `algorithms/*`.
- `throttle` is `-1..1`; `steering` is radians, positive left.

## 6. Sensors

- Lidar: 720 beams by default, range/angle noise, quantization, per-beam
  capture times, motion distortion, divergence/mixed pixels, incidence dropout,
  dead beams, dropped scans, intensity, wall-height occlusion, mount RPY,
  latency/jitter and calibration offset. Rays are traced in 3D so a tilted mount
  changes what is measured. `Observation.lidar_current` re-expresses the latest
  scan in the current lidar frame using the estimated odometry; the raw scan
  remains in `Observation.sensors.lidar`.
- Ultrasonic: cone integration, blind zone, max range, range noise, low update
  rate, latency, cross-talk, specular dropouts and missed thin objects.
- Encoders: rear-wheel tick counts at 50 Hz, wheel-radius and track-width
  calibration errors, timestamp jitter and dropouts. Wheel spin from the
  physics contact model includes slip.
- IMU: gyro bias and random walk, scale error, noise, cross-axis coupling,
  mounting misalignment, saturation and latency.
- Steering: measured linkage angle with noise, quantization and latency.
- Estimator: the same `onboard.estimator.WheelImuEstimator` runs in simulation
  and on the Pi. It fuses rear-wheel speed, gyro yaw rate and steering, performs
  a zero-velocity bias update and raises a slip flag when wheel and gyro
  disagree.
- Realism presets: `ideal`, `realistic`, `harsh`; `realistic` is the default.

## 7. Tests and checks

```bash
uv run pytest
uv run ruff check simulator algorithms onboard dashboard tests examples
uv run mypy simulator algorithms onboard dashboard
cd dashboard/web && npm run build
```

The tests cover the sensor-only boundary, lidar/ultrasonic latency, deterministic
reset, encoder/IMU estimator behaviour, configuration limits, generated MJCF
geometry and dashboard payloads.

## 8. Raspberry Pi transfer

The Pi runtime is `onboard/` plus `algorithms/` and the shared config files.
`simulator/`, `dashboard/` and MuJoCo stay on the host.

```bash
scp -r onboard algorithms configs pi@raspberrypi.local:~/
```

On the Pi, install only the portable dependencies (`pip install numpy pyyaml`)
and create a `SensorRobot`:

```python
from onboard.config import load_onboard_config
from onboard.drivers.serial_link import SerialActuatorSink, SerialSensorSource
from onboard.estimator import WheelImuEstimator
from onboard.runtime import ControllerRuntime, SensorRobot
from algorithms.gap_follow import FurthestGapController

config = load_onboard_config(
    "configs/vehicle_awake2027.yaml",
    "configs/sensors_awake2027.yaml",
    "configs/estimator.yaml",
)
estimator = WheelImuEstimator(
    config.tuning,
    wheel_radius_m=config.wheel_radius_m,
    track_width_m=config.track_width_m,
    wheelbase_m=config.wheelbase_m,
    ticks_per_rev=config.ticks_per_rev,
)
# source/sink are your board-specific SensorSource / ActuatorSink instances,
# for example SerialSensorSource(transport, lidar_driver) and SerialActuatorSink(transport).
robot = SensorRobot(source, sink, estimator, control_dt=1.0 / 60.0)
runtime = ControllerRuntime(robot, FurthestGapController())
runtime.run(steps=10_000)
```

`SerialSensorSource` / `SerialActuatorSink` use the JSON-lines protocol in
`onboard/drivers/serial_link.py`; replace them with your board-specific drivers
if you use a different link. The portable contract needs NumPy and PyYAML only.

## 9. Configuration

Shared car, sensor and estimator configuration lives in the root `configs/`.
Simulator-only scenarios live in `configs/sim/` and reference the shared files:

```yaml
# configs/sim/scenario.yaml
vehicle: ../vehicle.yaml
sensors: ../sensors.yaml
seed: 42
realism: realistic
race:
  laps: 5
simulation:
  physics_hz: 240
  control_hz: 60
  max_time_s: 240
```

Estimator tuning lives in `configs/estimator.yaml`; geometry and encoder
resolution come from the shared vehicle and sensor YAML, so there is still only
one car definition.

The simulator uses a fixed physics timestep, seeded sensor RNG streams and a
fixed control period. Rendering never affects results.
