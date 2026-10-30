# Simulation model

## 1. Physics engine

PyBullet 3.2.7 provides:

- rigid-body dynamics with fixed timestep;
- a raycast vehicle with suspension, wheels and tire friction;
- batched raycasts for lidar and ultrasonic models;
- collision detection against track walls and obstacles;
- headless operation for batch experiments.

The simulator targets **credible comparative dynamics**, not a certified digital
twin. Once the real car exists, measured parameters replace the generic defaults;
see `docs/10-calibration.md`.

## 2. World

- Track area: 15 × 8 m, flat floor, dark mat material.
- Walls: 16 cm high grey VMC pipes (160 mm diameter, radius ≈8 cm), arranged along the track
  boundary; continuous pipe chains with small irregularities.
- Obstacles: boxes, cones and pipe barriers added by the generator.
- Gravity: −9.81 m/s²; indoor, no wind.
- Floor friction: configurable 0.8–1.0 with spatial variation and seams.
- Lighting: neutral indoor light; only relevant if the optional camera is added.

## 3. Vehicle model

Default preset: generic 1/10 4WD Ackermann RC car. All values are configurable in
`configs/vehicles/generic_1_10_4wd.yaml` and validated against the competition
limits.

| Parameter | Default | Notes |
|---|---|---|
| Mass | 2.0 kg | Includes battery and electronics |
| Length / width / height | 0.40 / 0.19 / 0.16 m | Competition max 0.45 / 0.20 / 0.20 |
| Wheelbase | 0.26 m | Rear axle to front axle |
| Track width | 0.18 m | Left-right wheel centres |
| Wheel radius | 0.035 m | 1/10 touring car |
| Ground clearance | 0.008 m | Competition minimum 0.005 |
| Drive | 4WD | 2WD configurable |
| Max steering | 25° | Servo limit |
| Steering rate | 300°/s | Rate-limited |
| Motor | ~150 W equivalent | Torque curve configurable |
| Battery | 2S LiPo, 7.4 V | Voltage sag model |
| Top speed | ≈6 m/s | Depends on gearing and track |
| Tire friction | μ ≈ 1.0 | Tunable front/rear |

### 3.1 Suspension and tires

The PyBullet raycast vehicle uses:

- independent spring/damper suspension per wheel;
- longitudinal/lateral tire friction;
- rolling resistance and aerodynamic drag;
- Ackermann steering geometry;
- 4WD torque split (open differential approximation);
- wheel slip derived from the contact state.

The model exposes wheel slip and contact state to the odometry/encoder sensor, so
odometry drifts realistically under acceleration and cornering.

### 3.2 Motor, ESC and battery

- Motor torque curve vs RPM, with a current limit.
- ESC command maps to engine force/braking with a short lag.
- Battery voltage sag `V = V_oc - I·R_int`, with temperature derating.
- Regenerative braking is optional; default is friction braking only.
- Thermal derating is optional for long runs.

### 3.3 Steering

- Commanded angle is rate-limited, quantized and delayed.
- Deadband and backlash are configurable.
- The steering sensor measures the actual angle with noise and quantization.

### 3.4 Collisions and penalties

- Wall pipes are rigid bodies; the car collides with them.
- A contact impulse above `deformation_impulse` triggers a “wall deformation”
  penalty event. Light grazing below the threshold is tolerated, matching the
  competition rule.
- A stuck/reposition command triggers an intervention penalty.
- Optional visual displacement of the hit pipe for feedback; full deformable
  physics is out of scope.

## 4. Timing

| Loop | Rate | Notes |
|---|---|---|
| Physics | 240 Hz | Fixed `dt = 1/240 s` |
| Control | 50 Hz | Reads latency-filtered sensors |
| Lidar | 10 Hz | 720 beams default |
| Ultrasonic | 20 Hz | 3 sensors |
| IMU | 200 Hz | Raw gyro/accel |
| Encoders | 50 Hz | Per-wheel ticks |
| Telemetry | 20–30 Hz | Decoupled from physics |
| Rendering | Browser rate | May drop frames without affecting sim |

A run can be executed at:

- **real time** for interactive driving;
- **slow motion** for debugging;
- **as fast as possible** for batch experiments (headless).

## 5. Race manager

- Detects start/finish line crossing using ground-truth referee state.
- Counts 5 consecutive laps; records per-lap and total time.
- Validates that a lap is complete (full loop, correct direction).
- Tracks penalties: wall deformation, intervention, incomplete lap.
- Provides race state to the controller (lap number, elapsed time) but not the
  ground-truth pose.

## 6. Determinism checklist

- Fixed physics timestep and solver iterations.
- Seeded per-sensor RNG streams.
- No wall-clock or frame-rate dependence in physics/sensors.
- Config hash + seed + code version stored with every run.
- Batch workers use the same code path as interactive runs.
- Golden-run test: fixed seed and config produce identical metrics.

## 7. Performance budget

- Physics + sensors should run at least 5–10× real time headless on the laptop.
- Lidar raycasts: 720 beams × 3 sub-rays × 10 Hz = 21,600 rays/s; batched.
- SLAM: <20 ms per scan target on desktop.
- Browser telemetry: ≤2 MB/s typical, with binary arrays and decimation.

## 8. What is deliberately simplified

- Pipe walls are rigid; deformation is a penalty event, not full FEM.
- Tire model is a tunable friction model, not a full Pacejka curve.
- Motor/ESC thermal behaviour is optional.
- No aerodynamic ground effect or indoor air flow.
- No camera rendering in the MVP.

These simplifications are documented so results are interpreted correctly.
