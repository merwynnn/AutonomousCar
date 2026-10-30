# Non-ideal sensor models

> **Rule:** realistic is the default. Ideal sensors are a debug mode only.
> Controllers and SLAM never receive ground truth or future measurements.

## 1. Why imperfection matters

A perfect simulator makes every strategy look good. The real car will face:

- lidar range and angle noise, dead beams, dropouts and motion distortion;
- 20–80 ms of sensor and processing latency;
- ultrasonic cone ambiguity, blind zones and missed thin objects;
- IMU drift and wheel slip;
- steering backlash and motor voltage sag.

These effects change lap times, safety margins and SLAM quality. They are
therefore part of the default simulation, not optional polish.

## 2. Realism presets

| Preset | Use | Behaviour |
|---|---|---|
| `ideal` | Debug, algorithm sanity checks | Analytic ranges, no noise, no latency, no drops |
| `realistic` | Default, strategy comparison | Moderate noise, dropouts, latency, vibration |
| `harsh` | Robustness / competition worst case | Stronger noise, more drops, higher latency, miscalibration |

Every preset is implemented as a set of parameters, not as separate code paths.
The UI shows the active preset and a warning when `ideal` is selected.

## 3. Lidar — LD06-class 2D spinning lidar

Default configuration:

| Parameter | Default |
|---|---|
| Beams per scan | 720 (0.5°) |
| Scan rate | 10 Hz |
| Range | 0.03–12 m |
| Range accuracy | ±1.5 cm typical |
| Angular resolution | 0.5° |
| Mount height | 0.14 m (configurable) |
| Scan direction | clockwise |
| Interface latency | 35 ms + jitter |

### 3.1 Measurement model

For each beam `i`:

1. Compute the beam angle at its **actual capture time**:
   `t_i = t_start + i / N * T_scan + jitter_i`.
2. Cast a small bundle of sub-rays (default 3) inside the beam divergence.
3. Interpolate the car/lidar pose at `t_i` and transform the ray into world.
4. Raycast against walls, obstacles and the car body (self-filter).
5. Combine sub-rays into a range (min or probabilistic mixed pixel).
6. Apply range bias, Gaussian noise, quantization and dropout.
7. Compute intensity from material, incidence angle and range.
8. Publish the point in the scan-start lidar frame, with its timestamp.

The raw scan is therefore **motion-distorted** exactly like a real spinning lidar.
SLAM must deskew it using IMU/odometry; the dashboard can show raw vs deskewed.

### 3.2 Non-idealities

| Effect | Model |
|---|---|
| Range noise | Bias + Gaussian, σ increases with range (default 1–1.5 cm) |
| Angle noise | Per-beam jitter, rotation-speed variation |
| Quantization | Range to 1 mm, angle to encoder resolution |
| Motion distortion | Per-beam timestamp + pose interpolation |
| Dropouts | Logistic probability vs range, incidence and reflectivity |
| Dead beams | 0–2% of beams stuck at max range or zero |
| Dropped scans | 1% of scans missing, plus packet-level loss |
| Beam divergence | 3 sub-rays; mixed pixels at edges |
| Intensity | Material reflectivity × cos(incidence)^k / range² + noise |
| Wall height | 16 cm walls; no return if the scan plane is above them |
| Self-occlusion | Car body/wheels block blind sectors |
| Mounting | Extrinsic pose, static tilt, vibration |
| Latency | 35 ms + 0–15 ms jitter |
| Calibration error | Range scale/bias, angle offset, extrinsic yaw/pitch |
| Temperature | Slow drift of range bias and angle scale |
| Grazing incidence | Weaker returns and higher dropout probability |
| Floor | Returns only if the lidar is tilted; floor material is dark/absorptive |

**Wall-height insight:** a lidar mounted above 16 cm sees over the track walls.
The simulator will warn if the configured height makes wall detection unreliable.
This is a real hardware design constraint for the car.

### 3.3 Validation

- Analytic flat wall at 1, 3 and 8 m: mean range and σ within configured bounds.
- Cylinder at known distance: range scalloping and incidence-dependent intensity.
- Corner scene: correct occlusion and beam ordering.
- Wall-height test: 0.14 m mount sees the wall; 0.18 m mount does not.
- Motion-distortion test: at 3 m/s, raw points smear; deskewed points align with
  ground truth within a tolerance.
- Dropout test: dropout rate increases with incidence/range, never zero by default.
- Latency test: a scan with `t_available` in the future is not visible to control.
- Determinism test: same seed → identical scan sequence.

## 4. Ultrasonic sensors

Three default sensors: front-left, front-right, rear-centre. Each is an
independent model with its own RNG stream.

| Effect | Model |
|---|---|
| Cone | 30° full angle, sampled by 15 rays |
| Range | 0.02–4 m; outside → no echo |
| Blind zone | <2 cm not detected |
| Noise | 1 cm + 1% of range, Gaussian + quantization |
| Update rate | 20 Hz |
| Latency | 30 ms |
| Cross-talk | Firing overlap causes false short readings |
| Specular dropout | Smooth surfaces at grazing incidence may not return |
| Thin objects | Poles narrower than the cone sampling may be missed |
| Floor bounce | Modelled when mounted with downward tilt |
| EMI | Occasional spikes correlated with motor current |
| Mounting | Extrinsic pose + occlusion by the car body |

The ultrasonic model is used for low-speed safety and redundancy; it is not the
primary racing sensor.

## 5. IMU

| Effect | Model |
|---|---|
| Gyro bias | Constant + temperature-dependent, default 0.5–2°/s |
| Gyro noise | White noise + random walk |
| Gyro scale/cross-axis | 1% scale, 0.5% cross-axis |
| Accel bias/noise | 0.05 m/s² bias, 0.02 m/s²/√Hz noise |
| Vibration | Motor/floor-induced oscillation |
| Saturation | Gyro ±500°/s, accel ±16 g |
| Mounting | 1° misalignment by default |
| Rate/latency | 200 Hz, 5 ms |
| Orientation | Not provided as perfect attitude; integration drifts |

The SLAM motion model uses raw gyro/odometry, not a perfect heading.

## 6. Wheel odometry and steering

| Effect | Model |
|---|---|
| Encoder resolution | Configurable ticks/rev; quantization |
| Wheel radius error | 2% default |
| Track width error | 1% default |
| Slip | Longitudinal/lateral slip from the physics contact state |
| Dropouts | Occasional missing ticks |
| Timestamp jitter | Per-sample jitter |
| Steering sensor | Quantization, deadband, backlash, latency |
| Steering command | Rate limit, saturation, servo lag |

Odometry therefore overestimates speed during wheelspin and drifts during
cornering, as on the real car.

## 7. Actuator realism

- Steering servo: max angle, rate limit, deadband, backlash, latency.
- Motor/ESC: torque curve, current limit, battery voltage sag, thermal derating,
  braking behaviour, 4WD torque split.
- These affect lap times and must be included in strategy comparisons.

## 8. Environment effects

- Floor friction varies spatially and with seams.
- Pipe walls have small radius/height irregularities.
- Obstacle placement has a small uncertainty in generated scenarios.
- Chassis vibration is injected into sensor poses.
- Lighting is irrelevant for lidar/ultrasonic but will matter for the optional
  camera.

## 9. Latency-aware sensor bus

Each message has:

```text
t_measure    simulation time of the physical measurement
t_available  t_measure + transport/processing latency (+ jitter)
frame_id     sensor frame
payload      ranges/intensities/accel/ticks ...
valid        false for dropped/corrupted data
```

At control time `t`, the bus exposes only messages with `t_available <= t`. This
is enforced centrally and tested; individual controllers cannot bypass it.

## 10. Visualization and diagnostics

The dashboard exposes:

- raw vs deskewed lidar;
- points coloured by intensity, age or dropout;
- ultrasonic cones and hit points;
- IMU bias/drift and odometry error vs ground truth;
- sensor rate, latency, dropout rate, dead beams, calibration offsets;
- a “sensor health” panel and warnings (e.g., lidar above wall height);
- toggles for `ideal`/`realistic`/`harsh` and individual effects.

## 11. What is not modelled yet

- Full 3D multi-layer lidar (the competition reference is 2D).
- Camera rolling shutter, exposure and lens distortion (optional later).
- Multipath reflections and transparent surfaces.
- Detailed ultrasonic transducer physics.
- Temperature chambers and long-term aging.

These are candidates for later milestones, not blockers for the strategy lab.
