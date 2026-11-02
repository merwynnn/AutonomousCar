# Sensor models

All sensors use seeded NumPy RNG streams. `ideal` multiplies noise, dropout and
latency effects by zero; `realistic` is the default; `harsh` increases the
effects. Sensor mounts are in the vehicle body frame (x forward, y left, z up).

## Lidar

`LidarConfig` includes beams, rate, min/max range, angular resolution, scan
direction, mount position/RPY, range/angle noise, bias, scale error,
quantization, divergence sub-rays, incidence dropout, dead beams, dropped scans,
packet loss, latency/jitter, temperature drift and self-filter warnings.

The model:

- assigns each beam a capture time during the rotating scan;
- traces 3D rays from the rotated mount through the MuJoCo world, so mount RPY
  affects what is measured;
- combines sub-rays for mixed pixels;
- transforms hits into the scan-start lidar frame, leaving motion distortion for
  downstream algorithms;
- applies incidence dropout, range/angle noise, quantization, intensity and
  latency.

The dashboard live mode forces one sub-ray per beam to keep frame rate; CLI and
batch simulation keep the configured divergence.

### Motion compensation

A scan is acquired over a full sweep and expressed in the lidar frame at the
start of that sweep. By the time a controller receives it, the car has moved.
`onboard/scan_motion.py` keeps a short history of the estimated vehicle pose
(from `StateEstimate`, never ground truth) and re-expresses the latest scan in
the current lidar frame:

```text
p_now = T_lidar(now)^-1 * T_lidar(scan_start) * p_scan
```

The result is exposed as `Observation.compensated_lidar`; the convenience
property `Observation.lidar_current` returns it and falls back to the raw scan.
The raw measurement remains available as `Observation.sensors.lidar` and stays
in the scan-start lidar frame.

The transform is rigid and planar. It corrects the bulk motion between scans,
including the lidar mount lever arm, but not per-beam motion distortion inside
the 100 ms sweep. Full deskewing needs per-beam timestamps from the driver,
which the current `LidarObservation` does not yet expose.

Timing note: the model currently marks a scan available after
`latency_ms` from scan start, while a physical sweep takes one full period.
The scan is therefore optimistic compared with a real driver; a per-beam
availability model is the next timing improvement.

## Ultrasonic

`UltrasonicConfig` includes rate, cone angle, rays, min/max range, range noise,
latency, cross-talk and specular dropout probabilities. Each mounted sensor has a
body-frame mount with position and RPY. The model samples a ray fan inside the
cone, takes the nearest hit, then adds range noise, blind-zone clamping, random
specular dropouts and cross-talk spikes.

## Encoders

`EncoderConfig` contains rate, ticks per revolution, wheel-radius and
track-width calibration errors, timestamp jitter, latency and dropout
probability. The model converts the four physics wheel-joint speeds into
cumulative ticks. A positive `wheel_radius_error` makes the encoder under-report
distance; `track_width_error` scales the left/right differential.

Dropped samples keep the previous tick count and are marked `dropped=True`; the
shared estimator ignores them. Wheel spin from the contact model includes tyre
slip, so encoder-derived speed can disagree with the true body speed.

## IMU

`ImuConfig` contains gyro bias and random walk, scale error, noise, cross-axis
coupling, saturation, accelerometer noise/vibration and mounting misalignment.
The model produces `ImuSample` with gyro and accelerometer vectors plus
measurement/availability timestamps. The planar estimator uses the yaw-rate
component and learns the gyro bias.

## Steering sensor

`SteeringSensorConfig` models the steering-linkage sensor: rate, noise,
quantization and latency. It produces `SteeringSample` in the vehicle frame
(positive left), independent of the commanded steering angle. The actuator
layer separately models command latency, rate, deadband and backlash.

## Motor telemetry

`MotorModel` produces `MotorSample` with ERPM, motor current, duty, battery
voltage and temperature. It is a measurement model derived from the MuJoCo
wheel state and the commanded throttle, not a second motor model. Controllers
can use it to detect saturation, battery sag or a stalled drivetrain. The
serial driver also decodes the same fields from board packets.

## Shared estimator

`onboard/estimator/wheel_imu.py` fuses the raw frames:

- forward speed from the rear-wheel encoder ticks;
- yaw rate primarily from the gyro, with wheel and steering checks;
- zero-velocity gyro-bias learning when the car is still;
- slip detection when wheel and gyro yaw rates disagree.

It is a complementary estimator, not a full EKF. Its 3x3 covariance is a
conservative diagnostic estimate; a later EKF can replace it without changing
the controller contract. The same estimator is used by the simulator and by the
Raspberry Pi runtime.
