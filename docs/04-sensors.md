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
- traces 3D rays from the rotated mount through the active physics backend, so
  mount RPY affects what is measured;
- combines sub-rays for mixed pixels;
- transforms hits into the scan-start lidar frame, leaving motion distortion for
  downstream algorithms;
- applies incidence dropout, range/angle noise, quantization, intensity and
  latency.

The dashboard live mode forces one sub-ray per beam to keep frame rate; CLI and
batch simulation keep the configured divergence.

## Ultrasonic

`UltrasonicConfig` includes rate, cone angle, rays, min/max range, range noise,
latency, cross-talk and specular dropout probabilities. Each mounted sensor has a
body-frame mount with position and RPY. The model samples a ray fan inside the
cone, takes the nearest hit, then adds range noise, blind-zone clamping, random
specular dropouts and cross-talk spikes.

## Odometry and encoders

`OdometryModel` integrates the measured speed and yaw rate with scale error,
bias, random walk and noise. It intentionally drifts. Encoder characteristics
(rate, ticks, wheel radius error, track width error, jitter and dropout) are part
of `SensorsConfig` and are available to the dashboard and future hardware adapters.
