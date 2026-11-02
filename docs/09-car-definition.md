# Car definition

The car is defined once and everything else is derived from it.

## Sources

- `configs/vehicle_awake2027.yaml` — reference AWAKE 2027 car (BOM in
  [`10-hardware.md`](10-hardware.md)).
- `configs/vehicle.yaml` — generic example car.
- `configs/sensors.yaml` / `sensors_awake2027.yaml` — sensor
  characteristics and body-frame mounts.
- Python — load the same models and modify them, then pass them to
  `Simulation(vehicle=..., sensors=...)`. See `examples/custom_car.py`.
- `configs/estimator.yaml` — shared estimator tuning only. Wheel radius,
  wheelbase, track width and encoder resolution are read from the vehicle and
  sensor files by `onboard.config.load_onboard_config`.

The vehicle config generates the MuJoCo MJCF. The dashboard car studio reads the
same object, so physics and the rendered car cannot silently drift.

## Body frame

Positions use the vehicle body frame: **x forward, y left, z up**, origin at the
chassis centre. Sensor mount RPY is applied as roll, pitch, yaw in that order.

## Structure parts

```yaml
structure:
  - name: main_body
    shape: box            # box | cylinder | sphere
    position_m: [0, 0, 0.005]
    rpy_deg: [0, 0, 0]
    size_m: [0.38, 0.18, 0.055]
    color: "#a53235"
    opacity: 1.0
    visible: true
    collision: true
```

- `box`: `size_m` is full length, width, height.
- `cylinder`: `[diameter, diameter, height]`.
- `sphere`: `[diameter, diameter, diameter]`.
- `collision: false` parts are visual-only; they still appear in the car studio.
- The generated MJCF uses the same parts for collision and visual geometry.

## Wheels

Wheel joints are derived from `wheelbase_m`, `track_width_m`,
`wheel_offset_z_m`, `wheel_radius_m` and `wheel_width_m`. The ride height is
`wheel_radius_m - wheel_offset_z_m`. Changing a wheel parameter updates the
generated MJCF, physics and dashboard model in one step.

## Reloading

After editing YAML, click **Reload config** in the dashboard. The dashboard
reloads the scenario, rebuilds the MJCF and sensor models and sends a fresh car
payload to the browser.

The **Tune** console edits the raw vehicle and sensor configs in the browser.
**Apply live** rebuilds without writing files; **Apply & save YAML** validates,
writes the shared vehicle and sensor YAML files (preserving comments where
possible) and rebuilds the estimator and simulation.
`Ctrl+Z`/`Ctrl+Y` undo/redo draft values.

Code-defined cars are rebuilt by constructing a new `Simulation`.

## Validation

- Competition size limits are enforced by `VehicleConfig`.
- Total mass must exceed the four wheel masses.
- Sensor mounts are validated as typed `Mount` objects and serialized with their
  characteristics to the dashboard.
