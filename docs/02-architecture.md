# Architecture

```text
Browser (React + Three.js)
    │ REST + WebSocket JSON
FastAPI dashboard (dashboard/, host only)
    │
DashboardSimulation (adapts simulator.Simulation)
    │
simulator/  (MuJoCo, simulated sensors, track, timing; host only)
    │  implements onboard.api.RobotAPI via simulator/bridge.py
onboard/    (portable contract, shared estimator, Pi runtime and drivers)
    │
algorithms/ (Observation -> Actuation, portable; optional debug frames)
```

## Boundaries

- `onboard/` is the deployable runtime. It owns `Observation`, `Actuation`,
  `SensorFrame`, `StateEstimate`, `RobotAPI`, the shared estimator, the control
  loop and the hardware-facing protocols.
- `onboard/` and `algorithms/` must never import `simulator.*` or
  `dashboard.*`, a physics backend or FastAPI. `tests/test_boundary.py` enforces
  the rule.
- `simulator/` owns physics, simulated sensor measurements, latency, the track
  and lap timing. It must not import dashboard code or React state.
- `simulator/bridge.py` exposes `SimRobotAPI`, so the simulator is only one
  provider of the portable robot interface.
- `algorithms/` owns driving policy. It imports only `onboard.api`, its own
  package modules and NumPy. `algorithms/interface.py` remains a compatibility
  re-export. `algorithms/debug.py` is a host-neutral debug snapshot contract and
  `algorithms/registry.py` lists controllers available to the dashboard.
- `dashboard/` owns transport, operator commands and rendering. It may read
  ground truth for display but passes only observations to controllers.
- The browser never runs physics.

## Observation contract

```text
Observation
├── t, dt
├── sensors: SensorFrame
│   ├── lidar: LidarObservation | None
│   ├── ultrasonic: tuple[UltrasonicObservation, ...]
│   ├── imu: ImuSample | None
│   ├── encoders: EncoderSample | None
│   ├── steering: SteeringSample | None
│   ├── motor: MotorSample | None
│   └── extra: Mapping[str, object]
├── state: StateEstimate
│   ├── pose, speed_mps, yaw_rate_rad_s
│   ├── covariance, valid, source
│   └── gyro_bias_rad_s, slip_detected
├── limits: RobotLimits
│   ├── max_steering_rad
│   ├── max_steering_rate_rad_s
│   └── max_speed_mps
└── compensated_lidar: LidarObservation | None
    (raw scan re-expressed in the current lidar frame;
     `lidar_current` falls back to `sensors.lidar`)
```

Raw data and fused state are both available. Compatibility properties
(`observation.lidar`, `observation.odometry`, ...) keep older controllers
working. `onboard/scan_motion.py` produces the compensated scan from the
estimated pose history and the lidar mount; it never reads ground truth.

## Data flow

1. Physics advances at a fixed rate.
2. Simulator sensor models produce timestamped raw frames: lidar, ultrasonic,
   encoders, IMU, steering and motor telemetry.
3. The shared `onboard.estimator.WheelImuEstimator` consumes only the frames
   whose `t_available` is in the past and produces `StateEstimate`.
4. `onboard.scan_motion.ScanMotionCompensator` re-expresses the latest scan in
   the current lidar frame using the estimated pose history and mount.
5. `DashboardSimulation` calls the controller with the resulting `Observation`.
6. The manager broadcasts `state`, `scan` and `hello` messages over WebSocket.
7. The browser renders messages. It never predicts or integrates physics.
8. Commands (`run.start`, `drive.command`, `config.reload`, etc.) travel back
   over the same WebSocket.

On the real car the same estimator and controller run in `onboard/runtime.py`.
A `SensorSource` reads real drivers, an `ActuatorSink` sends commands, and the
only difference is where `SensorFrame` comes from. Both sides use
`onboard.actuation` for the servo/VESC conversion; `MockBoard` exercises the same
serial path without hardware.

## Configuration

Shared configuration lives in the root `configs/` directory:

- `vehicle_awake2027.yaml` / `vehicle.yaml`: car definition;
- `sensors_awake2027.yaml` / `sensors.yaml`: sensor mounts and characteristics;
- `estimator.yaml`: shared estimator tuning;
- `hardware.yaml`: servo/VESC/board calibration shared with the Pi;
- `sim/*.yaml`: simulator-only scenarios.

A scenario references its vehicle and sensors relative to `configs/sim/`
(`../vehicle*.yaml`). `Simulation` can also receive `VehicleConfig` and
`SensorsConfig` directly from Python.

The generated MJCF, dashboard car studio and physics collision geometry all use
the same `VehicleConfig`; `dashboard/spec.py` serializes it for the browser.
