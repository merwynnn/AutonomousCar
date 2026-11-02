"""Portable runtime boundary shared by the simulator and the real car.

Everything in this module is intended to run unchanged on the Raspberry Pi.
It must never import ``simulator``, ``dashboard``, a physics backend or a
dashboard dependency. Algorithms import this module (directly or through the
``algorithms.interface`` compatibility re-export) plus NumPy only.

The contract deliberately separates **raw sensor frames** from the **fused
state estimate**:

```text
Observation
├── sensors: SensorFrame   raw, timestamped measurements
└── state:   StateEstimate fused pose / speed / yaw rate
```

Adding a sensor means adding a typed frame here, an optional field on
``SensorFrame``, a simulator model in ``simulator/sensors/`` and a hardware
driver in ``onboard/drivers/``. ``SensorFrame.extra`` exists for short-lived
experiments, not as the normal extension path.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

import numpy as np


@dataclass(frozen=True)
class Pose2:
    """Planar pose in metres and radians (x forward, y left, yaw CCW)."""

    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class Actuation:
    """Normalised command sent to the car.

    ``throttle`` is a fraction of the configured maximum speed, not a raw motor
    torque or PWM value. ``steering`` is a desired road-wheel angle in radians,
    positive left. The actuator layer owns the conversion to servo/VESC units.
    """

    throttle: float = 0.0  # -1 (reverse) .. +1 (forward)
    steering: float = 0.0  # radians, positive left


@dataclass(frozen=True)
class RobotLimits:
    """Static actuation limits a controller may rely on."""

    max_steering_rad: float = 0.49
    max_steering_rate_rad_s: float = 1.57
    max_speed_mps: float = 4.0


@dataclass(frozen=True)
class LidarObservation:
    """One 2D lidar scan in the lidar frame."""

    angles_rad: np.ndarray
    ranges_m: np.ndarray  # NaN for invalid beams
    intensities: np.ndarray
    valid: np.ndarray
    t_measure: float
    t_available: float
    frame_id: str = "lidar"
    # When true, angles/ranges are re-expressed in the current lidar frame at
    # ``t_compensated``. ``t_measure``/``t_available`` still describe the
    # original raw measurement.
    motion_compensated: bool = False
    t_compensated: float | None = None

    @property
    def beam_count(self) -> int:
        return int(self.angles_rad.size)

    @property
    def age_s(self) -> float | None:
        """Age of the measurement at the compensation time, if known."""

        if self.t_compensated is None:
            return None
        return max(0.0, self.t_compensated - self.t_measure)


@dataclass(frozen=True)
class UltrasonicObservation:
    """One ultrasonic range reading in the sensor frame."""

    name: str
    range_m: float | None
    valid: bool
    t_measure: float
    t_available: float


@dataclass(frozen=True)
class ImuSample:
    """One IMU sample.

    ``gyro_rad_s`` and ``accel_m_s2`` are in the IMU frame. The planar estimator
    uses ``gyro_rad_s[2]`` (yaw rate, positive left) and ignores the rest.
    """

    gyro_rad_s: tuple[float, float, float]
    accel_m_s2: tuple[float, float, float]
    t_measure: float
    t_available: float
    frame_id: str = "imu"
    valid: bool = True


@dataclass(frozen=True)
class EncoderSample:
    """Cumulative wheel-encoder counts.

    ``ticks`` is ordered ``(front_left, front_right, rear_left, rear_right)``.
    Drivers may omit ticks (``None``) when a low-level board already reports
    wheel speeds; in that case ``wheel_speeds_mps`` is used directly. ``dt`` is
    the elapsed time covered by the tick delta.
    """

    ticks: tuple[int, int, int, int] | None
    wheel_speeds_mps: tuple[float, float, float, float] | None
    t_measure: float
    t_available: float
    dt: float
    frame_id: str = "encoders"
    dropped: bool = False


@dataclass(frozen=True)
class SteeringSample:
    """Measured steering angle (positive left) in the vehicle frame."""

    angle_rad: float
    t_measure: float
    t_available: float
    frame_id: str = "steering"
    valid: bool = True


@dataclass(frozen=True)
class MotorSample:
    """VESC-class motor telemetry in SI units.

    ``erpm`` is electrical RPM, ``duty`` is the commanded duty cycle in
    ``[-1, 1]`` and ``current_a`` is the motor current. The controller can use
    this to detect saturation, battery sag or a stalled drivetrain.
    """

    erpm: float
    current_a: float
    duty: float
    voltage_v: float
    temperature_c: float
    t_measure: float
    t_available: float
    frame_id: str = "motor"
    valid: bool = True


WHEEL_ORDER: tuple[str, str, str, str] = (
    "front_left",
    "front_right",
    "rear_left",
    "rear_right",
)
REAR_WHEEL_INDICES: tuple[int, int] = (2, 3)


@dataclass(frozen=True)
class SensorFrame:
    """All raw sensor measurements available at one control tick."""

    lidar: LidarObservation | None = None
    ultrasonic: tuple[UltrasonicObservation, ...] = ()
    imu: ImuSample | None = None
    encoders: EncoderSample | None = None
    steering: SteeringSample | None = None
    motor: MotorSample | None = None
    extra: Mapping[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class StateEstimate:
    """Fused planar state produced by the shared estimator.

    ``covariance`` is a 3x3 array ordered ``(x, y, theta)``. The first
    estimator implementation reports a conservative diagnostic covariance, not
    a calibrated EKF posterior.
    """

    t: float
    pose: Pose2
    speed_mps: float
    yaw_rate_rad_s: float
    covariance: np.ndarray
    valid: bool
    source: str = "wheel_imu"
    gyro_bias_rad_s: float = 0.0
    slip_detected: bool = False


@dataclass(frozen=True)
class Observation:
    """Everything an algorithm is allowed to know at one control tick.

    There is deliberately no ground-truth pose, no future scan and no map.
    Controllers can read raw sensor data from ``sensors`` and the shared
    estimator output from ``state``. Compatibility properties keep the older
    ``observation.lidar`` / ``observation.odometry`` style working.
    """

    t: float
    dt: float
    sensors: SensorFrame
    state: StateEstimate
    limits: RobotLimits = field(default_factory=RobotLimits)
    # Scan re-expressed in the current lidar frame by the shared
    # ``ScanMotionCompensator``. ``sensors.lidar`` remains the raw measurement.
    compensated_lidar: LidarObservation | None = None

    # ----------------------------------------------------- compatibility
    @property
    def lidar_current(self) -> LidarObservation | None:
        """Best scan in the current lidar frame, falling back to raw data.

        Algorithms that do not care about the distinction should use this
        property instead of ``observation.lidar``. The raw scan remains
        available as ``observation.sensors.lidar``.
        """

        return self.compensated_lidar or self.sensors.lidar

    @property
    def odometry(self) -> Pose2:
        return self.state.pose

    @property
    def speed_mps(self) -> float:
        return self.state.speed_mps

    @property
    def yaw_rate_rad_s(self) -> float:
        return self.state.yaw_rate_rad_s

    @property
    def lidar(self) -> LidarObservation | None:
        return self.sensors.lidar

    @property
    def ultrasonic(self) -> tuple[UltrasonicObservation, ...]:
        return self.sensors.ultrasonic

    @property
    def imu(self) -> ImuSample | None:
        return self.sensors.imu

    @property
    def encoders(self) -> EncoderSample | None:
        return self.sensors.encoders

    @property
    def steering(self) -> SteeringSample | None:
        return self.sensors.steering

    @property
    def motor(self) -> MotorSample | None:
        return self.sensors.motor


class Controller(Protocol):
    """Implement this protocol to drive the car."""

    def reset(self) -> None:
        """Called once before the first update."""

    def update(self, observation: Observation) -> Actuation:
        """Return a command for this control tick."""


@runtime_checkable
class RobotAPI(Protocol):
    """Portable robot boundary implemented by the simulator and the real car."""

    def reset(self) -> Observation:
        """Reset the robot and return the first observation."""

    def step(self, command: Actuation) -> Observation:
        """Apply a command for one control period and return the next observation."""

    def close(self) -> None:
        """Release hardware/simulation resources."""


# Short aliases used by newer code and documentation.
LidarFrame = LidarObservation
UltrasonicReading = UltrasonicObservation

__all__ = [
    "Actuation",
    "Controller",
    "EncoderSample",
    "ImuSample",
    "LidarFrame",
    "LidarObservation",
    "MotorSample",
    "Observation",
    "Pose2",
    "REAR_WHEEL_INDICES",
    "RobotAPI",
    "RobotLimits",
    "SensorFrame",
    "StateEstimate",
    "SteeringSample",
    "UltrasonicObservation",
    "UltrasonicReading",
    "WHEEL_ORDER",
]
