from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import pytest
from onboard.api import (
    Actuation,
    EncoderSample,
    ImuSample,
    Observation,
    Pose2,
    SensorFrame,
    SteeringSample,
)
from onboard.config import EstimatorTuning, load_onboard_config
from onboard.drivers.serial_link import (
    SerialSensorSource,
    decode_sensor_packet,
    encode_actuation,
)
from onboard.estimator import WheelImuEstimator
from onboard.runtime import ControllerRuntime, SensorRobot
from simulator.config import DEFAULT_ESTIMATOR_PATH

ROOT = Path(__file__).resolve().parents[1]
RADIUS = 0.035
TRACK = 0.20
WHEELBASE = 0.26
TICKS = 1024


def _encoder_ticks(distance_m: float) -> int:
    return round(distance_m / (2.0 * math.pi * RADIUS) * TICKS)


def _frame(
    t: float,
    *,
    left_ticks: int = 0,
    right_ticks: int = 0,
    dt: float = 0.02,
    gyro_z: float = 0.0,
    steering: float = 0.0,
) -> SensorFrame:
    return SensorFrame(
        encoders=EncoderSample(
            ticks=(left_ticks, right_ticks, left_ticks, right_ticks),
            wheel_speeds_mps=None,
            t_measure=t,
            t_available=t,
            dt=dt,
        ),
        imu=ImuSample(
            gyro_rad_s=(0.0, 0.0, gyro_z),
            accel_m_s2=(0.0, 0.0, 9.81),
            t_measure=t,
            t_available=t,
        ),
        steering=SteeringSample(steering, t, t),
    )


def _estimator() -> WheelImuEstimator:
    return WheelImuEstimator(
        EstimatorTuning(gyro_bias_gain=0.0, gyro_bias_tracking_gain=0.0),
        wheel_radius_m=RADIUS,
        track_width_m=TRACK,
        wheelbase_m=WHEELBASE,
        ticks_per_rev=TICKS,
    )


def test_estimator_integrates_rear_encoder_distance() -> None:
    estimator = _estimator()
    estimator.reset(Pose2(0.0, 0.0, 0.0))
    tick = _encoder_ticks(0.01)
    estimator.update(_frame(0.0), 0.0)
    estimate = estimator.update(_frame(0.02, left_ticks=tick, right_ticks=tick), 0.02)
    assert estimate.speed_mps == pytest.approx(0.5, abs=0.02)
    assert estimate.pose.x == pytest.approx(0.01, abs=0.002)
    assert estimate.source == "wheel_imu"


def test_estimator_uses_gyro_yaw_rate_when_wheels_agree() -> None:
    estimator = _estimator()
    estimator.reset(Pose2(0.0, 0.0, 0.0))
    left = _encoder_ticks(0.01)
    right = _encoder_ticks(0.02)
    wheel_yaw = (0.02 - 0.01) / 0.02 / TRACK
    estimator.update(_frame(0.0), 0.0)
    estimate = estimator.update(
        _frame(0.02, left_ticks=left, right_ticks=right, gyro_z=wheel_yaw),
        0.02,
    )
    assert estimate.slip_detected is False
    assert estimate.yaw_rate_rad_s == pytest.approx(wheel_yaw, abs=0.05)
    assert estimate.pose.theta > 0.0


def test_estimator_flags_wheel_slip_and_trusts_the_gyro() -> None:
    estimator = _estimator()
    estimator.reset(Pose2(0.0, 0.0, 0.0))
    left = _encoder_ticks(0.01)
    right = _encoder_ticks(0.02)
    estimator.update(_frame(0.0), 0.0)
    estimate = estimator.update(
        _frame(0.02, left_ticks=left, right_ticks=right, gyro_z=0.0),
        0.02,
    )
    assert estimate.slip_detected is True
    assert abs(estimate.yaw_rate_rad_s) < 0.5


def test_zero_velocity_update_learns_gyro_bias() -> None:
    tuning = EstimatorTuning(gyro_bias_gain=0.5, zero_velocity_yaw_rate_rad_s=0.08)
    estimator = WheelImuEstimator(
        tuning,
        wheel_radius_m=RADIUS,
        track_width_m=TRACK,
        wheelbase_m=WHEELBASE,
        ticks_per_rev=TICKS,
    )
    estimator.reset(Pose2(0.0, 0.0, 0.0))
    for index in range(400):
        now = index * 0.01
        estimate = estimator.update(_frame(now, gyro_z=0.05, dt=0.01), now)
    assert estimate.gyro_bias_rad_s > 0.03


@dataclass
class _FakeSource:
    frame: SensorFrame
    closed: bool = False

    def read(self, now: float) -> SensorFrame:
        return self.frame

    def close(self) -> None:
        self.closed = True


@dataclass
class _FakeSink:
    commands: list[Actuation] = field(default_factory=list)
    closed: bool = False

    def apply(self, command: Actuation, now: float) -> None:
        self.commands.append(command)

    def close(self) -> None:
        self.closed = True


class _StopController:
    def reset(self) -> None:
        pass

    def update(self, observation: Observation) -> Actuation:
        return Actuation(throttle=0.0, steering=0.0)


def test_sensor_robot_and_controller_runtime() -> None:
    source = _FakeSource(_frame(0.0))
    sink = _FakeSink()
    robot = SensorRobot(source, sink, _estimator(), initial_pose=Pose2(1.0, 2.0, 0.1))
    runtime = ControllerRuntime(robot, _StopController())
    observation = runtime.run(3)
    assert observation.t == pytest.approx(3.0 / 60.0)
    assert len(sink.commands) == 4  # reset + three steps
    runtime.close()
    assert source.closed and sink.closed


def test_serial_packet_round_trip() -> None:
    command = Actuation(throttle=0.4, steering=-0.1)
    payload = json.loads(encode_actuation(command, 1.25))
    assert payload["type"] == "actuation"
    assert payload["throttle"] == 0.4

    frame = decode_sensor_packet(
        b'{"t":1.25,"encoders":{"ticks":[1,2,3,4],"dt":0.02},'
        b'"imu":{"gyro_rad_s":[0,0,0.1],"accel_m_s2":[0,0,9.81]},'
        b'"steering":{"angle_rad":0.05}}'
    )
    assert frame.encoders is not None
    assert frame.encoders.ticks == (1, 2, 3, 4)
    assert frame.imu is not None
    assert frame.imu.gyro_rad_s[2] == 0.1
    assert frame.steering is not None
    assert frame.steering.angle_rad == 0.05


@dataclass
class _QueueTransport:
    lines: list[bytes]

    def readline(self) -> bytes | None:
        return self.lines.pop(0) if self.lines else None

    def write(self, data: bytes) -> None:
        pass

    def close(self) -> None:
        pass


def test_serial_sensor_source_keeps_latest_frame() -> None:
    transport = _QueueTransport(
        [
            b'{"t":1.0,"encoders":{"ticks":[0,0,0,0],"dt":0.02}}\n',
            b'{"t":1.1,"encoders":{"ticks":[1,1,2,2],"dt":0.02}}\n',
        ]
    )
    source = SerialSensorSource(transport)
    frame = source.read(1.2)
    assert frame.encoders is not None
    assert frame.encoders.ticks == (1, 1, 2, 2)
    assert frame.encoders.t_measure == 1.1


def test_onboard_config_reads_shared_car_files() -> None:
    config = load_onboard_config(
        ROOT / "configs" / "vehicle_awake2027.yaml",
        ROOT / "configs" / "sensors_awake2027.yaml",
        DEFAULT_ESTIMATOR_PATH,
    )
    assert config.vehicle_name == "awake_2027_reference_car"
    assert config.wheel_radius_m == 0.035
    assert config.ticks_per_rev == 1024
    assert config.tuning.name == "wheel_imu_default"

