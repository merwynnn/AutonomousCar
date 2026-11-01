"""JSON-lines protocol for the low-level board and a serial driver skeleton.

The protocol is intentionally small: the RP2040/Teensy timestamps the sensors,
the Pi timestamps availability and runs the estimator. It is not a replacement
for a binary protocol when bandwidth becomes a problem.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any

from onboard.api import (
    Actuation,
    EncoderSample,
    ImuSample,
    LidarObservation,
    SensorFrame,
    SteeringSample,
    UltrasonicObservation,
)
from onboard.drivers.base import LidarDriver, SerialTransport


def encode_actuation(command: Actuation, now: float) -> bytes:
    """Encode one command as a JSON line for the low-level board."""

    payload = {
        "type": "actuation",
        "t": round(float(now), 6),
        "throttle": float(command.throttle),
        "steering": float(command.steering),
    }
    return (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")


def _triplet(value: object, name: str) -> tuple[float, float, float]:
    if not isinstance(value, list | tuple) or len(value) != 3:
        raise ValueError(f"{name} must be a three-element array")
    return (float(value[0]), float(value[1]), float(value[2]))


def _quad(value: object, name: str) -> tuple[int, int, int, int]:
    if not isinstance(value, list | tuple) or len(value) != 4:
        raise ValueError(f"{name} must be a four-element array")
    return (int(value[0]), int(value[1]), int(value[2]), int(value[3]))


def decode_sensor_packet(line: bytes | str, t_available: float | None = None) -> SensorFrame:
    """Decode one JSON line into the portable raw sensor frame."""

    text = line.decode("utf-8") if isinstance(line, bytes) else line
    payload: Any = json.loads(text)
    if not isinstance(payload, dict):
        raise ValueError("sensor packet must be a JSON object")
    packet_type = payload.get("type")
    if packet_type not in (None, "sensors"):
        raise ValueError(f"unexpected packet type {packet_type!r}")
    t = float(payload.get("t", 0.0))
    available = t if t_available is None else float(t_available)

    encoders: EncoderSample | None = None
    encoder_data = payload.get("encoders")
    if isinstance(encoder_data, dict):
        ticks = encoder_data.get("ticks")
        speeds = encoder_data.get("wheel_speeds")
        encoders = EncoderSample(
            ticks=None if ticks is None else _quad(ticks, "encoders.ticks"),
            wheel_speeds_mps=None if speeds is None else _triplet_or_quad(speeds),
            t_measure=float(encoder_data.get("t", t)),
            t_available=float(encoder_data.get("t_available", available)),
            dt=float(encoder_data.get("dt", 0.0)),
            frame_id=str(encoder_data.get("frame_id", "encoders")),
            dropped=bool(encoder_data.get("dropped", False)),
        )

    imu: ImuSample | None = None
    imu_data = payload.get("imu")
    if isinstance(imu_data, dict):
        imu = ImuSample(
            gyro_rad_s=_triplet(imu_data.get("gyro_rad_s", (0.0, 0.0, 0.0)), "imu.gyro_rad_s"),
            accel_m_s2=_triplet(
                imu_data.get("accel_m_s2", (0.0, 0.0, 0.0)), "imu.accel_m_s2"
            ),
            t_measure=float(imu_data.get("t", t)),
            t_available=float(imu_data.get("t_available", available)),
            frame_id=str(imu_data.get("frame_id", "imu")),
            valid=bool(imu_data.get("valid", True)),
        )

    steering: SteeringSample | None = None
    steering_data = payload.get("steering")
    if isinstance(steering_data, dict):
        steering = SteeringSample(
            angle_rad=float(steering_data.get("angle_rad", 0.0)),
            t_measure=float(steering_data.get("t", t)),
            t_available=float(steering_data.get("t_available", available)),
            frame_id=str(steering_data.get("frame_id", "steering")),
            valid=bool(steering_data.get("valid", True)),
        )
    elif isinstance(steering_data, int | float):
        steering = SteeringSample(
            angle_rad=float(steering_data),
            t_measure=t,
            t_available=available,
        )

    ultrasonic: list[UltrasonicObservation] = []
    sonar_data = payload.get("ultrasonic", [])
    if isinstance(sonar_data, list):
        for reading in sonar_data:
            if not isinstance(reading, dict):
                continue
            range_value = reading.get("range_m")
            ultrasonic.append(
                UltrasonicObservation(
                    name=str(reading.get("name", "sonar")),
                    range_m=None if range_value is None else float(range_value),
                    valid=bool(reading.get("valid", range_value is not None)),
                    t_measure=float(reading.get("t", t)),
                    t_available=float(reading.get("t_available", available)),
                )
            )

    return SensorFrame(
        ultrasonic=tuple(ultrasonic),
        imu=imu,
        encoders=encoders,
        steering=steering,
        extra=dict(payload.get("extra", {})) if isinstance(payload.get("extra"), dict) else {},
    )


def _triplet_or_quad(value: object) -> tuple[float, float, float, float]:
    if not isinstance(value, list | tuple) or len(value) != 4:
        raise ValueError("encoders.wheel_speeds must be a four-element array")
    return (float(value[0]), float(value[1]), float(value[2]), float(value[3]))


@dataclass
class SerialSensorSource:
    """Drain JSON lines from the low-level board and merge an optional lidar."""

    transport: SerialTransport
    lidar: LidarDriver | None = None
    _latest: SensorFrame = field(default_factory=SensorFrame, init=False)

    def read(self, now: float) -> SensorFrame:
        while True:
            line = self.transport.readline()
            if line is None:
                break
            try:
                self._latest = decode_sensor_packet(line, t_available=now)
            except (ValueError, TypeError, json.JSONDecodeError, UnicodeDecodeError):
                # A malformed or partial line must not take down the control loop.
                continue
        scan: LidarObservation | None = None
        if self.lidar is not None:
            scan = self.lidar.read(now)
        if scan is None:
            return self._latest
        return SensorFrame(
            lidar=scan,
            ultrasonic=self._latest.ultrasonic,
            imu=self._latest.imu,
            encoders=self._latest.encoders,
            steering=self._latest.steering,
            extra=self._latest.extra,
        )

    def close(self) -> None:
        self.transport.close()
        if self.lidar is not None:
            self.lidar.close()


@dataclass
class SerialActuatorSink:
    """Send actuator commands to the low-level board."""

    transport: SerialTransport

    def apply(self, command: Actuation, now: float) -> None:
        self.transport.write(encode_actuation(command, now))

    def close(self) -> None:
        self.transport.close()


__all__ = [
    "SerialActuatorSink",
    "SerialSensorSource",
    "decode_sensor_packet",
    "encode_actuation",
]
