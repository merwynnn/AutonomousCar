"""Check the portable robot path before driving the real car.

Examples:
    awake-selftest --profile mock
    awake-selftest --profile serial --port /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from onboard.api import Actuation, RobotLimits
from onboard.config import load_onboard_config
from onboard.drivers.mock_board import MockBoard
from onboard.drivers.serial_link import SerialActuatorSink, SerialSensorSource
from onboard.drivers.transport import SerialPortTransport
from onboard.estimator import WheelImuEstimator
from onboard.runtime import SensorRobot


def _build_robot(
    *,
    profile: str,
    port: str | None,
    vehicle: Path,
    sensors: Path,
    estimator: Path,
    hardware: Path | None,
) -> SensorRobot:
    config = load_onboard_config(vehicle, sensors, estimator, hardware)
    if profile == "mock":
        board = MockBoard(
            hardware=config.hardware,
            wheelbase_m=config.wheelbase_m,
            wheel_radius_m=config.wheel_radius_m,
        )
        source = SerialSensorSource(board)
        sink = SerialActuatorSink(board, hardware=config.hardware)
    else:
        if port is None:
            raise ValueError("--port is required for --profile serial")
        transport = SerialPortTransport(port)
        source = SerialSensorSource(transport)
        sink = SerialActuatorSink(transport, hardware=config.hardware)

    estimator_impl = WheelImuEstimator(
        config.tuning,
        wheel_radius_m=config.wheel_radius_m,
        track_width_m=config.track_width_m,
        wheelbase_m=config.wheelbase_m,
        ticks_per_rev=config.ticks_per_rev,
    )
    limits = RobotLimits(
        max_steering_rad=config.max_steering_rad,
        max_steering_rate_rad_s=config.max_steering_rate_rad_s,
        max_speed_mps=config.hardware.drive.max_speed_mps,
    )
    return SensorRobot(
        source,
        sink,
        estimator_impl,
        control_dt=1.0 / 60.0,
        limits=limits,
    )


def _last_steering(robot: SensorRobot, command: float, steps: int = 20) -> float:
    observation = robot.observation
    if observation is None:
        observation = robot.reset()
    for _ in range(steps):
        observation = robot.step(Actuation(throttle=0.0, steering=command))
    assert observation.steering is not None
    return observation.steering.angle_rad


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="AWAKE portable robot self-test")
    parser.add_argument("--profile", choices=("mock", "serial"), default="mock")
    parser.add_argument("--port", default=None)
    parser.add_argument("--vehicle", type=Path, default=Path("configs/vehicle_awake2027.yaml"))
    parser.add_argument("--sensors", type=Path, default=Path("configs/sensors_awake2027.yaml"))
    parser.add_argument("--estimator", type=Path, default=Path("configs/estimator.yaml"))
    parser.add_argument("--hardware", type=Path, default=None)
    args = parser.parse_args(argv)

    failures: list[str] = []
    robot = _build_robot(
        profile=args.profile,
        port=args.port,
        vehicle=args.vehicle,
        sensors=args.sensors,
        estimator=args.estimator,
        hardware=args.hardware,
    )
    try:
        observation = robot.reset()
        for name in ("encoders", "imu", "steering"):
            if getattr(observation.sensors, name) is None:
                failures.append(f"missing {name}")
        if observation.motor is None:
            failures.append("missing motor telemetry")
        if not observation.state.valid:
            failures.append("estimator did not produce a valid state")

        left = _last_steering(robot, 0.12)
        right = _last_steering(robot, -0.12)
        if left <= 0.0:
            failures.append(f"positive steering command produced {left:.3f} rad")
        if right >= 0.0:
            failures.append(f"negative steering command produced {right:.3f} rad")

        print(f"profile:   {args.profile}")
        print(f"limits:    {observation.limits}")
        print(f"steering:  left={left:.3f} rad right={right:.3f} rad")
        print(f"motor:     {observation.motor}")
        if failures:
            print("FAIL: " + "; ".join(failures))
            return 1
        print("PASS")
        return 0
    finally:
        robot.close()


if __name__ == "__main__":
    sys.exit(main())
