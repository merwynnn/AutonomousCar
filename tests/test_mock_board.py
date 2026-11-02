from __future__ import annotations

from pathlib import Path

from algorithms.template import TemplateController
from onboard.api import Actuation, RobotLimits
from onboard.config import load_onboard_config
from onboard.drivers.mock_board import MockBoard
from onboard.drivers.serial_link import SerialActuatorSink, SerialSensorSource
from onboard.estimator import WheelImuEstimator
from onboard.runtime import ControllerRuntime, SensorRobot

ROOT = Path(__file__).resolve().parents[1]


def _robot() -> SensorRobot:
    config = load_onboard_config(
        ROOT / "configs" / "vehicle_awake2027.yaml",
        ROOT / "configs" / "sensors_awake2027.yaml",
        ROOT / "configs" / "estimator.yaml",
    )
    board = MockBoard(
        hardware=config.hardware,
        wheelbase_m=config.wheelbase_m,
        wheel_radius_m=config.wheel_radius_m,
    )
    source = SerialSensorSource(board)
    sink = SerialActuatorSink(board, hardware=config.hardware)
    estimator = WheelImuEstimator(
        config.tuning,
        wheel_radius_m=config.wheel_radius_m,
        track_width_m=config.track_width_m,
        wheelbase_m=config.wheelbase_m,
        ticks_per_rev=config.ticks_per_rev,
    )
    return SensorRobot(
        source,
        sink,
        estimator,
        control_dt=1.0 / 60.0,
        limits=RobotLimits(
            max_steering_rad=0.49,
            max_steering_rate_rad_s=1.57,
            max_speed_mps=config.hardware.drive.max_speed_mps,
        ),
    )


def test_mock_board_runs_the_serial_robot_loop() -> None:
    robot = _robot()
    try:
        observation = robot.reset()
        assert observation.motor is not None
        assert observation.encoders is not None
        assert observation.encoders.t_available <= observation.t + 1e-9
        assert observation.steering is not None

        for _ in range(20):
            observation = robot.step(Actuation(throttle=0.5, steering=0.1))
        assert observation.motor is not None
        assert observation.motor.t_available <= observation.t + 1e-9
        assert observation.motor.erpm != 0.0
        assert observation.state.speed_mps > 0.0
    finally:
        robot.close()


def test_controller_runtime_runs_against_mock_board() -> None:
    robot = _robot()
    runtime = ControllerRuntime(robot, TemplateController())
    try:
        observation = runtime.run(3)
        assert observation.motor is not None
    finally:
        runtime.close()
