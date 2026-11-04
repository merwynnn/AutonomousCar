from __future__ import annotations

from pathlib import Path

import numpy as np
from algorithms.interface import Actuation, Observation
from algorithms.template import TemplateController
from simulator.cli import load_controller
from simulator.simulation import Simulation

GENERIC_SCENARIO = (
    Path(__file__).resolve().parents[1] / "configs" / "sim" / "scenario.yaml"
)


def test_observation_is_sensor_only_and_realtime_filtered() -> None:
    with Simulation(GENERIC_SCENARIO, realism="ideal", lidar_subrays=1) as simulation:
        observation = simulation.reset()
        assert isinstance(observation, Observation)
        assert not hasattr(observation, "true_pose")
        assert observation.lidar is not None
        assert observation.lidar.beam_count == 720
        assert observation.lidar.ranges_m.shape == (720,)
        assert observation.lidar.t_available <= observation.t + 1e-9
        assert len(observation.ultrasonic) == 3
        assert observation.encoders is not None
        assert observation.imu is not None
        assert observation.steering is not None
        assert observation.state.source in {
            "wheel_imu",
            "zero_velocity",
            "initialized",
        }

        for _ in range(12):
            observation = simulation.step(Actuation(throttle=0.3, steering=0.0))
            assert observation.t > 0.0
            if observation.lidar is not None:
                assert observation.lidar.t_available <= observation.t + 1e-9
            for reading in observation.ultrasonic:
                assert reading.t_available <= observation.t + 1e-9

        status = simulation.status()
        assert status.odometry_pose.x == observation.odometry.x
        assert np.isfinite(status.true_pose.x)


def test_template_controller_and_loader() -> None:
    with Simulation(GENERIC_SCENARIO, realism="ideal", lidar_subrays=1) as simulation:
        observation = simulation.reset()
        controller = load_controller("algorithms.template:TemplateController")
        controller.reset()
        command = controller.update(observation)
        assert isinstance(command, Actuation)
        assert command.throttle == 0.0
        assert command.steering == 0.0
        assert isinstance(TemplateController(), TemplateController)


def test_shared_estimator_is_deterministic_for_a_fixed_seed() -> None:
    def run() -> tuple[float, float, float]:
        with Simulation(GENERIC_SCENARIO, realism="realistic", lidar_subrays=1, seed=11) as sim:
            observation = sim.reset()
            for _ in range(60):
                observation = sim.step(Actuation(throttle=0.3, steering=0.0))
            return (
                observation.state.pose.x,
                observation.state.pose.y,
                observation.state.pose.theta,
            )

    first = run()
    second = run()
    assert first == second


def test_physics_step_hook_runs_once_per_substep() -> None:
    calls: list[int] = []
    with Simulation(
        GENERIC_SCENARIO,
        realism="ideal",
        lidar_subrays=1,
        physics_step_hook=lambda _world: calls.append(1),
    ) as simulation:
        simulation.reset()
        simulation.step(Actuation())
        assert len(calls) == simulation.substeps


def test_reset_restores_start_state() -> None:
    with Simulation(GENERIC_SCENARIO, realism="ideal", lidar_subrays=1) as simulation:
        first = simulation.reset()
        for _ in range(30):
            simulation.step(Actuation(throttle=0.4, steering=0.1))
        second = simulation.reset()
        assert second.t == 0.0
        assert simulation.status().lap_times == ()
        assert second.odometry.x == first.odometry.x
        assert second.odometry.y == first.odometry.y
