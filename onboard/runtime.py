"""Portable control loop for the simulator and the Raspberry Pi.

``SensorRobot`` composes a sensor source, an actuator sink and the shared state
estimator into the same ``RobotAPI`` the simulator implements. Hardware drivers
only have to produce ``SensorFrame`` objects and apply ``Actuation`` commands;
the estimator and controller code stay identical to simulation.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Protocol, runtime_checkable

from onboard.api import (
    Actuation,
    Controller,
    Observation,
    Pose2,
    RobotAPI,
    RobotLimits,
    SensorFrame,
    StateEstimate,
)
from onboard.estimator.base import StateEstimator


@runtime_checkable
class SensorSource(Protocol):
    """Reads the latest available raw sensor frame."""

    def read(self, now: float) -> SensorFrame:
        """Return measurements available at ``now``."""

    def close(self) -> None:
        """Release the sensor bus."""


@runtime_checkable
class ActuatorSink(Protocol):
    """Applies one control command to the car."""

    def apply(self, command: Actuation, now: float) -> None:
        """Send a command to the low-level board."""

    def close(self) -> None:
        """Release the actuator bus."""


class SensorRobot:
    """``RobotAPI`` implementation for a real sensor/actuator stack."""

    def __init__(
        self,
        source: SensorSource,
        sink: ActuatorSink,
        estimator: StateEstimator,
        *,
        initial_pose: Pose2 | None = None,
        control_dt: float = 1.0 / 60.0,
        clock: Callable[[], float] | None = None,
        limits: RobotLimits | None = None,
    ) -> None:
        if control_dt <= 0.0:
            raise ValueError("control_dt must be positive")
        self.source = source
        self.sink = sink
        self.estimator = estimator
        self.initial_pose = initial_pose or Pose2(0.0, 0.0, 0.0)
        self.control_dt = control_dt
        self.clock = clock
        self.limits = limits or RobotLimits()
        self._now = 0.0
        self._observation: Observation | None = None

    @property
    def observation(self) -> Observation | None:
        return self._observation

    def reset(self) -> Observation:
        self._now = self.clock() if self.clock is not None else 0.0
        self.estimator.reset(self.initial_pose)
        self.sink.apply(Actuation(), self._now)
        sensors = self.source.read(self._now)
        state = self.estimator.update(sensors, self._now)
        self._observation = Observation(
            t=self._now,
            dt=self.control_dt,
            sensors=sensors,
            state=state,
            limits=self.limits,
        )
        return self._observation

    def step(self, command: Actuation) -> Observation:
        self._now = (
            self.clock() if self.clock is not None else self._now + self.control_dt
        )
        self.sink.apply(command, self._now)
        sensors = self.source.read(self._now)
        state = self.estimator.update(sensors, self._now)
        self._observation = Observation(
            t=self._now,
            dt=self.control_dt,
            sensors=sensors,
            state=state,
            limits=self.limits,
        )
        return self._observation

    def close(self) -> None:
        self.source.close()
        self.sink.close()


class ControllerRuntime:
    """Run a controller against any ``RobotAPI`` implementation."""

    def __init__(
        self,
        robot: RobotAPI,
        controller: Controller,
    ) -> None:
        self.robot = robot
        self.controller = controller
        self.observation: Observation | None = None

    def reset(self) -> Observation:
        self.observation = self.robot.reset()
        self.controller.reset()
        return self.observation

    def step(self) -> Observation:
        if self.observation is None:
            self.reset()
        assert self.observation is not None
        command = self.controller.update(self.observation)
        self.observation = self.robot.step(command)
        return self.observation

    def run(
        self,
        steps: int,
        stop: Callable[[Observation], bool] | None = None,
    ) -> Observation:
        if steps <= 0:
            raise ValueError("steps must be positive")
        observation = self.reset()
        for _ in range(steps):
            if stop is not None and stop(observation):
                break
            observation = self.step()
        return observation

    def close(self) -> None:
        self.robot.close()


def make_observation(
    sensors: SensorFrame,
    state: StateEstimate,
    *,
    t: float,
    dt: float,
    limits: RobotLimits | None = None,
) -> Observation:
    """Small helper for hardware loops that build observations manually."""

    return Observation(
        t=t,
        dt=dt,
        sensors=sensors,
        state=state,
        limits=limits or RobotLimits(),
    )


__all__ = [
    "ActuatorSink",
    "ControllerRuntime",
    "SensorRobot",
    "SensorSource",
    "make_observation",
]
