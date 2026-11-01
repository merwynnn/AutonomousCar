"""Standalone fixed-step simulator with a portable sensor-only boundary."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import numpy as np
from onboard.api import (
    Actuation,
    EncoderSample,
    ImuSample,
    LidarObservation,
    Observation,
    Pose2,
    SensorFrame,
    SteeringSample,
    UltrasonicObservation,
)
from onboard.config import EstimatorTuning, load_estimator_tuning
from onboard.estimator.wheel_imu import WheelImuEstimator

from simulator.config import (
    DEFAULT_ESTIMATOR_PATH,
    DEFAULT_SCENARIO_PATH,
    Realism,
    ScenarioBundle,
    SensorsConfig,
    VehicleConfig,
    load_bundle,
)
from simulator.mujoco_world import MuJoCoWorld
from simulator.sensors.encoders import EncoderModel
from simulator.sensors.imu import ImuModel
from simulator.sensors.lidar import LidarModel, LidarScan
from simulator.sensors.steering import SteeringSensorModel
from simulator.sensors.ultrasonic import UltrasonicModel, UltrasonicReading
from simulator.track import TrackGeometry, build_test_track, project_to_polyline
from simulator.vehicle import VehicleState
from simulator.world import PyBulletWorld
from simulator.world_api import World

DEFAULT_SCENARIO = DEFAULT_SCENARIO_PATH


@dataclass(frozen=True)
class SimulationStatus:
    """Simulator-internal status for logging and tests.

    ``true_pose`` is ground truth and must never be placed in an Observation.
    """

    t: float
    lap: int
    laps: int
    lap_times: tuple[float, ...]
    finished: bool
    progress_m: float
    lateral_m: float
    true_pose: Pose2
    odometry_pose: Pose2
    estimator_source: str = "wheel_imu"
    slip_detected: bool = False
    gyro_bias_rad_s: float = 0.0


class Simulation:
    """Fixed-step simulator implementing the portable ``RobotAPI`` boundary.

    ``backend="pybullet"`` is the legacy world. ``backend="mujoco"`` uses the
    contact-based MuJoCo digital twin. Both return the same ``Observation`` and
    expose ground truth only through ``true_state()``.

    Raw encoder/IMU/steering measurements are produced by simulator sensor
    models and consumed by the same ``WheelImuEstimator`` that can run on the
    Raspberry Pi.
    """

    def __init__(
        self,
        scenario_path: str | Path | None = None,
        gui: bool = False,
        lidar_subrays: int | None = None,
        seed: int | None = None,
        realism: Realism | None = None,
        vehicle: VehicleConfig | None = None,
        sensors: SensorsConfig | None = None,
        backend: Literal["pybullet", "mujoco"] = "mujoco",
        track: TrackGeometry | None = None,
        estimator_config: EstimatorTuning | None = None,
    ) -> None:
        self.bundle: ScenarioBundle = load_bundle(scenario_path or DEFAULT_SCENARIO)
        self.scenario = self.bundle.scenario
        if seed is not None:
            self.scenario.seed = seed
        if realism is not None:
            self.scenario.realism = realism
        # Programmatic overrides keep the dashboard/CLI path identical while
        # letting code build a car without editing YAML.
        self.vehicle = vehicle if vehicle is not None else self.bundle.vehicle
        self.sensors = sensors if sensors is not None else self.bundle.sensors
        self.backend = backend
        self.track: TrackGeometry = track if track is not None else build_test_track(width_m=0.9)
        self.gui = gui
        self.realism_scale = {"ideal": 0.0, "realistic": 1.0, "harsh": 2.0}[
            self.scenario.realism
        ]

        cfg = self.scenario.simulation
        self.physics_dt = 1.0 / cfg.physics_hz
        self.substeps = max(1, round(cfg.physics_hz / cfg.control_hz))
        self.control_dt = self.substeps * self.physics_dt
        self.scan_every = max(1, round(1.0 / (self.sensors.lidar.rate_hz * self.control_dt)))
        self.sonar_every = max(
            1, round(1.0 / (self.sensors.ultrasonic.rate_hz * self.control_dt))
        )
        self._imu_period = 1.0 / self.sensors.imu.rate_hz
        self._encoder_period = 1.0 / self.sensors.encoders.rate_hz
        self._steering_period = 1.0 / self.sensors.steering.rate_hz

        lidar_config = self.sensors.lidar.model_copy(deep=True)
        if lidar_subrays is not None:
            lidar_config.divergence["subrays"] = max(1, lidar_subrays)
        self._lidar_config = lidar_config

        self.world: World
        if backend == "mujoco":
            self.world = MuJoCoWorld(
                self.track,
                self.vehicle,
                physics_hz=cfg.physics_hz,
                gui=gui,
            )
        else:
            self.world = PyBulletWorld(
                self.track,
                self.vehicle,
                gui=gui,
                physics_hz=cfg.physics_hz,
            )

        tuning = estimator_config
        if tuning is None:
            tuning = (
                load_estimator_tuning(DEFAULT_ESTIMATOR_PATH)
                if DEFAULT_ESTIMATOR_PATH.exists()
                else EstimatorTuning()
            )
        self.estimator = WheelImuEstimator(
            tuning,
            wheel_radius_m=self.vehicle.wheel_radius_m,
            track_width_m=self.vehicle.track_width_m,
            wheelbase_m=self.vehicle.wheelbase_m,
            ticks_per_rev=self.sensors.encoders.ticks_per_rev,
        )
        self.lidar = LidarModel(self._lidar_config, self.world, seed=self.scenario.seed)
        self.ultrasonic = UltrasonicModel(
            self.sensors.ultrasonic, self.world, seed=self.scenario.seed
        )
        self.encoders = EncoderModel(
            self.sensors.encoders,
            self.vehicle.wheel_radius_m,
            seed=self.scenario.seed,
        )
        self.imu = ImuModel(self.sensors.imu, seed=self.scenario.seed)
        self.steering_sensor = SteeringSensorModel(
            self.sensors.steering, seed=self.scenario.seed
        )

        self.sim_time = 0.0
        self.step_count = 0
        self.lap = 1
        self.lap_start_time = 0.0
        self.lap_times: list[float] = []
        self.finished = False
        self.progress_m = 0.0
        self.lateral_m = 0.0
        self.last_progress = 0.0
        self.lap_armed = False
        self.lap_distance = 0.0
        self._latest_scan: LidarScan | None = None
        self._available_scan: LidarScan | None = None
        self._latest_sonars: list[UltrasonicReading] = []
        self._available_sonars: dict[str, UltrasonicReading] = {}
        self._latest_encoders: EncoderSample | None = None
        self._available_encoders: EncoderSample | None = None
        self._latest_imu: ImuSample | None = None
        self._available_imu: ImuSample | None = None
        self._latest_steering: SteeringSample | None = None
        self._available_steering: SteeringSample | None = None
        self._recent_encoders: list[EncoderSample] = []
        self._recent_imu: list[ImuSample] = []
        self._recent_steering: list[SteeringSample] = []
        self._next_imu_time = 0.0
        self._next_encoder_time = 0.0
        self._next_steering_time = 0.0
        self._observation: Observation | None = None
        self.reset()

    # ------------------------------------------------------------------ reset
    def reset(self) -> Observation:
        start = self.track.sample(self.track.start_s)
        self.world.reset(start)
        self.lidar = LidarModel(self._lidar_config, self.world, seed=self.scenario.seed)
        self.ultrasonic = UltrasonicModel(
            self.sensors.ultrasonic, self.world, seed=self.scenario.seed
        )
        self.encoders = EncoderModel(
            self.sensors.encoders,
            self.vehicle.wheel_radius_m,
            seed=self.scenario.seed,
        )
        self.imu = ImuModel(self.sensors.imu, seed=self.scenario.seed)
        self.steering_sensor = SteeringSensorModel(
            self.sensors.steering, seed=self.scenario.seed
        )

        self.sim_time = 0.0
        self.step_count = 0
        self.lap = 1
        self.lap_start_time = 0.0
        self.lap_times = []
        self.finished = False
        self.progress_m = 0.0
        self.lateral_m = 0.0
        self.last_progress = 0.0
        self.lap_armed = False
        self.lap_distance = 0.0
        self._latest_scan = None
        self._available_scan = None
        self._latest_sonars = []
        self._available_sonars = {}
        self._latest_encoders = None
        self._available_encoders = None
        self._latest_imu = None
        self._available_imu = None
        self._latest_steering = None
        self._available_steering = None
        self._recent_encoders = []
        self._recent_imu = []
        self._recent_steering = []
        self._next_imu_time = 0.0
        self._next_encoder_time = 0.0
        self._next_steering_time = 0.0

        self.estimator.reset(Pose2(start.x, start.y, start.theta))
        self._sample_scan()
        self._sample_sonars()
        state = self.world.state()
        self._sample_aux_sensors(state, 0.0)
        self._next_imu_time = self._imu_period
        self._next_encoder_time = self._encoder_period
        self._next_steering_time = self._steering_period
        self._refresh_available(0.0)
        self._observation = self._make_observation(update_estimator=False)
        return self._observation

    # --------------------------------------------------------------- sensors
    def _sample_scan(self) -> None:
        if self.lidar.should_drop_scan(self.realism_scale):
            return
        state = self.world.state()
        self._latest_scan = self.lidar.scan(
            state, self.sim_time, self.realism_scale
        )

    def _sample_sonars(self) -> None:
        state = self.world.state()
        self._latest_sonars = self.ultrasonic.sample(
            state, self.sim_time, self.realism_scale
        )

    def _sample_aux_sensors(self, state: VehicleState, now: float) -> None:
        if now + 1e-9 >= self._next_imu_time:
            sample_imu = self.imu.sample(
                state, now, realism_scale=self.realism_scale
            )
            self._latest_imu = sample_imu
            self._recent_imu.append(sample_imu)
            self._recent_imu = self._recent_imu[-8:]
            while self._next_imu_time <= now + 1e-9:
                self._next_imu_time += self._imu_period
        if now + 1e-9 >= self._next_encoder_time:
            sample_encoder = self.encoders.sample(
                state, now, self.realism_scale
            )
            self._latest_encoders = sample_encoder
            self._recent_encoders.append(sample_encoder)
            self._recent_encoders = self._recent_encoders[-8:]
            while self._next_encoder_time <= now + 1e-9:
                self._next_encoder_time += self._encoder_period
        if now + 1e-9 >= self._next_steering_time:
            sample_steering = self.steering_sensor.sample(
                state, now, self.realism_scale
            )
            self._latest_steering = sample_steering
            self._recent_steering.append(sample_steering)
            self._recent_steering = self._recent_steering[-8:]
            while self._next_steering_time <= now + 1e-9:
                self._next_steering_time += self._steering_period

    def _refresh_available(self, now: float) -> None:
        if (
            self._latest_scan is not None
            and self._latest_scan.t_available <= now + 1e-9
        ):
            self._available_scan = self._latest_scan
        for reading in self._latest_sonars:
            if reading.t_available <= now + 1e-9:
                self._available_sonars[reading.name] = reading
        for sample in reversed(self._recent_encoders):
            if sample.t_available <= now + 1e-9:
                self._available_encoders = sample
                break
        for imu_sample in reversed(self._recent_imu):
            if imu_sample.t_available <= now + 1e-9:
                self._available_imu = imu_sample
                break
        for steering_sample in reversed(self._recent_steering):
            if steering_sample.t_available <= now + 1e-9:
                self._available_steering = steering_sample
                break

    def _sensor_frame(self) -> SensorFrame:
        scan = self._available_scan
        lidar = None
        if scan is not None:
            lidar = LidarObservation(
                angles_rad=np.array(scan.angles, copy=True),
                ranges_m=np.array(scan.ranges, copy=True),
                intensities=np.array(scan.intensities, copy=True),
                valid=np.array(scan.valid, copy=True),
                t_measure=scan.t_measure,
                t_available=scan.t_available,
                frame_id=scan.frame_id,
            )

        sonars = tuple(
            UltrasonicObservation(
                name=reading.name,
                range_m=reading.range_m,
                valid=reading.valid,
                t_measure=reading.t_measure,
                t_available=reading.t_available,
            )
            for reading in self._available_sonars.values()
        )
        return SensorFrame(
            lidar=lidar,
            ultrasonic=sonars,
            imu=self._available_imu,
            encoders=self._available_encoders,
            steering=self._available_steering,
        )

    def _make_observation(self, *, update_estimator: bool) -> Observation:
        sensors = self._sensor_frame()
        state = (
            self.estimator.update(sensors, self.sim_time)
            if update_estimator
            else self.estimator.state
        )
        return Observation(
            t=self.sim_time,
            dt=self.control_dt,
            sensors=sensors,
            state=state,
        )

    # ------------------------------------------------------------------- step
    def step(self, command: Actuation) -> Observation:
        if self.finished:
            assert self._observation is not None
            return self._observation

        self.world.set_actuation(command)
        for substep in range(self.substeps):
            self.world.step()
            now = self.sim_time + (substep + 1) * self.physics_dt
            if (
                now + 1e-9 >= self._next_imu_time
                or now + 1e-9 >= self._next_encoder_time
                or now + 1e-9 >= self._next_steering_time
            ):
                self._sample_aux_sensors(self.world.state(), now)

        self.sim_time += self.control_dt
        self.step_count += 1
        if self.step_count % self.scan_every == 0:
            self._sample_scan()
        if self.step_count % self.sonar_every == 0:
            self._sample_sonars()
        self._refresh_available(self.sim_time)
        state = self.world.state()
        self._update_progress(state)
        self._observation = self._make_observation(update_estimator=True)
        return self._observation

    def _update_progress(self, state: VehicleState) -> None:
        s, lateral, _ = project_to_polyline(self.track.centerline, (state.x, state.y))
        progress = (s - self.track.start_s) % self.track.length
        if progress > self.track.length - 0.2:
            progress = 0.0
        self.progress_m = progress
        self.lateral_m = lateral

        self.lap_distance += max(0.0, state.speed) * self.control_dt
        if self.lap_distance > 0.5 * self.track.length:
            self.lap_armed = True
        if (
            self.lap_armed
            and self.last_progress > 0.8 * self.track.length
            and self.progress_m < 0.2 * self.track.length
        ):
            lap_time = self.sim_time - self.lap_start_time
            self.lap_times.append(lap_time)
            self.lap_start_time = self.sim_time
            self.lap += 1
            self.lap_armed = False
            self.lap_distance = 0.0
            if self.lap > self.scenario.race.laps:
                self.finished = True
        self.last_progress = self.progress_m

    # ---------------------------------------------------------------- debug
    @property
    def observation(self) -> Observation:
        if self._observation is None:
            self._observation = self._make_observation(update_estimator=False)
        return self._observation

    @property
    def odometry(self) -> WheelImuEstimator:
        """Deprecated alias for the shared estimator."""

        return self.estimator

    def true_state(self) -> VehicleState:
        """Ground truth for timing/logging/tests only. Never give this to a controller."""

        return self.world.state()

    def status(self) -> SimulationStatus:
        state = self.world.state()
        estimate = self.observation.state
        pose = estimate.pose
        return SimulationStatus(
            t=self.sim_time,
            lap=min(self.lap, self.scenario.race.laps),
            laps=self.scenario.race.laps,
            lap_times=tuple(self.lap_times),
            finished=self.finished,
            progress_m=self.progress_m,
            lateral_m=self.lateral_m,
            true_pose=Pose2(state.x, state.y, state.yaw),
            odometry_pose=Pose2(pose.x, pose.y, pose.theta),
            estimator_source=estimate.source,
            slip_detected=estimate.slip_detected,
            gyro_bias_rad_s=estimate.gyro_bias_rad_s,
        )

    def close(self) -> None:
        self.world.close()

    def __enter__(self) -> Simulation:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()
