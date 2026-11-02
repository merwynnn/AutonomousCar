"""Build and run a car entirely from Python.

This is the recommended starting point when you do not want to edit YAML.
Every value below is part of the same VehicleConfig that generates the MJCF,
drives MuJoCo and feeds the dashboard car studio.

Run from the repository root:

    uv run python -m examples.custom_car
"""

from __future__ import annotations

from pathlib import Path

from algorithms.gap_follow import FurthestGapController
from simulator.config import BodyPart, load_sensors, load_vehicle
from simulator.simulation import Simulation

ROOT = Path(__file__).resolve().parents[1]


def build_car():
    """Return a modified vehicle and sensor set without touching YAML."""

    vehicle = load_vehicle(ROOT / "configs" / "vehicle.yaml")
    sensors = load_sensors(ROOT / "configs" / "sensors.yaml")

    # Vehicle dynamics and geometry.
    vehicle.name = "wide_track_car"
    vehicle.description = "Code-defined example: wider track, extra mass, taller lidar."
    vehicle.mass_kg = 2.35
    vehicle.wheelbase_m = 0.27
    vehicle.track_width_m = 0.19
    vehicle.wheel_width_m = 0.032
    vehicle.wheel_radius_m = 0.036
    vehicle.center_of_mass_m = (0.005, 0.0, 0.04)
    vehicle.tires.friction_front = 1.15
    vehicle.tires.friction_rear = 1.05
    vehicle.motor.max_torque_nm = 0.72
    vehicle.steering.max_angle_deg = 28.0

    # Body structure. Same data generates the MJCF collision/visual geometry and
    # the car studio in the dashboard.
    vehicle.structure = [
        BodyPart(
            name="lower_deck",
            shape="box",
            position_m=(0.0, 0.0, -0.04),
            size_m=(0.34, 0.16, 0.018),
            color="#22272d",
        ),
        BodyPart(
            name="main_body",
            shape="box",
            position_m=(0.0, 0.0, 0.006),
            size_m=(0.39, 0.19, 0.052),
            color="#2f6f8f",
        ),
        BodyPart(
            name="cabin",
            shape="box",
            position_m=(-0.04, 0.0, 0.047),
            size_m=(0.16, 0.14, 0.036),
            color="#161b20",
            collision=False,
        ),
        BodyPart(
            name="nose",
            shape="box",
            position_m=(0.183, 0.0, 0.006),
            size_m=(0.04, 0.15, 0.026),
            color="#d9dde1",
            collision=False,
        ),
    ]

    # Sensors are in the body frame: x forward, y left, z up.
    sensors.lidar.beams = 540
    sensors.lidar.max_range_m = 10.0
    sensors.lidar.mount.position_m = (0.105, 0.0, 0.06)
    sensors.lidar.mount.rpy_deg = (0.0, 0.0, 0.0)
    sensors.ultrasonic.sensors[0].mount.position_m = (0.185, 0.075, 0.0)
    sensors.ultrasonic.sensors[1].mount.position_m = (0.185, -0.075, 0.0)
    sensors.imu.mount.position_m = (0.0, 0.0, 0.015)
    return vehicle, sensors


def main() -> None:
    vehicle, sensors = build_car()
    print(vehicle.name)
    print(f"  mass:       {vehicle.mass_kg:.2f} kg")
    print(f"  wheelbase:  {vehicle.wheelbase_m:.3f} m")
    print(f"  structure:  {len(vehicle.structure)} parts")
    print(f"  sensors:    {len(sensors.ultrasonic.sensors) + 2} mounts")

    with Simulation(vehicle=vehicle, sensors=sensors, lidar_subrays=1) as simulation:
        controller = FurthestGapController()
        observation = simulation.reset()
        while not simulation.finished and simulation.sim_time < 10.0:
            observation = simulation.step(controller.update(observation, speed_limit=1.8))
        status = simulation.status()
        print(f"  result:     {status.lap} lap(s), {status.t:.1f} s, {status.progress_m:.1f} m")


if __name__ == "__main__":
    main()
