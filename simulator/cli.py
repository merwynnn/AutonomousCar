"""Headless simulator runner.

Examples:
    python -m simulator.cli
    python -m simulator.cli --controller algorithms.template:TemplateController
    python -m simulator.cli --controller my_team.controller:make_controller --laps 5
"""

from __future__ import annotations

import argparse
import importlib
import time
from pathlib import Path
from typing import Any

from onboard.api import Controller

from simulator.config import Realism, SensorsConfig, VehicleConfig, load_sensors, load_vehicle
from simulator.simulation import DEFAULT_SCENARIO, Simulation


def load_controller(spec: str) -> Controller:
    """Load ``module:ClassOrFactory`` without importing simulator internals."""

    module_name, separator, attribute = spec.partition(":")
    if not separator or not module_name or not attribute:
        raise ValueError(
            f"controller must be 'module:ClassOrFactory', got {spec!r}"
        )
    module = importlib.import_module(module_name)
    target: Any = getattr(module, attribute)
    controller = target() if callable(target) else target
    if not hasattr(controller, "update") or not hasattr(controller, "reset"):
        raise TypeError(
            f"{spec} must provide reset() and update(observation) methods"
        )
    return controller


def run(
    scenario: Path,
    controller_spec: str,
    laps: int,
    seed: int | None,
    realism: Realism | None,
    lidar_subrays: int,
    realtime: bool,
    max_time: float | None,
    vehicle: VehicleConfig | None = None,
    sensors: SensorsConfig | None = None,
) -> int:
    controller = load_controller(controller_spec)
    with Simulation(
        scenario,
        lidar_subrays=lidar_subrays,
        seed=seed,
        realism=realism,
        vehicle=vehicle,
        sensors=sensors,
    ) as simulation:
        simulation.scenario.race.laps = laps
        observation = simulation.reset()
        controller.reset()
        limit = max_time if max_time is not None else simulation.scenario.simulation.max_time_s
        next_tick = time.perf_counter()

        while not simulation.finished and simulation.sim_time < limit:
            command = controller.update(observation)
            observation = simulation.step(command)
            if realtime:
                next_tick += simulation.control_dt
                delay = next_tick - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.perf_counter()

        status = simulation.status()
        state = simulation.true_state()
        drift = (
            (status.true_pose.x - status.odometry_pose.x) ** 2
            + (status.true_pose.y - status.odometry_pose.y) ** 2
        ) ** 0.5

        print(f"scenario:   {simulation.scenario.name}")
        print(f"controller: {controller_spec}")
        print(f"sim time:   {status.t:.2f} s")
        print(f"laps:       {len(status.lap_times)}/{status.laps}")
        print(f"status:     {'OK' if simulation.finished else 'DNF'}")
        if status.lap_times:
            print(f"best lap:   {min(status.lap_times):.2f} s")
            print(f"total:      {sum(status.lap_times):.2f} s")
            print("lap times:  " + ", ".join(f"{t:.2f}" for t in status.lap_times))
        print(f"true pose:  ({state.x:.2f}, {state.y:.2f}, {state.yaw:.2f})")
        print(f"odom drift: {drift:.3f} m (debug only)")
        return 0 if simulation.finished else 1


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        type=Path,
        default=DEFAULT_SCENARIO,
        help="scenario YAML (vehicle/sensor paths are relative to it)",
    )
    parser.add_argument(
        "--vehicle",
        type=Path,
        default=None,
        help="override the scenario vehicle YAML",
    )
    parser.add_argument(
        "--sensors",
        type=Path,
        default=None,
        help="override the scenario sensor YAML",
    )
    parser.add_argument(
        "--controller",
        default="algorithms.template:TemplateController",
        help="module:ClassOrFactory implementing reset() and update()",
    )
    parser.add_argument("--laps", type=int, default=1)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument(
        "--realism",
        choices=("ideal", "realistic", "harsh"),
        default=None,
    )
    parser.add_argument(
        "--lidar-subrays",
        type=int,
        default=3,
        help="1 is faster, 3 is the configured mixed-pixel fidelity",
    )
    parser.add_argument(
        "--realtime",
        action="store_true",
        help="pace the loop in real time (slower than headless)",
    )
    parser.add_argument(
        "--max-time",
        type=float,
        default=None,
        help="simulation-time limit in seconds",
    )
    args = parser.parse_args()
    raise SystemExit(
        run(
            scenario=args.scenario,
            controller_spec=args.controller,
            laps=args.laps,
            seed=args.seed,
            realism=args.realism,
            lidar_subrays=args.lidar_subrays,
            realtime=args.realtime,
            max_time=args.max_time,
            vehicle=load_vehicle(args.vehicle) if args.vehicle else None,
            sensors=load_sensors(args.sensors) if args.sensors else None,
        )
    )


if __name__ == "__main__":
    main()
