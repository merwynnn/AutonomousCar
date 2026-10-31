"""Command-line interface for the simulator."""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

import typer
from rich.console import Console
from rich.table import Table

from awake_sim import __version__
from awake_sim.config import load_scenario
from awake_sim.control.racing_line import build_racing_line
from awake_sim.server.demo import DemoSimulation

app = typer.Typer(help="AWAKE Challenge 2027 simulator", no_args_is_help=True)
console = Console()
ROOT = Path(__file__).resolve().parents[2]


def _run(cmd: list[str], cwd: Path | None = None) -> None:
    console.print(f"[dim]$ {' '.join(cmd)}[/dim]")
    subprocess.run(cmd, cwd=cwd or ROOT, check=True)


@app.command()
def version() -> None:
    """Print the simulator version."""
    console.print(f"awake-sim {__version__}")


@app.command("build-cpp")
def build_cpp(verbose: bool = typer.Option(False, "--verbose", "-v")) -> None:
    """Configure and build the C++ SLAM extension."""

    pybind11_dir = subprocess.check_output(
        [sys.executable, "-m", "pybind11", "--cmakedir"], text=True
    ).strip()
    build_dir = ROOT / "build"
    cmake_args = [
        "cmake",
        "-S",
        str(ROOT),
        "-B",
        str(build_dir),
        "-G",
        "Ninja",
        "-DCMAKE_BUILD_TYPE=Release",
        f"-Dpybind11_DIR={pybind11_dir}",
    ]
    _run(cmake_args)
    _run(["cmake", "--build", str(build_dir), "--target", "_awake_slam"])
    if verbose:
        console.print(f"[green]C++ extension built in {build_dir}[/green]")


@app.command("test-cpp")
def test_cpp() -> None:
    """Build and run the C++ tests."""

    pybind11_dir = subprocess.check_output(
        [sys.executable, "-m", "pybind11", "--cmakedir"], text=True
    ).strip()
    build_dir = ROOT / "build"
    _run(
        [
            "cmake",
            "-S",
            str(ROOT),
            "-B",
            str(build_dir),
            "-G",
            "Ninja",
            "-DCMAKE_BUILD_TYPE=Release",
            f"-Dpybind11_DIR={pybind11_dir}",
        ]
    )
    _run(
        [
            "cmake",
            "--build",
            str(build_dir),
            "--target",
            "awake_slam_tests",
            "awake_pose_graph_tests",
        ]
    )
    _run(["ctest", "--test-dir", str(build_dir), "--output-on-failure"])


@app.command()
def run(
    scenario: Path = typer.Option(
        ROOT / "configs" / "scenarios" / "test_track_baseline.yaml",
        help="Scenario YAML",
    ),
    duration: float = typer.Option(60.0, help="Simulated seconds to run"),
    speed: float = typer.Option(2.0, help="Demo centreline speed (m/s)"),
) -> None:
    """Run a headless demo and print lap results (M0 placeholder)."""

    cfg = load_scenario(scenario)
    sim = DemoSimulation(scenario)
    sim.speed = speed
    sim.paused = False
    dt = 1.0 / 240.0
    steps = int(duration / dt)
    for _ in range(steps):
        sim.step(dt)
        if sim.finished:
            break
    table = Table(title=f"Demo run — {cfg.name}")
    table.add_column("Metric")
    table.add_column("Value", justify="right")
    table.add_row("Laps completed", str(len(sim.lap_times)))
    table.add_row("Lap times (s)", ", ".join(f"{t:.2f}" for t in sim.lap_times) or "—")
    table.add_row("Best lap (s)", f"{min(sim.lap_times):.2f}" if sim.lap_times else "—")
    table.add_row("Total (s)", f"{sum(sim.lap_times):.2f}")
    table.add_row("Track length (m)", f"{sim.track.length:.2f}")
    console.print(table)


def _run_race(
    controller: str,
    laps: int,
    scenario: Path,
    speed: float = 2.4,
) -> tuple[list[float], float]:
    from awake_sim.server.pybullet_sim import PyBulletSimulation

    sim = PyBulletSimulation(
        scenario,
        enable_lidar=controller in {"wall_follow", "gap_follow"},
        enable_slam=False,
    )
    sim.scenario.race.laps = laps
    sim.auto_controller = controller
    sim.speed_limit = speed
    sim.pure_pursuit.speed_limit = speed
    sim.controller.base_speed = speed
    sim.paused = False
    started = time.time()
    while not sim.finished and sim.sim_time < 240 and time.time() - started < 220:
        sim.step_once()
    sim.world.close()
    return list(sim.lap_times), time.time() - started


@app.command()
def race(
    controller: str = typer.Option(
        "pure_pursuit", help="wall_follow, pure_pursuit or gap_follow"
    ),
    laps: int = typer.Option(5, min=1, max=5),
    speed: float = typer.Option(2.4, min=0.5, max=6.0, help="Speed limit in m/s"),
    scenario: Path = typer.Option(
        ROOT / "configs" / "scenarios" / "test_track_baseline.yaml",
    ),
) -> None:
    """Run a headless race with a selected controller."""

    if controller not in {"wall_follow", "pure_pursuit", "gap_follow"}:
        console.print(
            "[red]controller must be wall_follow, pure_pursuit or gap_follow[/red]"
        )
        raise typer.Exit(code=2)
    lap_times, wall = _run_race(controller, laps, scenario, speed)
    table = Table(title=f"{controller} — {laps} laps at {speed:.1f} m/s")
    table.add_column("Lap")
    table.add_column("Time (s)", justify="right")
    for index, lap_time in enumerate(lap_times, start=1):
        table.add_row(str(index), f"{lap_time:.2f}")
    if lap_times:
        table.add_row("Best", f"{min(lap_times):.2f}")
        table.add_row("Total", f"{sum(lap_times):.2f}")
    table.add_row("Wall time", f"{wall:.1f}")
    console.print(table)


@app.command()
def compare(
    laps: int = typer.Option(5, min=1, max=5),
    speed: float = typer.Option(2.4, min=0.5, max=6.0, help="Speed limit in m/s"),
    scenario: Path = typer.Option(
        ROOT / "configs" / "scenarios" / "test_track_baseline.yaml",
    ),
) -> None:
    """Compare wall-follow, pure pursuit and furthest-gap on the test track."""

    rows: list[tuple[str, str, str, str]] = []
    for controller in ("wall_follow", "pure_pursuit", "gap_follow"):
        lap_times, wall = _run_race(controller, laps, scenario, speed)
        if lap_times:
            rows.append(
                (
                    controller,
                    f"{sum(lap_times):.2f}",
                    f"{min(lap_times):.2f}",
                    f"{wall:.1f}",
                )
            )
        else:
            rows.append((controller, "DNF", "—", f"{wall:.1f}"))
    table = Table(title=f"Controller comparison — {laps} laps at {speed:.1f} m/s")
    table.add_column("Controller")
    table.add_column("Total (s)", justify="right")
    table.add_column("Best (s)", justify="right")
    table.add_column("Wall (s)", justify="right")
    for row in rows:
        table.add_row(*row)
    console.print(table)


@app.command("gap-bench")
def gap_bench(
    laps: int = typer.Option(5, min=1, max=5),
    speed: float = typer.Option(3.2, min=0.5, max=6.0, help="Speed limit in m/s"),
    scenario: Path = typer.Option(
        ROOT / "configs" / "scenarios" / "test_track_baseline.yaml",
    ),
) -> None:
    """Lidar-only furthest-gap benchmark: steer at the widest opening, max speed.

    No map, no localization pose and no ground truth are used by the controller.
    """

    lap_times, wall = _run_race("gap_follow", laps, scenario, speed)
    table = Table(title=f"Furthest-gap benchmark — {laps} laps at {speed:.1f} m/s")
    table.add_column("Metric")
    table.add_column("Value", justify="right")
    table.add_row("Controller", "furthest gap (lidar only)")
    table.add_row("Status", "OK" if len(lap_times) == laps else f"DNF ({len(lap_times)}/{laps})")
    table.add_row("Total (s)", f"{sum(lap_times):.2f}" if lap_times else "—")
    table.add_row("Best lap (s)", f"{min(lap_times):.2f}" if lap_times else "—")
    table.add_row("Lap times (s)", ", ".join(f"{t:.2f}" for t in lap_times) or "—")
    table.add_row("Wall time (s)", f"{wall:.1f}")
    console.print(table)


@app.command("map-and-race")
def map_and_race(
    laps: int = typer.Option(5, min=1, max=5),
    mapping_speed: float = typer.Option(1.2, min=0.5, max=3.0),
    race_speed: float = typer.Option(0.9, min=0.5, max=3.0),
    global_relocalization: bool = typer.Option(
        False,
        "--global-relocalization",
        help="Enable experimental global MCL relocalization",
    ),
) -> None:
    """Map one slow lap, freeze/save the map, then race it with MCL."""

    import numpy as np

    from awake_sim.server.pybullet_sim import PyBulletSimulation

    sim = PyBulletSimulation(enable_slam=True, lidar_subrays=1)
    sim.racing_line = build_racing_line(sim.track, sim.vehicle, lateral_margin=0.35)
    sim.pure_pursuit.line = sim.racing_line
    sim.scenario.race.laps = 1
    sim.auto_controller = "pure_pursuit"
    # Mapping is sensor-only too: MCL localization on the growing SLAM map.
    sim.localization_mode = "mcl"
    sim.slam_mcl_enabled = True
    sim.slam_loop_enabled = False
    sim.speed_limit = min(mapping_speed, 1.2)
    sim.pure_pursuit.speed_scale = 0.5
    sim.pure_pursuit.speed_limit = sim.speed_limit
    sim.controller.base_speed = sim.speed_limit
    assert sim.slam is not None
    # Mapping quality first: stable, no rank dropping for the map-building lap.
    sim.slam.set_mcl_params(
        particles=400,
        rank_keep=1.0,
        softmax=10.0,
        score_stride=3,
        beam_weight=0.0,
        odom_prior_xy=1000.0,
        odom_prior_theta=1000.0,
    )
    sim.slam.set_mcl_enabled(True)
    sim.slam.set_loop_closure_enabled(False)
    sim.paused = False
    started = time.time()
    while not sim.finished and sim.sim_time < 120 and time.time() - started < 120:
        sim.step_once()
    mapping_lap = sim.lap_times[0] if sim.lap_times else float("nan")
    start = sim.track.sample(sim.track.start_s)
    final = sim.slam_pose
    seam = (
        float(np.hypot(final[0] - start.x, final[1] - start.y))
        if final is not None
        else float("nan")
    )
    map_path = sim.save_map(
        ROOT / "data" / "maps" / "test_track_mapping.json",
        metadata={
            "mapping_lap_s": mapping_lap,
            "seam_error_m": seam,
            "mapping_speed_mps": mapping_speed,
            "lidar_subrays": 3,
        },
    )

    # Race on the frozen map with MCL.
    sim.scenario.race.laps = laps
    sim.reset(keep_map=True)
    sim.localization_mode = "mcl"
    sim.slam_mcl_enabled = True
    sim.slam.set_mcl_params(
        particles=400,
        rank_keep=0.99,
        softmax=10.0,
        score_stride=3,
        beam_weight=0.0,
        odom_prior_xy=1000.0,
        odom_prior_theta=1000.0,
    )
    sim.slam.set_mcl_enabled(True)
    sim.slam_global_relocalization_enabled = global_relocalization
    sim.slam.set_global_relocalization_enabled(global_relocalization)
    sim.speed_limit = race_speed
    sim.pure_pursuit.speed_scale = 0.5
    sim.pure_pursuit.speed_limit = race_speed
    sim.controller.base_speed = race_speed
    sim.paused = False
    mcl_ates: list[float] = []
    last_scan = 0
    started = time.time()
    max_sim = max(40.0, laps * 30.0)
    max_wall = max(70.0, laps * 35.0)
    while not sim.finished and sim.sim_time < max_sim and time.time() - started < max_wall:
        sim.step_once()
        if sim.slam_scan_count != last_scan:
            last_scan = sim.slam_scan_count
            if sim.slam_mcl_pose is not None:
                mcl_ates.append(sim.slam_mcl_ate)
    table = Table(title=f"Map-and-race — {laps} timed laps")
    table.add_column("Metric")
    table.add_column("Value", justify="right")
    table.add_row("Mapping lap (s)", f"{mapping_lap:.2f}")
    table.add_row("Map seam (cm)", f"{seam * 100:.1f}")
    table.add_row("Map file", str(map_path))
    table.add_row("Race speed (m/s)", f"{race_speed:.1f}")
    table.add_row(
        "Global relocalization",
        "on" if global_relocalization else "off",
    )
    table.add_row("Race status", "OK" if len(sim.lap_times) >= laps else f"DNF ({len(sim.lap_times)}/{laps})")
    table.add_row("Race total (s)", f"{sum(sim.lap_times):.2f}")
    table.add_row("Race best (s)", f"{min(sim.lap_times):.2f}" if sim.lap_times else "—")
    table.add_row("Race laps", ", ".join(f"{t:.2f}" for t in sim.lap_times) or "—")
    table.add_row("MCL ATE mean (cm)", f"{float(np.mean(mcl_ates)) * 100:.1f}" if mcl_ates else "—")
    table.add_row("MCL ATE max (cm)", f"{float(np.max(mcl_ates)) * 100:.1f}" if mcl_ates else "—")
    table.add_row("Global relocalizations", str(sim.slam_global_relocalizations))
    console.print(table)
    sim.world.close()


@app.command("localization-bench")
def localization_bench(
    laps: int = typer.Option(5, min=1, max=5),
    controller: str = typer.Option("pure_pursuit", help="wall_follow or pure_pursuit"),
) -> None:
    """Compare GT, odometry-only and MCL localization with the racing controller."""

    import numpy as np

    from awake_sim.server.pybullet_sim import PyBulletSimulation

    if controller not in {"wall_follow", "pure_pursuit"}:
        console.print("[red]controller must be wall_follow or pure_pursuit[/red]")
        raise typer.Exit(code=2)
    rows: list[tuple[str, str, str, str, str]] = []
    for mode in ("gt", "odom", "mcl"):
        sim = PyBulletSimulation(enable_slam=True)
        # Use a safe racing line and a moderate speed for all three modes so the
        # comparison isolates localization rather than corner-cutting risk.
        sim.racing_line = build_racing_line(sim.track, sim.vehicle, lateral_margin=0.35)
        sim.pure_pursuit.line = sim.racing_line
        sim.speed_limit = 1.2
        sim.pure_pursuit.speed_limit = 1.2
        sim.controller.base_speed = 1.2
        sim.scenario.race.laps = laps
        sim.auto_controller = controller
        sim.localization_mode = mode
        sim.paused = False
        if mode == "mcl":
            sim.slam_mcl_enabled = True
            assert sim.slam is not None
            sim.slam.set_mcl_enabled(True)
        ates: list[float] = []
        last_scan = 0
        started = time.time()
        max_sim = max(40.0, laps * 30.0)
        max_wall = max(70.0, laps * 35.0)
        while not sim.finished and sim.sim_time < max_sim and time.time() - started < max_wall:
            sim.step_once()
            if sim.slam_scan_count != last_scan:
                last_scan = sim.slam_scan_count
                state = sim.world.state()
                if mode == "gt":
                    error = 0.0
                elif mode == "odom":
                    odom = sim.odometry.pose
                    error = float(np.hypot(odom.x - state.x, odom.y - state.y))
                else:
                    error = sim.slam_mcl_ate
                ates.append(error)
        sim.world.close()
        table_total = sum(sim.lap_times)
        table_best = min(sim.lap_times) if sim.lap_times else 0.0
        rows.append(
            (
                mode.upper(),
                f"{table_total:.2f}" if sim.lap_times else "DNF",
                f"{table_best:.2f}" if sim.lap_times else "—",
                f"{float(np.mean(ates)) * 100:.1f}" if ates else "—",
                f"{float(np.max(ates)) * 100:.1f}" if ates else "—",
            )
        )
    table = Table(title=f"Localization comparison — {laps} laps, {controller}")
    table.add_column("Pose source")
    table.add_column("Total (s)", justify="right")
    table.add_column("Best (s)", justify="right")
    table.add_column("ATE mean (cm)", justify="right")
    table.add_column("ATE max (cm)", justify="right")
    for row in rows:
        table.add_row(*row)
    console.print(table)


@app.command("slam-bench")
def slam_bench(
    duration: float = typer.Option(20.0, min=2.0, max=120.0, help="Simulated seconds"),
    controller: str = typer.Option("pure_pursuit", help="wall_follow or pure_pursuit"),
    mcl: bool = typer.Option(False, "--mcl", help="Enable particle-filter MCL"),
    loop_closure: bool = typer.Option(
        False, "--loop-closure", help="Enable experimental loop closure"
    ),
) -> None:
    """Run the live SLAM front-end against ground truth."""

    import numpy as np

    from awake_sim.server.pybullet_sim import PyBulletSimulation

    if controller not in {"wall_follow", "pure_pursuit"}:
        console.print("[red]controller must be wall_follow or pure_pursuit[/red]")
        raise typer.Exit(code=2)
    sim = PyBulletSimulation(enable_slam=True)
    sim.auto_controller = controller
    sim.paused = False
    sim.slam_mcl_enabled = mcl
    sim.slam_loop_enabled = loop_closure
    assert sim.slam is not None
    sim.slam.set_mcl_enabled(mcl)
    sim.slam.set_loop_closure_enabled(loop_closure)
    ates: list[float] = []
    mcl_ates: list[float] = []
    scan_times: list[float] = []
    last_scan = 0
    while sim.sim_time < duration and not sim.finished:
        sim.step_once()
        if sim.slam_scan_count != last_scan:
            last_scan = sim.slam_scan_count
            ates.append(sim.slam_ate)
            scan_times.append(sim.slam_scan_ms)
            if mcl and sim.slam_mcl_pose is not None:
                mcl_ates.append(sim.slam_mcl_ate)
    occupancy = sim.slam.map_array() if sim.slam is not None else None
    sim.world.close()
    if not ates:
        console.print("[red]no SLAM scans were processed[/red]")
        raise typer.Exit(code=1)
    table = Table(title=f"SLAM front-end — {duration:.0f} s {controller}")
    table.add_column("Metric")
    table.add_column("Value", justify="right")
    table.add_row("Scans", str(len(ates)))
    table.add_row("ATE mean (cm)", f"{float(np.mean(ates)) * 100:.2f}")
    table.add_row("ATE max (cm)", f"{float(np.max(ates)) * 100:.2f}")
    table.add_row("Scan mean (ms)", f"{float(np.mean(scan_times)):.1f}")
    table.add_row("Scan p95 (ms)", f"{float(np.percentile(scan_times, 95)):.1f}")
    table.add_row("Loop closures", str(sim.slam_loop_closures))
    table.add_row("Graph nodes", str(sim.slam_graph_nodes))
    table.add_row("Odom drift (m)", f"{sim.odom_drift_m:.3f}")
    if mcl_ates:
        table.add_row("MCL ATE mean (cm)", f"{float(np.mean(mcl_ates)) * 100:.2f}")
        table.add_row("MCL ATE max (cm)", f"{float(np.max(mcl_ates)) * 100:.2f}")
        table.add_row("MCL ESS (last)", f"{sim.slam_mcl_ess:.0f}")
        table.add_row("MCL spread (m)", f"{sim.slam_mcl_spread:.2f}")
    if occupancy is not None:
        table.add_row("Occupied cells", str(int(np.count_nonzero(occupancy > 0.5))))
        table.add_row("Free cells", str(int(np.count_nonzero(occupancy < -0.2))))
    console.print(table)


if __name__ == "__main__":
    app()
