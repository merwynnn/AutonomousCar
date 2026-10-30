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
    _run(["cmake", "--build", str(build_dir), "--target", "awake_slam_tests"])
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


def _run_race(controller: str, laps: int, scenario: Path) -> tuple[list[float], float]:
    from awake_sim.server.pybullet_sim import PyBulletSimulation

    sim = PyBulletSimulation(scenario, enable_lidar=controller == "wall_follow")
    sim.scenario.race.laps = laps
    sim.auto_controller = controller
    sim.paused = False
    started = time.time()
    while not sim.finished and sim.sim_time < 240 and time.time() - started < 220:
        sim.step_once()
    sim.world.close()
    return list(sim.lap_times), time.time() - started


@app.command()
def race(
    controller: str = typer.Option(
        "pure_pursuit", help="wall_follow or pure_pursuit"
    ),
    laps: int = typer.Option(5, min=1, max=5),
    scenario: Path = typer.Option(
        ROOT / "configs" / "scenarios" / "test_track_baseline.yaml",
    ),
) -> None:
    """Run a headless race with a selected controller."""

    if controller not in {"wall_follow", "pure_pursuit"}:
        console.print("[red]controller must be wall_follow or pure_pursuit[/red]")
        raise typer.Exit(code=2)
    lap_times, wall = _run_race(controller, laps, scenario)
    table = Table(title=f"{controller} — {laps} laps")
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
    scenario: Path = typer.Option(
        ROOT / "configs" / "scenarios" / "test_track_baseline.yaml",
    ),
) -> None:
    """Compare wall-follow and pure pursuit on the test track."""

    rows: list[tuple[str, str, str, str]] = []
    for controller in ("wall_follow", "pure_pursuit"):
        lap_times, wall = _run_race(controller, laps, scenario)
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
    table = Table(title=f"Controller comparison — {laps} laps")
    table.add_column("Controller")
    table.add_column("Total (s)", justify="right")
    table.add_column("Best (s)", justify="right")
    table.add_column("Wall (s)", justify="right")
    for row in rows:
        table.add_row(*row)
    console.print(table)


@app.command()
def batch(
    config: Path = typer.Argument(..., help="Batch experiment YAML"),
) -> None:
    """Run a full batch matrix (planned for the next M2 increment)."""

    console.print(f"[yellow]Batch matrix runner is the next M2 increment; config: {config}[/yellow]")
    raise typer.Exit(code=2)


@app.command("slam-bench")
def slam_bench(
    duration: float = typer.Option(20.0, min=2.0, max=120.0, help="Simulated seconds"),
    controller: str = typer.Option("pure_pursuit", help="wall_follow or pure_pursuit"),
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
    ates: list[float] = []
    scan_times: list[float] = []
    last_scan = 0
    while sim.sim_time < duration and not sim.finished:
        sim.step_once()
        if sim.slam_scan_count != last_scan:
            last_scan = sim.slam_scan_count
            ates.append(sim.slam_ate)
            scan_times.append(sim.slam_scan_ms)
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
    if occupancy is not None:
        table.add_row("Occupied cells", str(int(np.count_nonzero(occupancy > 0.5))))
        table.add_row("Free cells", str(int(np.count_nonzero(occupancy < -0.2))))
    console.print(table)


@app.command("slam-eval")
def slam_eval(
    recording: Path | None = typer.Argument(None, help="Recording to evaluate"),
) -> None:
    """Evaluate a real/sim recording through SLAM (next increment)."""

    console.print("[yellow]Recording replay evaluation is the next SLAM increment[/yellow]")
    if recording:
        console.print(f"recording: {recording}")
    raise typer.Exit(code=2)


if __name__ == "__main__":
    app()
