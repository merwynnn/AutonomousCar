# Runbook

## 1. Prerequisites

- WSL2 Ubuntu (tested on 26.04) with a browser on Windows.
- Python 3.11 managed by `uv` (do not use system Python 3.14: PyBullet wheels stop
  at cp311).
- Node.js 22+ and npm.
- System toolchain: `build-essential cmake ninja-build libeigen3-dev`.

One-time toolchain install:

```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build libeigen3-dev
```

Install `uv` if missing:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

## 2. Install

```bash
cd /mnt/c/Users/Louis/Documents/Projects/AutonomousCar
uv sync --all-extras
uv run python -m awake_sim.cli build-cpp
cd web && npm install && cd ..
```

## 3. Run

Two terminals:

```bash
# terminal 1 — backend
uv run uvicorn awake_sim.server.app:app --reload --port 8000

# terminal 2 — frontend
cd web
npm run dev -- --port 5173
```

Open `http://localhost:5173`. The Vite dev server proxies `/api` and `/ws` to the
backend.

Or use:

```bash
make dev
```

## 4. Checks

```bash
make lint        # ruff + mypy
make test        # pytest
make cpp-test    # C++ tests
make check       # lint + test + web build
```

## 5. Common commands

```bash
# headless run
uv run awake-sim run --scenario configs/scenarios/test_track.yaml

# controller comparison
uv run awake-sim compare --laps 5

# lidar-only furthest-gap benchmark (no map, no pose)
uv run awake-sim gap-bench --laps 5 --speed 3.2

# controller-specific race
uv run awake-sim race --controller gap_follow --laps 5 --speed 3.2

# localization comparison (GT / odom / MCL)
uv run awake-sim localization-bench --laps 1

# sensor-only mapping lap, freeze/save map, then MCL race
uv run awake-sim map-and-race --laps 1

# SLAM front-end benchmark + MCL
uv run awake-sim slam-bench --duration 20 --mcl

# experimental loop closure (may false-positive on repetitive walls)
uv run awake-sim slam-bench --duration 20 --loop-closure

# C++ rebuild after changes
uv run python -m awake_sim.cli build-cpp
```

## 6. Ports

| Service | Default |
|---|---|
| FastAPI | 8000 |
| Vite | 5173 |
| WebSocket | same origin, `/ws/telemetry` |

If a port is busy, change `PORT`/`WEB_PORT` in the Makefile or pass `--port`.

## 7. Troubleshooting

### `pybullet` import fails

Check the Python version:

```bash
uv run python --version   # must be 3.11.x
```

If not, run `uv python pin 3.11` and `uv sync --all-extras`.

### C++ extension does not build

- Verify `g++ --version`, `cmake --version`, `ninja --version`.
- Check Eigen: `ls /usr/include/eigen3/Eigen/Core`.
- Re-run `uv run python -m awake_sim.cli build-cpp -v`.
- Ensure `pybind11` is installed: `uv run python -c "import pybind11; print(pybind11.__version__)"`.

### Dashboard cannot connect

- Backend running on port 8000?
- Vite proxy configured in `web/vite.config.ts`?
- Open browser devtools → Network → WS; check the error.
- If accessing from Windows, use `localhost`, not the WSL IP.

### Simulation is slow

- Close unused browser tabs.
- Use headless `race` / `compare` for strategy runs.
- Reduce lidar beams/sub-rays in the scenario config.

### WSL memory pressure

The laptop has ~7.3 GB RAM. Avoid running Gazebo/ROS, huge point clouds or many
browser tabs simultaneously. Avoid running multiple headless comparisons at once.

## 8. Data and cleanup

- Optional raw logs live in `data/` (gitignored); experiment reports live in
  `docs/experiments/`.
- `make clean` removes build artefacts and dependencies; use with care.
- To reset the Python environment: `uv sync --reinstall --all-extras`.

## 9. Useful paths

| Path | Contents |
|---|---|
| `configs/` | vehicle, sensor, track, scenario YAML |
| `data/` | optional raw logs (gitignored) |
| `docs/devlog/` | work log |
| `docs/assets/` | screenshots |
| `cpp/` | SLAM core |
| `src/awake_sim/` | Python package |
| `web/` | dashboard |
