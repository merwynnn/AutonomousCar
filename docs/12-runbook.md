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

# batch experiment
uv run awake-sim batch configs/experiments/example.yaml

# replay a recording
uv run awake-sim replay data/recordings/<run>.jsonl

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
- Use headless batch for experiments.
- Reduce lidar beams/sub-rays in the scenario config.
- Check `nproc`; batch worker count may be too high.

### WSL memory pressure

The laptop has ~7.3 GB RAM. Avoid running Gazebo/ROS, huge point clouds or many
browser tabs simultaneously. Batch runs should use 2–4 workers.

## 8. Data and cleanup

- Recordings and experiments live in `data/` (gitignored).
- `make clean` removes build artefacts and dependencies; use with care.
- To reset the Python environment: `uv sync --reinstall --all-extras`.

## 9. Useful paths

| Path | Contents |
|---|---|
| `configs/` | vehicle, sensor, track, scenario YAML |
| `configs/experiments/` | batch definitions |
| `data/experiments/` | results and recordings |
| `docs/devlog/` | work log |
| `docs/assets/` | screenshots |
| `cpp/` | SLAM core |
| `src/awake_sim/` | Python package |
| `web/` | dashboard |
