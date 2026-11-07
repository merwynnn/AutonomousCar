"""Host-only bridge between the dashboard and CMA-ES training runs.

The dashboard never trains in-process: starting a run spawns
``python -m training.cma_opt`` as a subprocess and follows the
``data/training/<stage>_status.json`` / ``<stage>_history.jsonl`` files the
runner writes. This keeps the simulator loop free of training load.
"""

from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = REPO_ROOT / "data" / "training"
RESULTS_DIR = DATA_DIR / "results"
STAGES = ("b1", "b2a", "b2b")

STARTABLE_FIELDS = {
    "generations": int,
    "popsize": int,
    "workers": int,
    "subrays": int,
    "seed": int,
    "sigma0": float,
    "max_time": float,
    "hidden": int,
    "budget_seconds": float,
    "random_track": bool,
    "tracks_per_generation": int,
    "stop_after_stall": int,
    "fitness_aggregation": str,
    "vary_sensor_seed": bool,
}


class TrainingError(Exception):
    """Raised for invalid training requests; mapped to a 400 response."""


@dataclass
class ActiveRun:
    stage: str
    process: subprocess.Popen[bytes]
    log_path: Path
    started_at: float


class TrainingManager:
    def __init__(self, data_dir: Path = DATA_DIR) -> None:
        self.data_dir = data_dir
        self._active: dict[str, ActiveRun] = {}

    # ------------------------------------------------------------- processes
    def is_running(self, stage: str) -> bool:
        run = self._active.get(stage)
        if run is None:
            return False
        if run.process.poll() is not None:
            del self._active[stage]
            return False
        return True

    def start(self, params: dict[str, Any]) -> dict[str, Any]:
        stage = str(params.get("stage", ""))
        if stage not in STAGES:
            raise TrainingError(f"stage must be one of {', '.join(STAGES)}")
        if self.is_running(stage):
            raise TrainingError(f"stage {stage} is already training")
        if importlib.util.find_spec("cma") is None:
            raise TrainingError(
                "the 'train' extra is not installed; run `uv sync --extra train`"
            )
        command = [sys.executable, "-m", "training.cma_opt", "--stage", stage]
        init_result = params.get("init_result")
        if init_result:
            init_path = self._result_policy_path(str(init_result))
            if init_path is None:
                raise TrainingError(f"unknown result {init_result!r}")
            policy_type = self._policy_type(init_path)
            if stage == "b1" and policy_type != "b1":
                raise TrainingError(
                    f"result {init_result!r} is a {policy_type} policy; "
                    "stage b1 can only warm-start from b1 results "
                    "(b2a/b2b can start from b1 or b2)"
                )
            command.extend(["--init", str(init_path)])
        run_name = params.get("run_name") or (
            f"{stage}_{time.strftime('%Y%m%d_%H%M%S')}"
        )
        safe = "".join(c for c in str(run_name) if c.isalnum() or c in "-_")
        if not safe:
            raise TrainingError("run_name must contain letters or digits")
        run_dir = RESULTS_DIR / safe
        # Dashboard runs keep their best reference-lap policy inside their own
        # library directory: they never auto-overwrite the shipped weights
        # (algorithms/weights/policy.npz), which are validated for robustness.
        command.extend(["--run-name", safe])
        command.extend(["--out", str(run_dir / "best_reference_lap.npz")])
        for field, kind in STARTABLE_FIELDS.items():
            if field not in params or params[field] is None:
                continue
            value = params[field]
            if kind is bool:
                if value:
                    command.append(f"--{field.replace('_', '-')}")
                continue
            try:
                command.extend([f"--{field.replace('_', '-')}", str(kind(value))])
            except (TypeError, ValueError) as exc:
                raise TrainingError(f"invalid value for {field}: {value!r}") from exc
        self.data_dir.mkdir(parents=True, exist_ok=True)
        log_path = self.data_dir / f"{stage}_run.log"
        log_handle = log_path.open("ab", buffering=0)
        try:
            process = subprocess.Popen(
                command,
                cwd=REPO_ROOT,
                stdout=log_handle,
                stderr=subprocess.STDOUT,
            )
        except OSError as exc:
            log_handle.close()
            raise TrainingError(f"could not start training: {exc}") from exc
        finally:
            # The child keeps its own descriptor; the parent can close its copy.
            log_handle.close()
        self._active[stage] = ActiveRun(stage, process, log_path, time.time())
        return {
            "ok": True,
            "stage": stage,
            "pid": process.pid,
            "log": str(log_path),
            "command": command,
            "init_result": init_result or None,
        }

    # ------------------------------------------------------- results library
    def _result_dir(self, name: str) -> Path:
        safe = "".join(c for c in name if c.isalnum() or c in "-_")
        return RESULTS_DIR / safe

    def _result_policy_path(self, name: str) -> Path | None:
        run_dir = self._result_dir(name)
        for candidate in ("objective_best.npz", "best_reference_lap.npz"):
            path = run_dir / candidate
            if path.exists():
                return path
        return None

    def _policy_type(self, path: Path) -> str:
        import numpy as np

        try:
            with np.load(path, allow_pickle=False) as data:
                return str(np.asarray(data["policy_type"]).item())
        except Exception:  # noqa: BLE001 - unreadable file: treat as unknown
            return "unknown"

    def list_results(self) -> list[dict[str, Any]]:
        results: list[dict[str, Any]] = []
        if not RESULTS_DIR.exists():
            return results
        for meta_path in sorted(RESULTS_DIR.glob("*/meta.json")):
            try:
                meta = json.loads(meta_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                continue
            policy = self._result_policy_path(meta.get("run_name", meta_path.parent.name))
            if policy is None:
                continue
            results.append(
                {
                    "name": meta_path.parent.name,
                    "stage": meta.get("stage"),
                    "policy_type": self._policy_type(policy),
                    "best_lap_s": meta.get("best_lap_s"),
                    "best_contacts": meta.get("best_contacts"),
                    "generations": meta.get("generations"),
                    "wall_time_s": meta.get("wall_time_s"),
                    "aggregation": meta.get("aggregation"),
                    "init": meta.get("init"),
                    "created": meta_path.stat().st_mtime,
                    "path": str(policy),
                }
            )
        results.sort(key=lambda item: item["created"], reverse=True)
        return results

    def result_path(self, name: str) -> Path | None:
        return self._result_policy_path(name)

    def stop(self, stage: str) -> dict[str, Any]:
        run = self._active.get(stage)
        if run is None or run.process.poll() is not None:
            self._active.pop(stage, None)
            return {"ok": True, "stage": stage, "stopped": False}
        run.process.terminate()
        try:
            run.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            run.process.kill()
            run.process.wait(timeout=5)
        del self._active[stage]
        return {"ok": True, "stage": stage, "stopped": True}

    # ------------------------------------------------------------------ reads
    def _status_path(self, stage: str) -> Path:
        return self.data_dir / f"{stage}_status.json"

    def _history_path(self, stage: str) -> Path:
        return self.data_dir / f"{stage}_history.jsonl"

    def best_path(self, stage: str) -> Path | None:
        candidate = self.data_dir / f"{stage}_best.npz"
        return candidate if candidate.exists() else None

    def status(self, stage: str | None = None) -> dict[str, Any]:
        if stage is not None:
            return self._stage_status(stage)
        return {"runs": [self._stage_status(name) for name in self._known_stages()]}

    def _known_stages(self) -> list[str]:
        stages = set(STAGES)
        if self.data_dir.exists():
            for path in self.data_dir.glob("*_status.json"):
                stages.add(path.name[: -len("_status.json")])
        stages.update(self._active)
        return sorted(stages)

    def _stage_status(self, stage: str) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "stage": stage,
            "phase": "idle",
            "process_running": self.is_running(stage),
            "best_available": self.best_path(stage) is not None,
            "best_path": (
                str(self.best_path(stage)) if self.best_path(stage) else None
            ),
        }
        status_path = self._status_path(stage)
        if status_path.exists():
            try:
                data = json.loads(status_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                data = {}
            payload.update(data)
        run = self._active.get(stage)
        if run is not None:
            payload["pid"] = run.process.pid
            payload["log"] = str(run.log_path)
            payload["process_running"] = True
        payload["stage"] = stage
        return payload

    def history(self, stage: str, limit: int = 2000) -> list[dict[str, Any]]:
        path = self._history_path(stage)
        if not path.exists():
            return []
        records: list[dict[str, Any]] = []
        try:
            with path.open("r", encoding="utf-8") as handle:
                for line in handle:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        records.append(json.loads(line))
                    except json.JSONDecodeError:
                        continue
        except OSError:
            return []
        return records[-limit:]

    def active_stages(self) -> list[str]:
        return [stage for stage in self._active if self.is_running(stage)]
