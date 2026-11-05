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
        }

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
