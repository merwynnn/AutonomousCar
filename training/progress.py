"""Human- and machine-readable progress reporting for long CMA-ES runs.

The runner prints one compact line per generation and mirrors it into
``data/training/<stage>_status.json`` and ``<stage>_status.txt``. You can
watch a run from another terminal with:

    watch -n 2 cat data/training/b1_status.txt

or read the JSON from a dashboard/script.
"""

from __future__ import annotations

import json
import os
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


def format_duration(seconds: float | None) -> str:
    if seconds is None or seconds < 0 or seconds == float("inf"):
        return "--:--"
    total = int(round(seconds))
    hours, rem = divmod(total, 3600)
    minutes, secs = divmod(rem, 60)
    if hours:
        return f"{hours:d}:{minutes:02d}:{secs:02d}"
    return f"{minutes:02d}:{secs:02d}"


def progress_bar(fraction: float, width: int = 20) -> str:
    fraction = max(0.0, min(1.0, fraction))
    filled = int(round(fraction * width))
    return "[" + "#" * filled + "." * (width - filled) + "]"


@dataclass
class RunReporter:
    stage: str
    generations: int
    popsize: int
    workers: int
    subrays: int
    seed: int
    out: Path
    data_dir: Path
    budget_seconds: float = 0.0
    history: list[float] = field(default_factory=list)
    started: float = field(default_factory=time.perf_counter)
    best_lap: float | None = None
    best_contacts: int | None = None
    best_generation: int | None = None
    best_progress: float = 0.0
    best_score: float | None = None
    phase: str = "starting"

    @property
    def json_path(self) -> Path:
        return self.data_dir / f"{self.stage}_status.json"

    @property
    def text_path(self) -> Path:
        return self.data_dir / f"{self.stage}_status.txt"

    def _write(self, text: str, payload: dict[str, Any]) -> None:
        self.data_dir.mkdir(parents=True, exist_ok=True)
        for path, content in ((self.text_path, text + "\n"),):
            with tempfile.NamedTemporaryFile(
                "w", dir=path.parent, delete=False, encoding="utf-8"
            ) as handle:
                handle.write(content)
                temporary = Path(handle.name)
            os.replace(temporary, path)
        with tempfile.NamedTemporaryFile(
            "w", dir=self.json_path.parent, delete=False, encoding="utf-8"
        ) as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            temporary = Path(handle.name)
        os.replace(temporary, self.json_path)

    def start(self) -> None:
        elapsed = time.perf_counter() - self.started
        text = (
            f"stage {self.stage} | starting | {self.generations} generations x "
            f"{self.popsize} candidates / {self.workers} workers | "
            f"subrays {self.subrays} | seed {self.seed} | out {self.out}"
        )
        print(text, flush=True)
        print(f"status file: {self.text_path}", flush=True)
        self._write(
            text,
            {
                "stage": self.stage,
                "phase": "starting",
                "generation": -1,
                "generations": self.generations,
                "percent": 0.0,
                "elapsed_s": elapsed,
                "eta_s": None,
                "popsize": self.popsize,
                "workers": self.workers,
                "subrays": self.subrays,
                "seed": self.seed,
                "out": str(self.out),
                "budget_seconds": self.budget_seconds or None,
                "updated_at": time.time(),
            },
        )

    def initial(self, result: Any, rollout_seconds: float) -> None:
        lap = result.lap_time_s
        lap_text = f"{lap:.2f} s" if lap is not None else "DNF"
        rough_generation = rollout_seconds * self.popsize / max(self.workers, 1)
        rough_total = rough_generation * self.generations
        if self.budget_seconds:
            rough_total = min(rough_total, self.budget_seconds)
        text = (
            f"initial policy: lap {lap_text} | contacts {result.contacts} | "
            f"progress {result.progress_m:.2f} m | score {result.score:.1f} | "
            f"rough ETA {format_duration(rough_total)}"
        )
        print(text, flush=True)
        self._write(
            text,
            {
                "stage": self.stage,
                "phase": "running",
                "generation": -1,
                "generations": self.generations,
                "percent": 0.0,
                "elapsed_s": time.perf_counter() - self.started,
                "eta_s": rough_total,
                "eta_text": format_duration(rough_total),
                "initial_lap_s": lap,
                "initial_contacts": result.contacts,
                "initial_progress_m": result.progress_m,
                "initial_score": result.score,
                "popsize": self.popsize,
                "workers": self.workers,
                "subrays": self.subrays,
                "seed": self.seed,
                "out": str(self.out),
                "budget_seconds": self.budget_seconds or None,
                "updated_at": time.time(),
            },
        )

    def update(
        self,
        *,
        generation: int,
        result: Any,
        best_lap: float | None,
        best_contacts: int | None,
        best_generation: int | None,
        best_progress: float,
        best_score: float | None,
        sigma: float,
        track_length_m: float,
        generation_seconds: float,
        track_note: str = "",
    ) -> None:
        self.phase = "running"
        self.history.append(generation_seconds)
        self.best_lap = best_lap
        self.best_contacts = best_contacts
        self.best_generation = best_generation
        self.best_progress = best_progress
        self.best_score = best_score
        done = generation + 1
        fraction = done / max(self.generations, 1)
        elapsed = time.perf_counter() - self.started
        average_generation = sum(self.history) / len(self.history)
        remaining = max(0, self.generations - done)
        eta = average_generation * remaining
        if self.budget_seconds:
            eta = min(eta, max(0.0, self.budget_seconds - elapsed))
        lap = result.lap_time_s
        lap_text = f"{lap:.2f}s" if lap is not None else "DNF"
        best_text = f"{best_lap:.2f}s" if best_lap is not None else "DNF"
        text = (
            f"stage {self.stage} | {progress_bar(fraction)} {fraction * 100:5.1f}% "
            f"| gen {done}/{self.generations} | elapsed {format_duration(elapsed)} "
            f"| ETA {format_duration(eta)} | best {best_text} "
            f"(gen {best_generation}, {best_contacts} contacts) "
            f"| current {lap_text} | sigma {sigma:.4f} "
            f"| progress {result.progress_m:.2f}/{track_length_m:.2f} m"
            + (f" | track {track_note}" if track_note else "")
        )
        print(text, flush=True)
        self._write(
            text,
            {
                "stage": self.stage,
                "phase": "running",
                "generation": generation,
                "generations": self.generations,
                "percent": round(fraction * 100.0, 1),
                "elapsed_s": elapsed,
                "eta_s": eta,
                "eta_text": format_duration(eta),
                "average_generation_s": average_generation,
                "generation_seconds": generation_seconds,
                "best_lap_s": best_lap,
                "best_contacts": best_contacts,
                "best_generation": best_generation,
                "best_progress_m": best_progress,
                "best_score": best_score,
                "generation_lap_s": lap,
                "generation_contacts": result.contacts,
                "generation_progress_m": result.progress_m,
                "generation_score": result.score,
                "sigma": sigma,
                "popsize": self.popsize,
                "workers": self.workers,
                "subrays": self.subrays,
                "seed": self.seed,
                "track": track_note,
                "out": str(self.out),
                "budget_seconds": self.budget_seconds or None,
                "updated_at": time.time(),
            },
        )

    def finish(self, *, reason: str, best_lap: float | None, best_contacts: int | None) -> None:
        self.phase = "finished"
        elapsed = time.perf_counter() - self.started
        best_text = f"{best_lap:.2f}s" if best_lap is not None else "DNF"
        text = (
            f"stage {self.stage} | finished ({reason}) | elapsed "
            f"{format_duration(elapsed)} | best {best_text} | "
            f"contacts {best_contacts} | {self.text_path}"
        )
        print(text, flush=True)
        self._write(
            text,
            {
                "stage": self.stage,
                "phase": "finished",
                "reason": reason,
                "generation": self.generations - 1,
                "generations": self.generations,
                "percent": 100.0,
                "elapsed_s": elapsed,
                "eta_s": 0.0,
                "best_lap_s": best_lap,
                "best_contacts": best_contacts,
                "best_generation": self.best_generation,
                "popsize": self.popsize,
                "workers": self.workers,
                "subrays": self.subrays,
                "seed": self.seed,
                "out": str(self.out),
                "updated_at": time.time(),
            },
        )


__all__ = ["RunReporter", "format_duration", "progress_bar"]
