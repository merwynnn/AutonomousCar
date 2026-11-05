"""Live terminal view of a CMA-ES training run.

The runner writes ``data/training/<stage>_status.json`` after every
generation.  This watcher renders it as a single refreshing block so you can
see progress, elapsed time, ETA and the best clean lap without scrolling.

Usage::

    uv run python -m training.watch --stage b2a

Use ``--once`` to print a single snapshot (for logs or scripts).
"""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime, timedelta
from pathlib import Path

from training.progress import format_duration, progress_bar


def _read(path: Path) -> dict[str, object] | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def render(status: dict[str, object], stage: str) -> str:
    phase = str(status.get("phase", "waiting"))
    generations = int(status.get("generations", 0) or 0)
    generation = int(status.get("generation", -1) or -1)
    percent = float(status.get("percent", 0.0) or 0.0)
    elapsed = float(status.get("elapsed_s", 0.0) or 0.0)
    eta = status.get("eta_s")
    eta_s = float(eta) if isinstance(eta, int | float) else None
    best_lap = status.get("best_lap_s")
    best_contacts = status.get("best_contacts")
    best_generation = status.get("best_generation")
    current_lap = status.get("generation_lap_s")
    current_contacts = status.get("generation_contacts")
    sigma = status.get("sigma")
    average = status.get("average_generation_s")
    updated = float(status.get("updated_at", time.time()) or time.time())

    lines: list[str] = []
    lines.append(
        f"stage {stage}  |  {phase}  |  "
        f"workers {status.get('workers')}  pop {status.get('popsize')}"
    )
    lines.append(
        f"{progress_bar(percent / 100.0, 32)} {percent:5.1f}%  "
        f"gen {generation + 1 if generation >= 0 else 0}/{generations}"
    )
    finish = ""
    if eta_s is not None and phase == "running":
        finish = (datetime.now() + timedelta(seconds=eta_s)).strftime("%H:%M:%S")
    lines.append(
        f"elapsed {format_duration(elapsed)}  |  left {format_duration(eta_s)}  |  "
        f"finish ~{finish or '--:--:--'}"
    )
    best_text = f"{float(best_lap):.2f} s" if isinstance(best_lap, int | float) else "DNF"
    lines.append(
        f"best {best_text}  (gen {best_generation}, contacts {best_contacts})"
    )
    cur_text = f"{float(current_lap):.2f} s" if isinstance(current_lap, int | float) else "DNF"
    lines.append(
        f"current gen {cur_text}  contacts {current_contacts}  "
        f"sigma {float(sigma):.4f}" if isinstance(sigma, int | float) else f"current gen {cur_text}"
    )
    if isinstance(average, int | float):
        lines.append(f"avg generation {float(average):.1f} s")
    age = time.time() - updated
    if age > 30 and phase == "running":
        lines.append(f"WARNING: status is {age:.0f} s stale")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", required=True)
    parser.add_argument("--data-dir", type=Path, default=Path("data/training"))
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    path = args.data_dir / f"{args.stage}_status.json"
    if args.once:
        status = _read(path)
        print(render(status, args.stage) if status else f"no status file at {path}")
        return

    print(f"watching {path} (Ctrl-C to stop)")
    try:
        while True:
            status = _read(path)
            block = render(status, args.stage) if status else f"waiting for {path}"
            print("\x1b[2J\x1b[H" + block, flush=True)
            if status and status.get("phase") == "finished":
                return
            time.sleep(max(0.2, args.interval))
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
