"""CMA-ES policy search over one-lap simulator rollouts.

Examples:
    uv run python -m training.cma_opt --stage b1 --generations 80
    uv run python -m training.cma_opt --stage b2a --generations 100
    uv run python -m training.cma_opt --stage b2b --generations 200 \
        --init data/training/b2a_best.npz --hidden 16
"""

from __future__ import annotations

import argparse
import json
import pickle
import time
from pathlib import Path
from typing import Any

import cma
import numpy as np
from algorithms.learned_gap import (
    B1_DEFAULT,
    B1_HIGH,
    B1_LOW,
    B1_SPECS,
    B2_SCALARS,
    WEIGHTS_PATH,
    _params_to_unit,
    b2_default_vector,
    b2_expand_to_hidden,
    b2_from_b1,
    b2_layout,
)
from simulator.track import build_test_track

from training.env import RolloutConfig
from training.export import load_policy, save_policy
from training.progress import RunReporter
from training.vec import RolloutPool

DATA_DIR = Path("data/training")


def _bounds_for(stage: str, hidden: int) -> tuple[np.ndarray, np.ndarray]:
    if stage == "b1":
        return B1_LOW * 0.0 + 0.0, B1_HIGH * 0.0 + 1.0
    layout = b2_layout(hidden)
    size = layout["_size"]
    low = np.full(size, -4.0, dtype=float)
    high = np.full(size, 4.0, dtype=float)
    start = layout["scalars"]
    low[start : start + len(B2_SCALARS)] = 0.0
    high[start : start + len(B2_SCALARS)] = 1.0
    return low, high


def _default_vector(stage: str, hidden: int) -> np.ndarray:
    if stage == "b1":
        return _params_to_unit(B1_DEFAULT, B1_SPECS)
    return b2_default_vector(hidden if stage == "b2b" else 0)


def _policy_type(stage: str) -> str:
    return "b1" if stage == "b1" else "b2"


def _load_init(path: Path | None, stage: str, hidden: int) -> np.ndarray:
    if path is None:
        return _default_vector(stage, hidden)
    policy_type, vector, loaded_hidden, meta = load_policy(path)
    if stage == "b2a" and policy_type == "b1":
        vector = b2_from_b1(vector)
    elif stage == "b2b" and policy_type == "b1":
        vector = b2_expand_to_hidden(b2_from_b1(vector), hidden)
    elif stage == "b2b" and policy_type == "b2" and loaded_hidden == 0:
        vector = b2_expand_to_hidden(vector, hidden)
    if stage == "b1" and policy_type != "b1":
        raise SystemExit(f"{path} is {policy_type}, not b1")
    return vector


def _maybe_export(
    out: Path,
    policy_type: str,
    vector: np.ndarray,
    hidden: int,
    meta: dict[str, Any],
) -> bool:
    """Export only when the new clean result is better than the current file."""

    new_time = meta.get("clean_lap_time_s")
    if out.exists():
        try:
            old_type, _, old_hidden, old_meta = load_policy(out)
            old_time = old_meta.get("clean_lap_time_s")
            old_clean = old_meta.get("contacts", 1) == 0 and old_time is not None
            new_clean = meta.get("contacts", 1) == 0 and new_time is not None
            if old_clean and not new_clean:
                return False
            if old_clean and new_clean and float(new_time) >= float(old_time):
                return False
        except Exception:  # noqa: BLE001 - corrupt/old file: replace it
            pass
    save_policy(out, policy_type, vector, hidden, meta)
    return True


def _meta_from_result(
    stage: str,
    generation: int,
    result: Any,
    start_s: float,
    args: argparse.Namespace,
) -> dict[str, Any]:
    return {
        "stage": stage,
        "generation": generation,
        "clean_lap_time_s": result.lap_time_s,
        "contacts": result.contacts,
        "contact_steps": result.contact_steps,
        "max_impulse": result.max_impulse,
        "top_speed_mps": result.top_speed_mps,
        "mean_speed_mps": result.mean_speed_mps,
        "min_wall_distance_m": result.min_wall_distance_m,
        "sim_time_s": result.sim_time_s,
        "progress_m": result.progress_m,
        "score": result.score,
        "start_s": start_s,
        "generations": generation + 1,
        "popsize": args.popsize,
        "seed": args.seed,
        "subrays": args.subrays,
    }


def _vector_summary(stage: str, hidden: int, vector: np.ndarray) -> str:
    if stage == "b1":
        physical = np.array(
            [
                B1_SPECS[i].low + np.clip(vector[i], 0, 1) * (B1_SPECS[i].high - B1_SPECS[i].low)
                for i in range(len(B1_SPECS))
            ]
        )
        return ", ".join(
            f"{spec.name}={physical[i]:.3f}" for i, spec in enumerate(B1_SPECS)
        )
    return f"{stage} vector length {vector.size}, hidden {hidden}"


def run(args: argparse.Namespace) -> int:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    hidden = args.hidden if args.stage == "b2b" else 0
    policy_type = _policy_type(args.stage)
    low, high = _bounds_for(args.stage, hidden)
    x0 = _load_init(args.init, args.stage, hidden)
    if x0.size != low.size:
        raise SystemExit(f"init vector has {x0.size} elements, expected {low.size}")
    x0 = np.clip(x0, low + 1e-9, high - 1e-9)

    default_start = float(build_test_track(width_m=0.9).start_s)
    track_length = float(build_test_track(width_m=0.9).length)
    rng = np.random.default_rng(args.seed)

    options = {
        "popsize": args.popsize,
        "bounds": [low.tolist(), high.tolist()],
        "seed": args.seed,
        "verbose": -9,
        "CMA_diagonal": args.stage == "b2b",
        "maxiter": args.generations,
        "tolfun": 1e-9,
        "tolx": 1e-9,
    }
    es = cma.CMAEvolutionStrategy(x0, args.sigma0, options)
    config = RolloutConfig(
        policy_type=policy_type,
        hidden=hidden,
        subrays=args.subrays,
        seed=args.seed,
        max_time_s=args.max_time,
    )

    log_path = DATA_DIR / f"{args.stage}_log.jsonl"
    best_vector = x0.copy()
    best_fitness = float("inf")
    best_meta: dict[str, Any] = {}
    started = time.perf_counter()
    reason = "max generations"
    reporter = RunReporter(
        stage=args.stage,
        generations=args.generations,
        popsize=args.popsize,
        workers=args.workers,
        subrays=args.subrays,
        seed=args.seed,
        out=Path(args.out),
        data_dir=DATA_DIR,
        budget_seconds=args.budget_seconds,
    )
    reporter.start()
    print(
        f"stage={args.stage} dim={x0.size} pop={args.popsize} "
        f"workers={args.workers} subrays={args.subrays} seed={args.seed}",
        flush=True,
    )
    print(f"initial: {_vector_summary(args.stage, hidden, x0)}", flush=True)

    with RolloutPool(config, args.workers) as pool:
        initial_started = time.perf_counter()
        initial_result = pool.evaluate([x0], start_s=default_start)[0]
        reporter.initial(initial_result, time.perf_counter() - initial_started)
        best_fitness = float(initial_result.fitness)
        best_vector = x0.copy()
        best_meta = _meta_from_result(
            args.stage, -1, initial_result, default_start, args
        )
        if initial_result.lap_time_s is not None:
            _maybe_export(Path(args.out), policy_type, best_vector, hidden, best_meta)
        if args.preflight:
            reporter.finish(
                reason="preflight",
                best_lap=initial_result.lap_time_s,
                best_contacts=initial_result.contacts,
            )
            return 0

        for generation in range(args.generations):
            generation_started = time.perf_counter()
            solutions = [np.asarray(s, dtype=float) for s in es.ask()]
            use_random_start = (
                generation < args.random_start_generations
                and rng.random() < args.random_start_prob
            )
            start_s = (
                (default_start + float(rng.uniform(0.0, track_length))) % track_length
                if use_random_start
                else default_start
            )
            results = pool.evaluate(solutions, start_s=start_s)
            generation_seconds = time.perf_counter() - generation_started
            fitnesses = [result.fitness for result in results]
            es.tell(solutions, fitnesses)

            best_index = int(np.argmin(fitnesses))
            generation_best = results[best_index]
            if fitnesses[best_index] < best_fitness:
                # A candidate that is fast from a random start offset may be
                # dirty at the reference pose.  Only promote/export a vector
                # that also validates cleanly at the reference start.
                validated = pool.evaluate(
                    [solutions[best_index]], start_s=default_start
                )[0]
                if (
                    validated.contacts == 0
                    and validated.lap_time_s is not None
                    and validated.fitness < best_fitness
                ):
                    best_fitness = float(validated.fitness)
                    best_vector = solutions[best_index].copy()
                    best_meta = _meta_from_result(
                        args.stage, generation, validated, default_start, args
                    )
                    _maybe_export(
                        Path(args.out), policy_type, best_vector, hidden, best_meta
                    )

            reporter.update(
                generation=generation,
                result=generation_best,
                best_lap=best_meta.get("clean_lap_time_s"),
                best_contacts=best_meta.get("contacts"),
                best_generation=best_meta.get("generation"),
                best_progress=best_meta.get("progress_m", 0.0),
                best_score=best_meta.get("score"),
                sigma=float(es.sigma),
                track_length_m=track_length,
                generation_seconds=generation_seconds,
            )

            if generation % args.log_every == 0 or generation == args.generations - 1:
                with log_path.open("a", encoding="utf-8") as handle:
                    handle.write(
                        json.dumps(
                            {
                                "stage": args.stage,
                                "generation": generation,
                                "best_fitness": best_fitness,
                                "best_lap_s": best_meta.get("clean_lap_time_s"),
                                "best_contacts": best_meta.get("contacts"),
                                "generation_lap_s": generation_best.lap_time_s,
                                "generation_contacts": generation_best.contacts,
                                "generation_progress_m": generation_best.progress_m,
                                "generation_score": generation_best.score,
                                "sigma": float(es.sigma),
                                "start_s": start_s,
                                "generation_seconds": generation_seconds,
                                "elapsed_s": time.perf_counter() - started,
                            }
                        )
                        + "\n"
                    )

            checkpoint = {
                "stage": args.stage,
                "hidden": hidden,
                "generation": generation,
                "best_vector": best_vector,
                "best_fitness": best_fitness,
                "best_meta": best_meta,
            }
            with (DATA_DIR / f"{args.stage}_checkpoint.pkl").open("wb") as handle:
                pickle.dump(checkpoint, handle)

            if args.budget_seconds and time.perf_counter() - started > args.budget_seconds:
                reason = "wall-clock budget"
                break
            stop = es.stop()
            if stop:
                reason = f"CMA-ES stop: {stop}"
                break

        # Candidate ranking uses occasional random start offsets to avoid
        # overfitting the launch pose.  Validate the winner on the reference
        # start before exporting, so the recorded lap is reproducible.
        final_result = pool.evaluate([best_vector], start_s=default_start)[0]
        best_meta = _meta_from_result(
            args.stage,
            int(best_meta.get("generation", -1)),
            final_result,
            default_start,
            args,
        )

    meta = {
        **best_meta,
        "stage": args.stage,
        "dimension": int(best_vector.size),
        "hidden": hidden,
        "workers": args.workers,
        "wall_time_s": time.perf_counter() - started,
    }
    (DATA_DIR / f"{args.stage}_best.npz").parent.mkdir(parents=True, exist_ok=True)
    save_policy(DATA_DIR / f"{args.stage}_best.npz", policy_type, best_vector, hidden, meta)
    exported = _maybe_export(Path(args.out), policy_type, best_vector, hidden, meta)
    reporter.finish(
        reason=reason,
        best_lap=meta.get("clean_lap_time_s"),
        best_contacts=meta.get("contacts"),
    )
    print(
        f"finished stage={args.stage} best_fit={best_fitness:.3f} "
        f"lap={meta.get('clean_lap_time_s')} contacts={meta.get('contacts')} "
        f"exported={exported} -> {args.out}",
        flush=True,
    )
    print(_vector_summary(args.stage, hidden, best_vector), flush=True)
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("b1", "b2a", "b2b"), required=True)
    parser.add_argument("--generations", type=int, default=100)
    parser.add_argument("--popsize", type=int, default=20)
    parser.add_argument("--sigma0", type=float, default=None)
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument("--subrays", type=int, default=1)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--max-time", type=float, default=32.0)
    parser.add_argument("--hidden", type=int, default=16)
    parser.add_argument("--init", type=Path, default=None)
    parser.add_argument("--out", type=Path, default=WEIGHTS_PATH)
    parser.add_argument("--log-every", type=int, default=1)
    parser.add_argument("--random-start-generations", type=int, default=25)
    parser.add_argument("--random-start-prob", type=float, default=0.20)
    parser.add_argument("--budget-seconds", type=float, default=0.0)
    parser.add_argument(
        "--preflight",
        action="store_true",
        help="evaluate the initial policy and print the estimated budget, then exit",
    )
    args = parser.parse_args()
    if args.sigma0 is None:
        args.sigma0 = 0.25 if args.stage == "b1" else 0.15
    raise SystemExit(run(args))


if __name__ == "__main__":
    main()
