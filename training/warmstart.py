"""Supervised warm start for the B2 beam scorer.

B1 is already a strong, clean controller.  Rather than hand-converting its
parameters (which does not reproduce its beam choice), we roll B1 out as an
expert and regress the B2 per-beam features onto B1's full-resolution score
surface.  A single-action imitation target collapses to "drive straight"
because most expert ticks happen on straights; the score surface preserves the
safety ordering around walls and gaps.

A short DAgger loop then rolls the B2 policy out and adds B1's score surface at
the states B2 actually visits, which fixes the compounding steering errors a
pure expert-state fit leaves behind.

Only B1's own runtime outputs are used; no ground truth, path or map enters the
fit.  The resulting vector is a *starting point* for CMA-ES, not a final policy.

Usage::

    uv run python -m training.warmstart \
        --b1 algorithms/weights/policy.npz \
        --out data/training/b2a_warm.npz \
        --laps 8 --iterations 3
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from algorithms.learned_gap import (
    B2_SCALARS,
    N_BEAM_FEATURES,
    N_GLOBAL,
    B1Params,
    BeamScorerPolicy,
    StructuredGapPolicy,
    _params_to_unit,
    _unit_to_params,
    b2_from_b1,
    b2_layout,
)
from simulator.simulation import Simulation

from training.export import load_policy, save_policy


@dataclass
class ExpertSample:
    beam_features: np.ndarray  # (n_beams, n_beam_features)
    global_features: np.ndarray  # (n_global,)
    target_scores: np.ndarray  # (n_beams,) B1's score at each B2 beam
    best_bearing: float
    speed: float
    source: str = "expert"


def _make_sim(start: float, seed: int, subrays: int) -> Simulation:
    sim = Simulation(
        "configs/sim/awake_2027.yaml",
        lidar_subrays=subrays,
        seed=seed,
    )
    sim.scenario.race.laps = 1
    sim.track.start_s = float(start)
    return sim


def _sample(
    prototype: BeamScorerPolicy,
    b1_scorer: StructuredGapPolicy,
    observation,
    previous_bearing: float,
    best_bearing: float,
    speed: float,
    source: str,
) -> ExpertSample | None:
    b1_scorer.last_angle = previous_bearing
    surface = b1_scorer.score_surface(observation)
    if surface is None:
        return None
    prototype.last_selected_bearing = previous_bearing
    data = prototype._features(observation)
    if data is None:
        return None
    centers, features, global_features = data
    angles = np.asarray(surface["angles"], dtype=float)
    front = np.asarray(surface["front"], dtype=bool)
    scores = np.asarray(surface["scores"], dtype=float)
    order = np.argsort(angles[front])
    target_scores = np.interp(
        centers,
        angles[front][order],
        scores[front][order],
        left=float(scores[front].min()),
        right=float(scores[front].min()),
    )
    scale = float(target_scores.std())
    if scale > 1e-6:
        target_scores = (target_scores - target_scores.mean()) / scale
    return ExpertSample(
        beam_features=features,
        global_features=global_features,
        target_scores=target_scores,
        best_bearing=best_bearing,
        speed=speed,
        source=source,
    )


def _collect_expert(
    b1_vector: np.ndarray,
    starts: list[float],
    seed: int,
    subrays: int,
    max_time_s: float,
) -> list[ExpertSample]:
    policy = StructuredGapPolicy(b1_vector)
    prototype = BeamScorerPolicy(b2_from_b1(b1_vector), hidden=0)
    samples: list[ExpertSample] = []
    for start in starts:
        sim = _make_sim(start, seed, subrays)
        observation = sim.reset()
        policy.reset()
        previous_bearing = 0.0
        while not sim.finished and sim.sim_time < max_time_s:
            command = policy.act(observation)
            debug = policy.last_debug
            if debug.get("valid", 0.0):
                sample = _sample(
                    prototype,
                    policy,
                    observation,
                    previous_bearing,
                    float(debug["best_bearing_rad"]),
                    float(debug["target_speed_mps"]),
                    "expert",
                )
                if sample is not None:
                    samples.append(sample)
                    previous_bearing = sample.best_bearing
            observation = sim.step(command)
    return samples


def _collect_dagger(
    vector: np.ndarray,
    b1_vector: np.ndarray,
    starts: list[float],
    seed: int,
    subrays: int,
    max_time_s: float,
) -> list[ExpertSample]:
    policy = BeamScorerPolicy(vector, hidden=0)
    b1_scorer = StructuredGapPolicy(b1_vector)
    prototype = BeamScorerPolicy(b2_from_b1(b1_vector), hidden=0)
    samples: list[ExpertSample] = []
    for start in starts:
        sim = _make_sim(start, seed, subrays)
        observation = sim.reset()
        policy.reset()
        previous_bearing = 0.0
        while not sim.finished and sim.sim_time < max_time_s:
            data = policy._features(observation)
            command = policy.act(observation)
            if data is not None and policy.last_debug.get("valid", 0.0):
                sample = _sample(
                    prototype,
                    b1_scorer,
                    observation,
                    previous_bearing,
                    float(policy.last_debug["selected_bearing_rad"]),
                    float(policy.last_debug["target_speed_mps"]),
                    "dagger",
                )
                if sample is not None:
                    samples.append(sample)
                    previous_bearing = sample.best_bearing
            observation = sim.step(command)
    return samples


def fit_beam_scores(samples: list[ExpertSample], ridge: float = 1e-2) -> np.ndarray:
    """Ridge regression from B2 beam features to B1's normalised score."""

    x = np.stack([s.beam_features for s in samples]).reshape(-1, N_BEAM_FEATURES)
    y = np.stack([s.target_scores for s in samples]).reshape(-1)
    # Drop the constant bias column: it is redundant with the beam intercept
    # and would absorb the ridge budget without changing the beam ranking.
    x = x[:, 1:]
    gram = x.T @ x + ridge * np.eye(N_BEAM_FEATURES - 1)
    weights = np.zeros(N_BEAM_FEATURES, dtype=float)
    weights[1:] = np.linalg.solve(gram, x.T @ y)
    peak = float(np.max(np.abs(weights)))
    if peak > 4.0:
        weights *= 4.0 / peak
    return weights


def fit_speed_head(
    samples: list[ExpertSample],
    speed_min: float,
    speed_max: float,
    epochs: int = 400,
    lr: float = 0.05,
    l2: float = 1e-4,
) -> tuple[np.ndarray, float]:
    best = np.stack([s.beam_features[np.argmax(s.target_scores)] for s in samples])
    glob = np.stack([s.global_features for s in samples])
    x = np.concatenate([best, glob], axis=1)
    y = np.clip(
        (np.asarray([s.speed for s in samples]) - speed_min)
        / max(speed_max - speed_min, 1e-6),
        0.02,
        0.98,
    )
    n, n_features = x.shape

    def grad_fn(params: np.ndarray) -> np.ndarray:
        w = params[:n_features]
        b = params[n_features]
        logits = x @ w + b
        pred = 1.0 / (1.0 + np.exp(-np.clip(logits, -60, 60)))
        diff = (pred - y) / n
        gw = x.T @ diff + l2 * w
        gb = diff.sum()
        return np.concatenate([gw, np.asarray([gb])])

    params = np.zeros(n_features + 1, dtype=float)
    m = np.zeros_like(params)
    v = np.zeros_like(params)
    beta1, beta2, eps = 0.9, 0.999, 1e-8
    for step in range(1, epochs + 1):
        grad = grad_fn(params)
        m = beta1 * m + (1.0 - beta1) * grad
        v = beta2 * v + (1.0 - beta2) * grad * grad
        params -= lr * (m / (1.0 - beta1**step)) / (
            np.sqrt(v / (1.0 - beta2**step)) + eps
        )
    return params[:n_features], float(params[n_features])


def build_vector(
    b1_vector: np.ndarray,
    beam_w: np.ndarray,
    beam_b: float,
    speed_w: np.ndarray,
    speed_b: float,
    hidden: int = 0,
    speed_cap: float = 2.7,
) -> np.ndarray:
    params = B1Params.from_vector(b1_vector)
    vector = b2_from_b1(b1_vector, hidden=hidden)
    layout = b2_layout(hidden)
    vector[layout["beam_linear_w"] : layout["beam_linear_w"] + beam_w.size] = beam_w
    vector[layout["beam_linear_b"]] = beam_b
    vector[layout["speed_linear_w"] : layout["speed_linear_w"] + speed_w.size] = speed_w
    vector[layout["speed_linear_b"]] = speed_b

    speed_min = params.speed_min_mps
    speed_max = min(params.base_speed_mps, max(speed_cap, speed_min + 0.1))
    scalars = _unit_to_params(
        vector[layout["scalars"] : layout["scalars"] + len(B2_SCALARS)], B2_SCALARS
    )
    scalars[0] = params.front_sector_deg
    scalars[1] = params.min_gap_m
    scalars[2] = 5.0
    scalars[3] = params.steer_gain
    scalars[4] = 0.0
    scalars[5] = params.steer_smoothing
    scalars[6] = params.speed_smoothing
    scalars[7] = speed_min
    scalars[8] = speed_max
    scalars[9] = params.clearance_window_deg
    vector[layout["scalars"] : layout["scalars"] + len(B2_SCALARS)] = _params_to_unit(
        scalars, B2_SCALARS
    )
    return vector


def _agreement(samples: list[ExpertSample], beam_w: np.ndarray) -> float:
    hits = 0
    for sample in samples:
        predicted = int(np.argmax(sample.beam_features @ beam_w))
        actual = int(np.argmax(sample.target_scores))
        hits += predicted == actual
    return hits / max(len(samples), 1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--b1", type=Path, default=Path("algorithms/weights/policy_b1.npz"))
    parser.add_argument("--out", type=Path, default=Path("data/training/b2a_warm.npz"))
    parser.add_argument("--laps", type=int, default=8)
    parser.add_argument("--iterations", type=int, default=3)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--subrays", type=int, default=1)
    parser.add_argument("--epochs", type=int, default=400)
    parser.add_argument("--ridge", type=float, default=1e-2)
    parser.add_argument(
        "--speed-cap",
        type=float,
        default=2.7,
        help="upper speed for the warm start; CMA-ES raises it as the policy improves",
    )
    args = parser.parse_args()

    policy_type, b1_vector, _, meta = load_policy(args.b1)
    if policy_type != "b1":
        raise SystemExit(f"{args.b1} is {policy_type}, not b1")
    print(
        f"expert: {args.b1} lap={meta.get('clean_lap_time_s')} "
        f"contacts={meta.get('contacts')}"
    )

    starts = [i * 40.0 / args.laps for i in range(args.laps)]
    samples = _collect_expert(b1_vector, starts, args.seed, args.subrays, 40.0)
    print(f"collected {len(samples)} expert ticks over {args.laps} start offsets")

    beam_w = fit_beam_scores(samples, ridge=args.ridge)
    beam_b = 0.0
    layout = b2_layout(0)
    warm = build_vector(
        b1_vector,
        beam_w,
        beam_b,
        np.zeros(N_BEAM_FEATURES + N_GLOBAL),
        0.0,
        speed_cap=args.speed_cap,
    )
    scalars = _unit_to_params(
        warm[layout["scalars"] : layout["scalars"] + len(B2_SCALARS)], B2_SCALARS
    )
    speed_w, speed_b = fit_speed_head(
        samples, float(scalars[7]), float(scalars[8]), epochs=args.epochs
    )
    vector = build_vector(
        b1_vector, beam_w, beam_b, speed_w, speed_b, speed_cap=args.speed_cap
    )
    print(f"iter 0 (expert): agreement {_agreement(samples, beam_w) * 100:.1f}%")

    for iteration in range(1, args.iterations + 1):
        dagger = _collect_dagger(
            vector, b1_vector, starts, args.seed, args.subrays, 40.0
        )
        samples = samples + dagger
        beam_w = fit_beam_scores(samples, ridge=args.ridge)
        speed_w, speed_b = fit_speed_head(
            samples, float(scalars[7]), float(scalars[8]), epochs=args.epochs
        )
        vector = build_vector(
            b1_vector, beam_w, beam_b, speed_w, speed_b, speed_cap=args.speed_cap
        )
        print(
            f"iter {iteration} (dagger): {len(dagger)} new ticks, "
            f"agreement {_agreement(dagger, beam_w) * 100:.1f}%"
        )

    print(f"beam_w={np.round(beam_w, 3)}")
    print(f"speed_b={speed_b:+.3f}")

    save_policy(
        args.out,
        "b2",
        vector,
        0,
        {
            "stage": "b2a_warm",
            "source": str(args.b1),
            "clean_lap_time_s": None,
            "contacts": None,
            "samples": len(samples),
            "beam_agreement": _agreement(samples, beam_w),
            "iterations": args.iterations,
            "seed": args.seed,
            "subrays": args.subrays,
        },
    )
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
