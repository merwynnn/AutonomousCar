"""Evaluate an evolved policy on the reference or generated tracks.

Examples::

    uv run python -m training.evaluate --policy algorithms/weights/policy.npz \
        --subrays 1 3

    # held-out random-track success rate
    uv run python -m training.evaluate --policy algorithms/weights/policy.npz \
        --random-sweep 10 --max-time 60
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from training.env import RolloutConfig, TrackSpec, run_vector
from training.export import load_policy


def _format_result(label: str, result) -> bool:
    lap = f"{result.lap_time_s:.2f} s" if result.lap_time_s is not None else "DNF"
    clean = bool(result.finished and result.contacts == 0)
    print(
        f"{label}: lap {lap} | contacts {result.contacts} | "
        f"top {result.top_speed_mps:.2f} m/s | mean {result.mean_speed_mps:.2f} m/s | "
        f"min wall {result.min_wall_distance_m:.3f} m | progress {result.progress_m:.2f} m"
        + ("" if clean else "  <-- NOT CLEAN")
    )
    return clean


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--policy",
        type=Path,
        default=Path("algorithms/weights/policy.npz"),
    )
    parser.add_argument("--subrays", type=int, nargs="+", default=[1, 3])
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--max-time", type=float, default=32.0)
    parser.add_argument(
        "--random-track",
        action="store_true",
        help="evaluate on one generated track instead of the reference circuit",
    )
    parser.add_argument("--track-seed", type=int, default=123)
    parser.add_argument("--object-density", type=float, default=0.5)
    parser.add_argument("--object-scale", type=float, default=1.4)
    parser.add_argument("--object-scale-jitter", type=float, default=0.3)
    parser.add_argument("--track-width", type=float, default=0.9)
    parser.add_argument(
        "--random-sweep",
        type=int,
        default=0,
        help="evaluate this many generated tracks and report the clean rate",
    )
    parser.add_argument(
        "--object-density-range", type=float, nargs=2, default=[0.0, 0.9]
    )
    parser.add_argument(
        "--object-scale-range", type=float, nargs=2, default=[1.0, 1.8]
    )
    parser.add_argument(
        "--track-width-range", type=float, nargs=2, default=[0.85, 1.0]
    )
    args = parser.parse_args()

    policy_type, vector, hidden, meta = load_policy(args.policy)
    print(
        f"{args.policy}: {policy_type} dim={vector.size} hidden={hidden} "
        f"recorded_lap={meta.get('clean_lap_time_s')} contacts={meta.get('contacts')}"
    )
    config = RolloutConfig(
        policy_type=policy_type,
        hidden=hidden,
        subrays=args.subrays[0],
        seed=args.seed,
        max_time_s=args.max_time,
    )

    if args.random_sweep > 0:
        rng = np.random.default_rng(args.track_seed)
        clean = 0
        times: list[float] = []
        for index in range(args.random_sweep):
            spec = TrackSpec(
                seed=int(rng.integers(0, 2**31 - 1)),
                width_m=float(rng.uniform(*args.track_width_range)),
                object_density=float(rng.uniform(*args.object_density_range)),
                object_scale=float(rng.uniform(*args.object_scale_range)),
                object_scale_jitter=args.object_scale_jitter,
            )
            result = run_vector(vector, config, track_spec=spec)
            label = (
                f"track {index:02d} seed {spec.seed} "
                f"d{spec.object_density:.2f} s{spec.object_scale:.2f} w{spec.width_m:.2f}"
            )
            if _format_result(label, result):
                clean += 1
                if result.lap_time_s is not None:
                    times.append(result.lap_time_s)
        print(
            f"clean {clean}/{args.random_sweep} | "
            f"mean clean lap {np.mean(times):.2f} s" if times else
            f"clean {clean}/{args.random_sweep} | no clean laps"
        )
        return

    track_spec = (
        TrackSpec(
            seed=args.track_seed,
            width_m=args.track_width,
            object_density=args.object_density,
            object_scale=args.object_scale,
            object_scale_jitter=args.object_scale_jitter,
        )
        if args.random_track
        else None
    )
    if track_spec is not None:
        print(
            f"track: seed {track_spec.seed} | density {track_spec.object_density:.2f} | "
            f"size {track_spec.object_scale:.2f}±{track_spec.object_scale_jitter:.2f} | "
            f"width {track_spec.width_m:.2f} m"
        )
    else:
        print("track: reference")
    for subrays in args.subrays:
        config = RolloutConfig(
            policy_type=policy_type,
            hidden=hidden,
            subrays=subrays,
            seed=args.seed,
            max_time_s=args.max_time,
        )
        result = run_vector(vector, config, track_spec=track_spec)
        _format_result(f"sub-rays {subrays}", result)


if __name__ == "__main__":
    main()
