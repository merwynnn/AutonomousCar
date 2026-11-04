"""Evaluate an evolved policy on the reference one-lap scenario.

Example::

    uv run python -m training.evaluate --policy algorithms/weights/policy.npz \
        --subrays 1 3
"""

from __future__ import annotations

import argparse
from pathlib import Path

from training.env import RolloutConfig, run_vector
from training.export import load_policy


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
    args = parser.parse_args()

    policy_type, vector, hidden, meta = load_policy(args.policy)
    print(
        f"{args.policy}: {policy_type} dim={vector.size} hidden={hidden} "
        f"recorded_lap={meta.get('clean_lap_time_s')} contacts={meta.get('contacts')}"
    )
    for subrays in args.subrays:
        result = run_vector(
            vector,
            RolloutConfig(
                policy_type=policy_type,
                hidden=hidden,
                subrays=subrays,
                seed=args.seed,
                max_time_s=args.max_time,
            ),
        )
        lap = f"{result.lap_time_s:.2f} s" if result.lap_time_s is not None else "DNF"
        print(
            f"sub-rays {subrays}: lap {lap} | contacts {result.contacts} | "
            f"top {result.top_speed_mps:.2f} m/s | mean {result.mean_speed_mps:.2f} m/s | "
            f"min wall {result.min_wall_distance_m:.3f} m | progress {result.progress_m:.2f} m"
        )


if __name__ == "__main__":
    main()
