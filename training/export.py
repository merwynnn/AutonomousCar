"""Atomic export of the current best evolved policy."""

from __future__ import annotations

import argparse
import json
import os
import tempfile
from pathlib import Path
from typing import Any

import numpy as np
from algorithms.learned_gap import B1_DEFAULT, WEIGHTS_PATH, b2_default_vector

DEFAULT_OUT = WEIGHTS_PATH


def save_policy(
    path: Path | str,
    policy_type: str,
    vector: np.ndarray,
    hidden: int = 0,
    meta: dict[str, Any] | None = None,
) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "policy_type": np.asarray(policy_type),
        "vector": np.asarray(vector, dtype=float),
        "hidden": np.asarray(int(hidden)),
        "meta_json": np.asarray(json.dumps(meta or {}, sort_keys=True)),
    }
    with tempfile.NamedTemporaryFile(
        suffix=".npz", dir=path.parent, delete=False
    ) as handle:
        temporary = Path(handle.name)
    try:
        np.savez(temporary, **payload)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()
    return path


def load_policy(
    path: Path | str,
) -> tuple[str, np.ndarray, int, dict[str, Any]]:
    with np.load(Path(path), allow_pickle=False) as data:
        policy_type = str(np.asarray(data["policy_type"]).item())
        vector = np.asarray(data["vector"], dtype=float)
        hidden = int(np.asarray(data["hidden"]).item())
        meta = json.loads(str(np.asarray(data["meta_json"]).item()))
    return policy_type, vector, hidden, meta


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--policy-type", choices=("b1", "b2"), default="b1")
    parser.add_argument("--hidden", type=int, default=0)
    parser.add_argument("--from", dest="source", type=Path, default=None)
    parser.add_argument("--note", default="manual export")
    args = parser.parse_args()

    if args.source is not None:
        policy_type, vector, hidden, meta = load_policy(args.source)
        meta = {**meta, "note": args.note}
    elif args.policy_type == "b2":
        policy_type, vector, hidden, meta = (
            "b2",
            b2_default_vector(args.hidden),
            args.hidden,
            {"note": args.note, "source": "default"},
        )
    else:
        policy_type, vector, hidden, meta = (
            "b1",
            B1_DEFAULT.copy(),
            0,
            {"note": args.note, "source": "default"},
        )
    path = save_policy(args.out, policy_type, vector, hidden, meta)
    print(f"exported {policy_type} policy to {path}")


if __name__ == "__main__":
    main()
