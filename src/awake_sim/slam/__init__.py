"""SLAM bindings, Python reference implementation and evaluation.

The production pipeline is the C++ extension built from ``cpp/``. A pure-Python
reference implementation lives alongside it for tests and teaching.
"""

from __future__ import annotations

try:  # pragma: no cover - depends on the compiled extension
    from awake_sim import _awake_slam as core  # type: ignore[attr-defined]

    HAVE_CPP = True
except Exception:  # pragma: no cover - exercised when the extension is absent
    core = None
    HAVE_CPP = False

__all__ = ["core", "HAVE_CPP"]
