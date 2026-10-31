from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_algorithms_do_not_import_the_simulator() -> None:
    for path in (ROOT / "algorithms").glob("*.py"):
        source = path.read_text(encoding="utf-8")
        assert "import simulator" not in source, f"{path} imports simulator"
        assert "from simulator" not in source, f"{path} imports simulator"
        assert "pybullet" not in source, f"{path} imports pybullet"
