from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from dashboard.training_api import STAGES, TrainingError, TrainingManager
from training.progress import RunReporter


def _result(**kwargs: object) -> SimpleNamespace:
    defaults = {
        "lap_time_s": 15.0,
        "contacts": 0,
        "progress_m": 42.0,
        "score": 30.0,
        "fitness": 15.0,
    }
    return SimpleNamespace(**{**defaults, **kwargs})


def test_reporter_writes_history_jsonl(tmp_path: Path) -> None:
    reporter = RunReporter(
        stage="b1",
        generations=2,
        popsize=4,
        workers=1,
        subrays=1,
        seed=1,
        out=tmp_path / "out.npz",
        data_dir=tmp_path,
    )
    reporter.start()
    reporter.initial(_result(lap_time_s=16.0), 1.0)
    for generation in range(2):
        reporter.update(
            generation=generation,
            result=_result(lap_time_s=15.0 - generation),
            best_lap=15.0 - generation,
            best_contacts=0,
            best_generation=generation,
            best_progress=42.0,
            best_score=30.0,
            sigma=0.2,
            track_length_m=60.0,
            generation_seconds=1.5,
        )
    records = [
        json.loads(line)
        for line in (tmp_path / "b1_history.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    assert len(records) == 3
    assert records[0]["phase"] == "initial"
    assert [r["generation"] for r in records[1:]] == [0, 1]
    assert records[2]["best_lap_s"] == 14.0
    assert records[2]["sigma"] == 0.2


def test_reporter_start_truncates_previous_history(tmp_path: Path) -> None:
    reporter = RunReporter(
        stage="b2a",
        generations=1,
        popsize=4,
        workers=1,
        subrays=1,
        seed=1,
        out=tmp_path / "out.npz",
        data_dir=tmp_path,
    )
    reporter.start()
    reporter.update(
        generation=0,
        result=_result(),
        best_lap=15.0,
        best_contacts=0,
        best_generation=0,
        best_progress=42.0,
        best_score=30.0,
        sigma=0.2,
        track_length_m=60.0,
        generation_seconds=1.0,
    )
    reporter.start()
    assert (tmp_path / "b2a_history.jsonl").read_text(encoding="utf-8") == ""


def test_training_manager_rejects_unknown_stage(tmp_path: Path) -> None:
    manager = TrainingManager(data_dir=tmp_path)
    with pytest.raises(TrainingError):
        manager.start({"stage": "nope"})


def test_training_manager_status_and_history(tmp_path: Path) -> None:
    manager = TrainingManager(data_dir=tmp_path)
    (tmp_path / "b1_status.json").write_text(
        json.dumps({"stage": "b1", "phase": "running", "percent": 50.0}),
        encoding="utf-8",
    )
    (tmp_path / "b1_history.jsonl").write_text(
        '{"generation": 0, "phase": "running"}\nnot json\n{"generation": 1, "phase": "running"}\n',
        encoding="utf-8",
    )
    status = manager.status("b1")
    assert status["phase"] == "running"
    assert status["percent"] == 50.0
    assert status["process_running"] is False
    records = manager.history("b1")
    assert [r["generation"] for r in records] == [0, 1]
    assert manager.status("b2b")["phase"] == "idle"
    assert manager.best_path("b1") is None
    (tmp_path / "b1_best.npz").write_bytes(b"fake")
    assert manager.best_path("b1") is not None


def test_training_manager_stop_unknown_stage_is_ok(tmp_path: Path) -> None:
    manager = TrainingManager(data_dir=tmp_path)
    assert manager.stop("b1")["stopped"] is False


def test_all_stages_are_known(tmp_path: Path) -> None:
    manager = TrainingManager(data_dir=tmp_path)
    assert set(STAGES) == {"b1", "b2a", "b2b"}
    assert set(manager.status()["runs"][0].keys()) >= {"stage", "phase"}
