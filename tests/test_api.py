from __future__ import annotations

from fastapi.testclient import TestClient

from awake_sim.server.app import app


def test_health() -> None:
    with TestClient(app) as client:
        response = client.get("/api/health")
        assert response.status_code == 200
        body = response.json()
        assert body["status"] == "ok"
        assert body["mode"] == "demo"


def test_status_and_track() -> None:
    with TestClient(app) as client:
        status = client.get("/api/status").json()
        assert status["mode"] == "demo"
        assert status["track_length_m"] > 15.0

        track = client.get("/api/track").json()
        assert len(track["centerline"]) > 100
        assert track["width_m"] == 0.9


def test_websocket_hello_and_state() -> None:
    with TestClient(app) as client, client.websocket_connect("/ws/telemetry") as ws:
        hello = ws.receive_json()
        assert hello["type"] == "hello"
        assert hello["protocol"] == 1
        assert hello["mode"] == "demo"
        assert hello["track"]["width_m"] == 0.9
        assert hello["sensors"]["lidar"]["beams"] == 720

        state = ws.receive_json()
        assert state["type"] in {"state", "scan"}
        if state["type"] == "scan":
            state = ws.receive_json()
        assert state["type"] == "state"
        assert state["race"]["laps"] == 5


def test_start_command() -> None:
    with TestClient(app) as client, client.websocket_connect("/ws/telemetry") as ws:
        ws.receive_json()  # hello
        ws.receive_json()  # initial state
        ws.send_json({"type": "run.start"})
        # The next state should be running (or already have advanced).
        for _ in range(10):
            msg = ws.receive_json()
            if msg["type"] == "state":
                assert msg["finished"] is False
                break
