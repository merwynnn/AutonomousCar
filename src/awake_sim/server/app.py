"""FastAPI application: REST config/status plus WebSocket telemetry."""

from __future__ import annotations

import asyncio
import contextlib
from collections.abc import AsyncIterator

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from awake_sim import __version__
from awake_sim.server.pybullet_sim import PyBulletSimulation
from awake_sim.slam import HAVE_CPP

manager = PyBulletSimulation()


@contextlib.asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    manager.start()
    try:
        yield
    finally:
        await manager.stop()


app = FastAPI(
    title="AWAKE 2027 Simulator",
    version=__version__,
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/health")
def health() -> dict:
    return {
        "status": "ok",
        "version": __version__,
        "mode": manager.status()["mode"],
        "slam_cpp": HAVE_CPP,
    }


@app.get("/api/status")
def status() -> dict:
    return manager.status()


@app.get("/api/scenarios")
def scenarios() -> dict:
    return {
        "scenarios": [
            {
                "id": "test_track_baseline",
                "name": "Test track — baseline",
                "track": "test_track",
                "controller": "wall-follow (lidar only)",
                "realism": manager.scenario.realism,
            }
        ]
    }


@app.get("/api/track")
def track() -> dict:
    hello = manager.hello()
    return hello["track"]


@app.websocket("/ws/telemetry")
async def telemetry(websocket: WebSocket) -> None:
    await websocket.accept()
    queue = manager.subscribe()
    try:
        await websocket.send_json(manager.hello())
        await websocket.send_json(manager.state_message())
        await websocket.send_json(manager.scan_message())
        if manager.enable_slam:
            await websocket.send_json(manager.slam_message(include_map=True))

        async def receive_commands() -> None:
            while True:
                message = await websocket.receive_json()
                if isinstance(message, dict):
                    manager.handle_command(message)

        receiver = asyncio.create_task(receive_commands())
        try:
            while True:
                message = await queue.get()
                await websocket.send_json(message)
        finally:
            receiver.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await receiver
    except WebSocketDisconnect:
        pass
    finally:
        manager.unsubscribe(queue)
