# ADR-0004 — Pin Python 3.11

- Status: accepted
- Date: 2026-10-30

## Context

The system Python is 3.14. PyBullet 3.2.7 publishes wheels only up to cp311, and
building PyBullet from source would add fragility for little benefit.

## Decision

Pin the project to Python 3.11 using `uv` (`.python-version`). Use the uv-managed
interpreter for both development and the C++ extension build.

## Rationale

- PyBullet wheels work out of the box on 3.11.
- NumPy/SciPy/FastAPI/pybind11 are mature on 3.11.
- The Raspberry Pi stack can also use Python 3.11/3.12, so controller code stays
  compatible.
- CI and onboarding become reproducible.

## Consequences

- Do not use system `python3` directly; use `uv run`.
- Revisit when PyBullet publishes newer wheels or if we switch physics engine.
- Document the version in README, AGENTS and runbook.
