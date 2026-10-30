# ADR-0003 — Non-ideal sensors are the default

- Status: accepted
- Date: 2026-10-30

## Context

A simulator with perfect sensors makes strategies look better than they will be on
the real car. The team explicitly requires realistic, non-perfect sensors, and the
competition environment includes noise, latency, dropouts and motion distortion.

## Decision

Realistic sensors are the default. Ideal sensors are a debug-only mode. Every
sensor message carries `t_measure` and `t_available`; controllers and SLAM only
see messages available at the current time. Ground truth is reserved for the
referee, logger and evaluation.

## Rationale

- Strategy rankings must include the cost of sensor imperfection.
- SLAM robustness cannot be evaluated with clean scans.
- Latency is a first-order effect at racing speed.
- A central latency bus prevents accidental future-data leakage.

## Consequences

- All controllers must handle missing/stale data.
- Tests must validate noise, dropout and latency behaviour.
- Results are noisier; experiments need multiple seeds.
- The dashboard must make imperfections visible so they are not mistaken for bugs.
- `ideal` mode is clearly labelled and excluded from headline results.
