# ADR-0002 — C++17 SLAM core with pybind11

- Status: accepted
- Date: 2026-10-30

## Context

The SLAM pipeline must run at 10 Hz on 720-beam scans while searching a 3-DOF
pose window. Pure Python is too slow. The code should also be portable to a
Raspberry Pi later.

## Decision

Implement the production SLAM pipeline in C++17 with Eigen, exposed to Python via
pybind11. Keep a Python reference implementation for tests and teaching.

## Rationale

- C++ + Eigen gives the required performance and is the standard robotics stack.
- pybind11 provides a clean NumPy interface and builds on both x86 and ARM.
- Eigen is header-only and available via `libeigen3-dev`.
- The C++ core has no ROS, CUDA or GUI dependency, so it can later run on the Pi.
- A Python reference keeps the algorithms inspectable and cross-checkable.

## Consequences

- The project needs a C++ toolchain (`build-essential`, CMake, Ninja, Eigen).
- Build and packaging add complexity; `awake-sim build-cpp` wraps it.
- The C++ core must expose diagnostics for visualization; otherwise the pipeline
  becomes a black box.
- If the extension is missing, SLAM modes are disabled rather than silently
  replaced by a perfect localizer.
