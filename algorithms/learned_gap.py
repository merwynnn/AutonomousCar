"""CMA-ES-evolved lidar gap controllers.

This module is the portable runtime side of the training pipeline in
``training/``. It imports only NumPy and ``onboard.api`` so the same code can
run in the simulator, on the dashboard and on the Raspberry Pi.

Two policy families are supported, selected by ``policy_type`` in the weights
file:

``b1``
    A parameterised version of the furthest-gap controller. CMA-ES tunes the
    scan processing, scoring and speed schedule directly.

``b2``
    A learned beam scorer. A small per-beam network scores downsampled lidar
    beams; the highest-scoring beam gives the steering direction and a small
    speed head gives the target speed. A linear scorer and a residual hidden
    layer share the same parameter layout so the search can grow without
    changing the runtime interface.

If no weights file exists the controller falls back to hand-tuned ``b1``
parameters, so it is selectable in the dashboard even before training.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from onboard.api import Actuation, Observation

WEIGHTS_PATH = Path(__file__).with_name("weights") / "policy.npz"

# ---------------------------------------------------------------------------
# Shared parameter helpers


@dataclass(frozen=True)
class ParamSpec:
    name: str
    low: float
    high: float
    default: float


def _coerce_odd(value: float, upper: int | None = None) -> int:
    count = max(1, int(round(value)))
    if count % 2 == 0:
        count += 1
    if upper is not None and count > upper:
        count = upper if upper % 2 == 1 else max(1, upper - 1)
    return count


def _clip(value: float, low: float, high: float) -> float:
    return float(min(high, max(low, value)))


def _sigmoid(value: float) -> float:
    value = _clip(value, -60.0, 60.0)
    return float(1.0 / (1.0 + np.exp(-value)))


def _unit_to_params(vector: np.ndarray, specs: tuple[ParamSpec, ...]) -> np.ndarray:
    """Map an unconstrained/unit vector to physical parameter bounds."""

    vector = np.asarray(vector, dtype=float)
    low = np.array([spec.low for spec in specs], dtype=float)
    high = np.array([spec.high for spec in specs], dtype=float)
    return low + np.clip(vector, 0.0, 1.0) * (high - low)


def _params_to_unit(params: np.ndarray, specs: tuple[ParamSpec, ...]) -> np.ndarray:
    low = np.array([spec.low for spec in specs], dtype=float)
    high = np.array([spec.high for spec in specs], dtype=float)
    return np.clip((np.asarray(params, dtype=float) - low) / (high - low), 0.0, 1.0)


# ---------------------------------------------------------------------------
# B1: parameterised furthest-gap controller

B1_SPECS: tuple[ParamSpec, ...] = (
    ParamSpec("front_sector_deg", 55.0, 135.0, 100.0),
    ParamSpec("clearance_window_deg", 20.0, 120.0, 60.0),
    ParamSpec("min_gap_m", 0.20, 1.20, 0.60),
    ParamSpec("smoothing_beams", 1.0, 11.0, 7.0),
    ParamSpec("angle_change_weight", 0.0, 1.5, 0.45),
    ParamSpec("clearance_weight", 0.0, 2.0, 1.0),
    ParamSpec("range_weight", 0.0, 1.0, 0.0),
    ParamSpec("gap_width_weight", 0.0, 2.0, 0.50),
    ParamSpec("center_bias_weight", 0.0, 1.0, 0.0),
    ParamSpec("aim_blend", 0.0, 1.0, 0.0),
    ParamSpec("max_steering_rad", 0.25, 0.49, 0.49),
    ParamSpec("steer_smoothing", 2.0, 30.0, 10.0),
    ParamSpec("steer_gain", 0.50, 2.0, 1.0),
    ParamSpec("base_speed_mps", 0.80, 4.0, 2.20),
    ParamSpec("clearance_slow_1_m", 0.20, 1.0, 0.55),
    ParamSpec("clearance_slow_1_speed", 0.30, 2.0, 0.50),
    ParamSpec("clearance_slow_2_m", 0.50, 2.0, 0.90),
    ParamSpec("clearance_slow_2_speed", 0.80, 3.0, 0.90),
    ParamSpec("gap_width_speed_gain", 0.0, 4.0, 1.0),
    ParamSpec("curvature_speed_gain", 0.0, 4.0, 0.80),
    ParamSpec("speed_smoothing", 2.0, 20.0, 8.0),
    ParamSpec("throttle_max_speed", 2.0, 5.5, 4.0),
    ParamSpec("speed_min_mps", 0.20, 1.0, 0.40),
)

B1_DEFAULT = np.array([spec.default for spec in B1_SPECS], dtype=float)
B1_LOW = np.array([spec.low for spec in B1_SPECS], dtype=float)
B1_HIGH = np.array([spec.high for spec in B1_SPECS], dtype=float)


@dataclass
class B1Params:
    front_sector_deg: float
    clearance_window_deg: float
    min_gap_m: float
    smoothing_beams: float
    angle_change_weight: float
    clearance_weight: float
    range_weight: float
    gap_width_weight: float
    center_bias_weight: float
    aim_blend: float
    max_steering_rad: float
    steer_smoothing: float
    steer_gain: float
    base_speed_mps: float
    clearance_slow_1_m: float
    clearance_slow_1_speed: float
    clearance_slow_2_m: float
    clearance_slow_2_speed: float
    gap_width_speed_gain: float
    curvature_speed_gain: float
    speed_smoothing: float
    throttle_max_speed: float
    speed_min_mps: float

    @classmethod
    def from_vector(cls, vector: np.ndarray | None = None) -> B1Params:
        values = B1_DEFAULT if vector is None else _unit_to_params(vector, B1_SPECS)
        return cls(**{spec.name: float(values[i]) for i, spec in enumerate(B1_SPECS)})


# ---------------------------------------------------------------------------
# B2: learned beam scorer

N_BEAMS = 36
N_GLOBAL = 7
# Per-beam feature layout.
BEAM_FEATURES = (
    "bias",
    "range_norm",
    "inv_range",
    "sin_bearing",
    "cos_bearing",
    "clearance_norm",
    "free_fraction",
    "left_free",
    "right_free",
    "valid",
    "prev_alignment",
    "angle_change",
)
N_BEAM_FEATURES = len(BEAM_FEATURES)

B2_SCALARS: tuple[ParamSpec, ...] = (
    ParamSpec("front_sector_deg", 70.0, 125.0, 100.0),
    ParamSpec("gap_threshold_m", 0.25, 1.20, 0.60),
    ParamSpec("max_range_m", 2.0, 8.0, 5.0),
    ParamSpec("steer_gain", 0.40, 2.2, 1.0),
    ParamSpec("steer_bias", -0.20, 0.20, 0.0),
    ParamSpec("steer_smoothing", 2.0, 30.0, 12.0),
    ParamSpec("speed_smoothing", 2.0, 20.0, 8.0),
    ParamSpec("speed_min_mps", 0.20, 1.00, 0.40),
    ParamSpec("speed_max_mps", 1.50, 4.00, 3.20),
    ParamSpec("clearance_window_deg", 10.0, 120.0, 55.0),
)

B2_SCALAR_DEFAULT = np.array([spec.default for spec in B2_SCALARS], dtype=float)
B2_SCALAR_LOW = np.array([spec.low for spec in B2_SCALARS], dtype=float)
B2_SCALAR_HIGH = np.array([spec.high for spec in B2_SCALARS], dtype=float)


def b2_layout(hidden: int) -> dict[str, int]:
    """Return the slice layout for a B2 parameter vector."""

    sizes = {
        "beam_linear_w": N_BEAM_FEATURES,
        "beam_linear_b": 1,
        "beam_res_w1": N_BEAM_FEATURES * hidden,
        "beam_res_b1": hidden,
        "beam_res_w2": hidden,
        "beam_res_b2": 1,
        "speed_linear_w": N_BEAM_FEATURES + N_GLOBAL,
        "speed_linear_b": 1,
        "speed_res_w1": (N_BEAM_FEATURES + N_GLOBAL) * hidden,
        "speed_res_b1": hidden,
        "speed_res_w2": hidden,
        "speed_res_b2": 1,
        "scalars": len(B2_SCALARS),
    }
    layout: dict[str, int] = {}
    offset = 0
    for name, size in sizes.items():
        layout[name] = offset
        offset += size
    layout["_size"] = offset
    layout["_hidden"] = hidden
    return layout


@dataclass
class B2Params:
    beam_linear_w: np.ndarray
    beam_linear_b: float
    beam_res_w1: np.ndarray
    beam_res_b1: np.ndarray
    beam_res_w2: np.ndarray
    beam_res_b2: float
    speed_linear_w: np.ndarray
    speed_linear_b: float
    speed_res_w1: np.ndarray
    speed_res_b1: np.ndarray
    speed_res_w2: np.ndarray
    speed_res_b2: float
    scalars: np.ndarray
    hidden: int

    @classmethod
    def from_vector(cls, vector: np.ndarray, hidden: int) -> B2Params:
        layout = b2_layout(hidden)
        v = np.asarray(vector, dtype=float)
        h = hidden

        def take(name: str) -> np.ndarray:
            start = layout[name]
            names = list(layout)
            index = names.index(name)
            next_name = names[index + 1]
            size = layout[next_name] - layout[name]
            return v[start : start + size].copy()

        beam_w = take("beam_linear_w").reshape(N_BEAM_FEATURES)
        beam_b = float(v[layout["beam_linear_b"]])
        beam_w1 = take("beam_res_w1").reshape(N_BEAM_FEATURES, h) if h else np.zeros((N_BEAM_FEATURES, 0))
        beam_b1 = take("beam_res_b1") if h else np.zeros(0)
        beam_w2 = take("beam_res_w2") if h else np.zeros(0)
        beam_b2 = float(v[layout["beam_res_b2"]]) if h else 0.0
        speed_w = take("speed_linear_w").reshape(N_BEAM_FEATURES + N_GLOBAL)
        speed_b = float(v[layout["speed_linear_b"]])
        speed_w1 = (
            take("speed_res_w1").reshape(N_BEAM_FEATURES + N_GLOBAL, h)
            if h
            else np.zeros((N_BEAM_FEATURES + N_GLOBAL, 0))
        )
        speed_b1 = take("speed_res_b1") if h else np.zeros(0)
        speed_w2 = take("speed_res_w2") if h else np.zeros(0)
        speed_b2 = float(v[layout["speed_res_b2"]]) if h else 0.0
        scalars = take("scalars")
        return cls(
            beam_linear_w=beam_w,
            beam_linear_b=beam_b,
            beam_res_w1=beam_w1,
            beam_res_b1=beam_b1,
            beam_res_w2=beam_w2,
            beam_res_b2=beam_b2,
            speed_linear_w=speed_w,
            speed_linear_b=speed_b,
            speed_res_w1=speed_w1,
            speed_res_b1=speed_b1,
            speed_res_w2=speed_w2,
            speed_res_b2=speed_b2,
            scalars=scalars,
            hidden=h,
        )


def b2_default_vector(hidden: int = 0) -> np.ndarray:
    """A safe linear scorer initialised from the hand-tuned B1 behaviour."""

    layout = b2_layout(hidden)
    v = np.zeros(layout["_size"], dtype=float)
    # Per-beam linear score approximates B1: clearance dominates, range and gap
    # width help, invalid beams are pushed down, and bearing is close to neutral.
    beam_w = np.array(
        [0.0, 0.35, 0.0, 0.0, 0.0, 1.10, 0.45, 0.0, 0.0, 0.0, 0.20, 0.60],
        dtype=float,
    )
    beam_w = beam_w / max(1e-9, np.linalg.norm(beam_w))
    v[layout["beam_linear_w"] : layout["beam_linear_w"] + N_BEAM_FEATURES] = beam_w
    v[layout["beam_linear_b"]] = 0.0
    # Speed linear head: start near 2.2 m/s, with a small gap-width boost.
    speed_w = np.zeros(N_BEAM_FEATURES + N_GLOBAL, dtype=float)
    speed_w[5] = 0.35  # clearance_norm
    speed_w[6] = 0.30  # free_fraction
    v[layout["speed_linear_w"] : layout["speed_linear_w"] + N_BEAM_FEATURES + N_GLOBAL] = speed_w
    v[layout["speed_linear_b"]] = -0.15
    v[layout["scalars"] : layout["scalars"] + len(B2_SCALARS)] = _params_to_unit(
        B2_SCALAR_DEFAULT, B2_SCALARS
    )
    return v


def b2_from_b1(b1_vector: np.ndarray, hidden: int = 0) -> np.ndarray:
    """Initialise the linear beam scorer from tuned B1 parameters.

    The score scale is arbitrary (only the argmax matters), but keeping it in
    metres makes the initialisation close to the structured controller and
    keeps the numbers inside CMA-ES bounds.
    """

    params = B1Params.from_vector(b1_vector)
    vector = b2_default_vector(hidden)
    layout = b2_layout(hidden)
    max_range = 5.0
    beam_w = np.zeros(N_BEAM_FEATURES, dtype=float)
    beam_w[1] = params.range_weight * max_range
    beam_w[5] = params.clearance_weight * max_range
    beam_w[6] = params.gap_width_weight * 2.0
    beam_w[9] = 0.0
    beam_w[10] = 0.20
    beam_w[11] = params.angle_change_weight * 2.0
    beam_w = np.clip(beam_w, -4.0, 4.0)
    vector[layout["beam_linear_w"] : layout["beam_linear_w"] + N_BEAM_FEATURES] = beam_w
    vector[layout["beam_linear_b"]] = 0.0

    speed_min = params.speed_min_mps
    speed_max = min(4.0, max(params.base_speed_mps + 0.8, 2.0))
    scalar_values = np.array(
        [
            params.front_sector_deg,
            params.min_gap_m,
            max_range,
            params.steer_gain,
            0.0,
            params.steer_smoothing,
            params.speed_smoothing,
            speed_min,
            speed_max,
            params.clearance_window_deg,
        ],
        dtype=float,
    )
    vector[layout["scalars"] : layout["scalars"] + len(B2_SCALARS)] = _params_to_unit(
        scalar_values, B2_SCALARS
    )
    ratio = (params.base_speed_mps - speed_min) / max(speed_max - speed_min, 1e-6)
    ratio = float(np.clip(ratio, 0.02, 0.98))
    vector[layout["speed_linear_b"]] = float(np.log(ratio / (1.0 - ratio)))
    speed_w = np.zeros(N_BEAM_FEATURES + N_GLOBAL, dtype=float)
    speed_w[5] = 0.20
    speed_w[6] = 0.20
    vector[layout["speed_linear_w"] : layout["speed_linear_w"] + N_BEAM_FEATURES + N_GLOBAL] = speed_w
    return vector


def b2_expand_to_hidden(linear_vector: np.ndarray, hidden: int, seed: int = 7) -> np.ndarray:
    """Expand a linear B2 vector to a residual-hidden vector.

    The residual output weights are left at zero, so the expanded policy starts
    numerically identical to the linear policy.
    """

    rng = np.random.default_rng(seed)
    linear_layout = b2_layout(0)
    layout = b2_layout(hidden)
    v = np.zeros(layout["_size"], dtype=float)
    # Copy the linear parts.
    v[layout["beam_linear_w"] : layout["beam_linear_w"] + N_BEAM_FEATURES] = linear_vector[
        linear_layout["beam_linear_w"] : linear_layout["beam_linear_w"] + N_BEAM_FEATURES
    ]
    v[layout["beam_linear_b"]] = linear_vector[linear_layout["beam_linear_b"]]
    v[layout["speed_linear_w"] : layout["speed_linear_w"] + N_BEAM_FEATURES + N_GLOBAL] = (
        linear_vector[
            linear_layout["speed_linear_w"] : linear_layout["speed_linear_w"]
            + N_BEAM_FEATURES
            + N_GLOBAL
        ]
    )
    v[layout["speed_linear_b"]] = linear_vector[linear_layout["speed_linear_b"]]
    v[layout["scalars"] : layout["scalars"] + len(B2_SCALARS)] = linear_vector[
        linear_layout["scalars"] : linear_layout["scalars"] + len(B2_SCALARS)
    ]
    # Small random hidden weights; zero output weights preserve the linear policy.
    beam_count = N_BEAM_FEATURES * hidden
    v[layout["beam_res_w1"] : layout["beam_res_w1"] + beam_count] = rng.normal(
        0.0, 0.05, beam_count
    )
    speed_count = (N_BEAM_FEATURES + N_GLOBAL) * hidden
    v[layout["speed_res_w1"] : layout["speed_res_w1"] + speed_count] = rng.normal(
        0.0, 0.05, speed_count
    )
    return v


# ---------------------------------------------------------------------------
# Feature extraction shared by training and runtime


@dataclass(frozen=True)
class ScanFeatures:
    angles: np.ndarray
    ranges: np.ndarray
    valid: np.ndarray
    clearance: np.ndarray
    free_fraction: np.ndarray
    left_free: np.ndarray
    right_free: np.ndarray


def _front_scan(observation: Observation, front_sector_rad: float) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    scan = observation.lidar_current
    if scan is None:
        return None
    angles = np.asarray(scan.angles_rad, dtype=float)
    ranges = np.asarray(scan.ranges_m, dtype=float)
    valid = np.asarray(scan.valid, dtype=bool) & np.isfinite(ranges) & (ranges > 0.05)
    if angles.size == 0 or ranges.size != angles.size or valid.size != angles.size:
        return None
    front = valid & (np.abs(angles) <= front_sector_rad)
    if not np.any(front):
        return None
    return angles, ranges, valid


def _resample_front(
    angles: np.ndarray,
    ranges: np.ndarray,
    valid: np.ndarray,
    front_sector_rad: float,
    n_beams: int,
    max_range: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Min-pool the front scan into ``n_beams`` equal angular bins."""

    edges = np.linspace(-front_sector_rad, front_sector_rad, n_beams + 1)
    centers = 0.5 * (edges[:-1] + edges[1:])
    out_ranges = np.full(n_beams, max_range, dtype=float)
    out_valid = np.zeros(n_beams, dtype=bool)
    for i in range(n_beams):
        mask = valid & (angles >= edges[i]) & (angles < edges[i + 1])
        if np.any(mask):
            value = float(np.min(ranges[mask]))
            out_ranges[i] = min(value, max_range)
            out_valid[i] = True
    return centers, out_ranges, out_valid


def _window_stats(
    values: np.ndarray,
    valid: np.ndarray,
    window: int,
    max_range: float,
    threshold: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    n = values.size
    pad = window // 2
    padded = np.concatenate([values[-pad:], values, values[:pad]]) if pad else values
    padded_valid = np.concatenate([valid[-pad:], valid, valid[:pad]]) if pad else valid
    windows = np.lib.stride_tricks.sliding_window_view(padded, window)
    windows_valid = np.lib.stride_tricks.sliding_window_view(padded_valid, window)
    clearance = np.where(windows_valid, windows, max_range).min(axis=1)
    free = windows >= threshold
    free_fraction = np.where(windows_valid, free, False).mean(axis=1)
    # Distance to the nearest blocked bin on each side, normalised by n.
    left_free = np.zeros(n, dtype=float)
    right_free = np.zeros(n, dtype=float)
    blocked = (~valid) | (values < threshold)
    for i in range(n):
        j = i
        while j >= 0 and not blocked[j]:
            j -= 1
        left_free[i] = min(n, i - j) / n
        j = i
        while j < n and not blocked[j]:
            j += 1
        right_free[i] = min(n, j - i) / n
    return clearance, free_fraction, left_free, right_free


def beam_features(
    angles: np.ndarray,
    ranges: np.ndarray,
    valid: np.ndarray,
    max_range: float,
    gap_threshold: float,
    window: int = 5,
    previous_bearing: float = 0.0,
    front_sector: float = np.pi,
) -> np.ndarray:
    clearance, free_fraction, left_free, right_free = _window_stats(
        ranges, valid, window, max_range, gap_threshold
    )
    range_norm = np.clip(ranges / max_range, 0.0, 1.0)
    inv_range = 1.0 / (1.0 + np.maximum(ranges, 0.0))
    delta = angles - previous_bearing
    features = np.column_stack(
        [
            np.ones_like(ranges),
            range_norm,
            inv_range,
            np.sin(angles),
            np.cos(angles),
            np.clip(clearance / max_range, 0.0, 1.0),
            free_fraction,
            left_free,
            right_free,
            valid.astype(float),
            np.cos(delta),
            -np.abs(delta) / max(front_sector, 1e-6),
        ]
    )
    return features


# ---------------------------------------------------------------------------
# Runtime policies


class StructuredGapPolicy:
    """B1: parameterised furthest-gap controller."""

    def __init__(self, vector: np.ndarray | None = None) -> None:
        self.params = B1Params.from_vector(vector)
        self.reset()

    def reset(self) -> None:
        self.steering = 0.0
        self.last_angle = 0.0
        self.speed = 0.0
        self.last_debug: dict[str, float] = {}

    def score_surface(self, observation: Observation) -> dict[str, np.ndarray | float | int] | None:
        """Compute the full-resolution beam score surface.

        Exposed for training and debugging: the B2 warm start regresses its
        per-beam features onto this surface instead of imitating a single
        action, which preserves the safety ordering B1 learned.
        """

        p = self.params
        front_sector = np.radians(p.front_sector_deg)
        scan = _front_scan(observation, front_sector)
        if scan is None:
            return None
        angles, ranges, valid = scan
        front = valid & (np.abs(angles) <= front_sector)
        if not np.any(front):
            return None

        open_range = float(np.nanmax(ranges[front]))
        work = np.where(front, ranges, open_range)

        kernel_count = _coerce_odd(p.smoothing_beams, angles.size)
        if kernel_count > 1:
            kernel = np.ones(kernel_count) / kernel_count
            pad = kernel_count // 2
            padded = np.concatenate([work[-pad:], work, work[:pad]])
            work = np.convolve(padded, kernel, mode="same")[pad:-pad]

        step = abs(float(np.median(np.diff(angles)))) if angles.size > 1 else 0.01
        window = _coerce_odd(np.radians(p.clearance_window_deg) / max(step, 1e-6), angles.size)
        pad = window // 2
        padded_work = np.concatenate([work[-pad:], work, work[:pad]]) if pad else work
        windows = np.lib.stride_tricks.sliding_window_view(padded_work, window)
        clearance = windows.min(axis=1)

        threshold = max(p.min_gap_m, 0.75 * float(np.max(clearance[front])))
        free = front & (work >= threshold)
        padded_free = (
            np.concatenate([free[-pad:], free, free[:pad]]).astype(float) if pad else free.astype(float)
        )
        free_windows = np.lib.stride_tricks.sliding_window_view(padded_free, window)
        free_fraction = free_windows.mean(axis=1)

        scores = (
            p.clearance_weight * clearance
            + p.range_weight * work
            + p.gap_width_weight * 2.0 * free_fraction
            - p.angle_change_weight * np.abs(angles - self.last_angle)
            - p.center_bias_weight * np.abs(angles)
        )
        scores = np.where(front, scores, -np.inf)
        return {
            "angles": angles,
            "front": front,
            "work": work,
            "clearance": clearance,
            "free": free,
            "free_fraction": free_fraction,
            "scores": scores,
            "threshold": threshold,
            "window": window,
            "step": step,
            "front_sector": front_sector,
        }

    def act(self, observation: Observation) -> Actuation:
        p = self.params
        surface = self.score_surface(observation)
        if surface is None:
            self.last_debug = {"valid": 0.0}
            return Actuation(throttle=0.0, steering=self.steering)
        angles = np.asarray(surface["angles"], dtype=float)
        clearance = np.asarray(surface["clearance"], dtype=float)
        free = np.asarray(surface["free"], dtype=bool)
        scores = np.asarray(surface["scores"], dtype=float)
        window = int(surface["window"])
        step = float(surface["step"])
        dt = max(observation.dt, 1e-4)

        best = int(np.argmax(scores))
        if not np.isfinite(scores[best]):
            return Actuation(throttle=0.0, steering=0.0)

        best_bearing = float(angles[best])
        self.last_angle = best_bearing
        best_clearance = float(clearance[best])

        if free[best]:
            lo = best
            hi = best
            while lo > 0 and free[lo - 1]:
                lo -= 1
            while hi < free.size - 1 and free[hi + 1]:
                hi += 1
            gap_left = float(min(angles[lo], angles[hi]))
            gap_right = float(max(angles[lo], angles[hi]))
            gap_width = float(abs(angles[hi] - angles[lo]))
            gap_center = 0.5 * (gap_left + gap_right)
        else:
            gap_width = float(window * step)
            gap_left = best_bearing - gap_width / 2.0
            gap_right = best_bearing + gap_width / 2.0
            gap_center = best_bearing

        desired = p.aim_blend * gap_center + (1.0 - p.aim_blend) * best_bearing
        target = _clip(p.steer_gain * desired, -p.max_steering_rad, p.max_steering_rad)
        alpha = min(1.0, p.steer_smoothing * dt)
        self.steering = _clip(
            self.steering + (target - self.steering) * alpha,
            -p.max_steering_rad,
            p.max_steering_rad,
        )

        speed = p.base_speed_mps
        if best_clearance < p.clearance_slow_1_m:
            speed = min(speed, p.clearance_slow_1_speed)
        elif best_clearance < p.clearance_slow_2_m:
            speed = min(speed, p.clearance_slow_2_speed)
        speed = min(p.base_speed_mps, speed + p.gap_width_speed_gain * gap_width)
        measured_speed = max(float(observation.state.speed_mps), 0.2)
        curvature = abs(float(observation.state.yaw_rate_rad_s)) / measured_speed
        speed = min(speed, p.base_speed_mps / (1.0 + p.curvature_speed_gain * curvature))
        speed = _clip(speed, p.speed_min_mps, p.base_speed_mps)
        alpha = min(1.0, p.speed_smoothing * dt)
        self.speed += (speed - self.speed) * alpha
        throttle = _clip(self.speed / max(p.throttle_max_speed, 1e-6), 0.0, 1.0)

        self.last_debug = {
            "valid": 1.0,
            "best_bearing_rad": best_bearing,
            "target_bearing_rad": desired,
            "steering_target_rad": target,
            "clearance_m": best_clearance,
            "gap_width_rad": gap_width,
            "target_speed_mps": self.speed,
            "throttle": throttle,
            "steering_rad": self.steering,
            "score": float(scores[best]),
        }
        return Actuation(throttle=throttle, steering=self.steering)


class BeamScorerPolicy:
    """B2: learned per-beam scorer with a small target-speed head."""

    def __init__(self, vector: np.ndarray, hidden: int = 0, n_beams: int = N_BEAMS) -> None:
        self.params = B2Params.from_vector(vector, hidden)
        self.hidden = hidden
        self.n_beams = n_beams
        self.reset()

    def reset(self) -> None:
        self.steering = 0.0
        self.speed = 0.0
        self.last_throttle = 0.0
        self.last_selected_bearing = 0.0
        self.last_debug: dict[str, object] = {}

    def _features(self, observation: Observation) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        scalars = _unit_to_params(self.params.scalars, B2_SCALARS)
        front_sector = np.radians(float(scalars[0]))
        gap_threshold = float(scalars[1])
        max_range = float(scalars[2])
        scan = _front_scan(observation, front_sector)
        if scan is None:
            return None
        angles, ranges, valid = scan
        centers, resampled, resampled_valid = _resample_front(
            angles, ranges, valid, front_sector, self.n_beams, max_range
        )
        beam_step = 2.0 * front_sector / max(self.n_beams, 1)
        window = _coerce_odd(
            np.radians(float(scalars[9])) / max(beam_step, 1e-6),
            self.n_beams,
        )
        features = beam_features(
            centers,
            resampled,
            resampled_valid,
            max_range,
            gap_threshold,
            window=window,
            previous_bearing=self.last_selected_bearing,
            front_sector=front_sector,
        )
        state = observation.state
        max_steer = max(float(observation.limits.max_steering_rad), 1e-6)
        global_features = np.array(
            [
                float(state.speed_mps) / 4.0,
                float(state.yaw_rate_rad_s),
                self.steering / max_steer,
                self.last_throttle,
                self.last_selected_bearing,
                np.sin(self.last_selected_bearing),
                np.cos(self.last_selected_bearing),
            ],
            dtype=float,
        )
        return centers, features, global_features

    def act(self, observation: Observation) -> Actuation:
        data = self._features(observation)
        if data is None:
            self.last_debug = {"valid": 0.0}
            return Actuation(throttle=0.0, steering=self.steering)
        centers, features, global_features = data
        scalars = _unit_to_params(self.params.scalars, B2_SCALARS)
        front_sector = np.radians(float(scalars[0]))
        max_range = float(scalars[2])
        steer_gain = float(scalars[3])
        steer_bias = float(scalars[4])
        steer_smoothing = float(scalars[5])
        speed_smoothing = float(scalars[6])
        speed_min = float(scalars[7])
        speed_max = min(float(scalars[8]), float(observation.limits.max_speed_mps))
        dt = max(observation.dt, 1e-4)

        p = self.params
        beam_scores = features @ p.beam_linear_w + p.beam_linear_b
        if self.hidden:
            beam_hidden = np.tanh(features @ p.beam_res_w1 + p.beam_res_b1)
            beam_scores = beam_scores + beam_hidden @ p.beam_res_w2 + p.beam_res_b2
        best = int(np.argmax(beam_scores))
        selected = features[best]
        selected_bearing = float(centers[best])
        self.last_selected_bearing = selected_bearing

        speed_input = np.concatenate([selected, global_features])
        speed_logit = float(speed_input @ p.speed_linear_w + p.speed_linear_b)
        if self.hidden:
            speed_hidden = np.tanh(speed_input @ p.speed_res_w1 + p.speed_res_b1)
            speed_logit += float(speed_hidden @ p.speed_res_w2 + p.speed_res_b2)
        target_speed = speed_min + (speed_max - speed_min) * _sigmoid(speed_logit)
        alpha = min(1.0, speed_smoothing * dt)
        self.speed += (target_speed - self.speed) * alpha

        max_steer = float(observation.limits.max_steering_rad)
        desired = _clip(steer_gain * selected_bearing + steer_bias, -max_steer, max_steer)
        alpha = min(1.0, steer_smoothing * dt)
        self.steering = _clip(
            self.steering + (desired - self.steering) * alpha,
            -max_steer,
            max_steer,
        )
        throttle = _clip(self.speed / max(float(observation.limits.max_speed_mps), 1e-6), 0.0, 1.0)
        self.last_throttle = throttle

        self.last_debug = {
            "valid": 1.0,
            "selected_index": best,
            "selected_bearing_rad": selected_bearing,
            "selected_range_m": float(selected[1]) * max_range,
            "target_speed_mps": self.speed,
            "throttle": throttle,
            "steering_rad": self.steering,
            "beam_score": float(beam_scores[best]),
            "front_sector_rad": float(front_sector),
            "max_range_m": max_range,
            "angles_rad": centers,
            "scores": beam_scores,
        }
        return Actuation(throttle=throttle, steering=self.steering)


# ---------------------------------------------------------------------------
# Weight loading and the dashboard-selected controller


def _load_policy_file(path: Path) -> tuple[str, np.ndarray, int, dict[str, object]]:
    with np.load(path, allow_pickle=False) as data:
        policy_type = str(np.asarray(data["policy_type"]).item())
        vector = np.asarray(data["vector"], dtype=float)
        hidden = int(np.asarray(data["hidden"]).item()) if "hidden" in data else 0
        meta_raw = str(np.asarray(data["meta_json"]).item()) if "meta_json" in data else "{}"
    try:
        meta = json.loads(meta_raw)
    except json.JSONDecodeError:
        meta = {}
    return policy_type, vector, hidden, meta


class LearnedGapController:
    """Dashboard/CLI controller that loads the current best evolved policy."""

    def __init__(self, weights_path: Path | str | None = None) -> None:
        self.weights_path = Path(weights_path) if weights_path is not None else WEIGHTS_PATH
        self.policy: StructuredGapPolicy | BeamScorerPolicy
        self.policy_type = "b1"
        self.meta: dict[str, object] = {}
        self._load()

    def _load(self) -> None:
        if not self.weights_path.exists():
            self.policy = StructuredGapPolicy()
            self.policy_type = "b1"
            self.meta = {"source": "default", "path": str(self.weights_path)}
            return
        try:
            policy_type, vector, hidden, meta = _load_policy_file(self.weights_path)
            if policy_type == "b2":
                self.policy = BeamScorerPolicy(vector, hidden=hidden)
            else:
                self.policy = StructuredGapPolicy(vector)
            self.policy_type = policy_type
            self.meta = meta
        except Exception as exc:  # noqa: BLE001 - fall back rather than break the dashboard
            self.policy = StructuredGapPolicy()
            self.policy_type = "b1"
            self.meta = {"source": "fallback", "error": str(exc)}

    def reset(self) -> None:
        self.policy.reset()

    def update(self, observation: Observation, speed_limit: float | None = None) -> Actuation:
        # The evolved policy owns its speed plan; the generic dashboard speed
        # slider is intentionally not used as a hard cap. ``speed_limit`` is
        # accepted for interface compatibility with the controller registry.
        return self.policy.act(observation)


__all__ = [
    "B1_DEFAULT",
    "B1_HIGH",
    "B1_LOW",
    "B1_SPECS",
    "B1Params",
    "B2_SCALARS",
    "B2Params",
    "BeamScorerPolicy",
    "LearnedGapController",
    "N_BEAMS",
    "StructuredGapPolicy",
    "WEIGHTS_PATH",
    "b2_default_vector",
    "b2_expand_to_hidden",
    "b2_from_b1",
    "b2_layout",
    "beam_features",
]
