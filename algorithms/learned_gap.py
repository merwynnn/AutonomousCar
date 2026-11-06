"""CMA-ES-evolved lidar gap controllers.

This module is the portable runtime side of the training pipeline in
``training/``. It imports only NumPy and ``onboard.api`` so the same code can
run in the simulator, on the dashboard and on the Raspberry Pi.

Two policy families are supported, selected by ``policy_type`` in the weights
file:

``b1``
    A parameterised version of the furthest-gap controller. CMA-ES tunes the
    scan processing, scoring and speed schedule directly. The implementation
    lives in ``algorithms/gap_policy.py`` (``StructuredGapPolicy``) and is
    re-exported here for import compatibility.

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
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
from onboard.api import Actuation, Observation

from algorithms.gap_policy import (
    B1_DEFAULT,
    B1_HIGH,
    B1_LOW,
    B1_SPECS,
    B1Params,
    ParamSpec,
    StructuredGapPolicy,
    _clip,
    _coerce_odd,
    _front_scan,
    _params_to_unit,
    _sigmoid,
    _unit_to_params,
)

WEIGHTS_PATH = Path(__file__).with_name("weights") / "policy.npz"



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
    # Enveloppe de sécurité évoluable : bornés en dur auparavant, ces gains
    # plafonnaient la vitesse à ~0.15 + 2.2 x dégagement (~2.7 m/s sur la
    # piste étroite) quelle que soit la tête apprise. Ils font maintenant
    # partie du génome pour que CMA-ES puisse les relâcher.
    ParamSpec("min_clearance_m", 0.15, 0.60, 0.30),
    ParamSpec("clearance_speed_gain", 0.5, 6.0, 2.2),
    ParamSpec("steer_slow_gain", 0.0, 0.60, 0.25),
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
            0.30,
            2.2,
            0.25,
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
    mask = valid & (angles >= edges[0]) & (angles <= edges[-1])
    if np.any(mask):
        bins = np.clip(np.digitize(angles[mask], edges) - 1, 0, n_beams - 1)
        np.minimum.at(out_ranges, bins, np.minimum(ranges[mask], max_range))
        out_valid = np.bincount(bins, minlength=n_beams) > 0
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
    blocked = (~valid) | (values < threshold)
    indices = np.arange(n)
    # The original scan-edge semantics: with no blocked bin, the distance is
    # measured to the array edge (1/n or (n-i)/n), not to a large sentinel.
    left_blocked = np.maximum.accumulate(np.where(blocked, indices, -1))
    left_free = (indices - left_blocked) / n
    right_blocked = np.minimum.accumulate(
        np.where(blocked, indices, n)[::-1]
    )[::-1]
    right_free = (right_blocked - indices) / n
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


class BeamScorerPolicy:
    """B2: learned per-beam scorer with a small target-speed head.

    A sensor-only safety envelope wraps the learned head:

    - beams whose clearance is below ``min_clearance_m`` cannot be selected;
    - target speed is capped by the selected clearance and by the steering
      angle, so the car actually slows for corners instead of holding a
      constant throttle;
    - if the car is pinned against something while still commanding throttle,
      a short reverse manoeuvre backs it out.

    These rules use only ``Observation`` and are identical in simulation and on
    the Pi.
    """

    min_clearance_m = 0.30
    clearance_speed_gain = 2.2
    clearance_speed_offset = 0.15
    steer_slow_gain = 0.25
    stuck_speed_mps = 0.06
    stuck_time_s = 0.8
    recovery_time_s = 0.8
    recovery_throttle = -0.30
    recovery_steer = 0.35

    def __init__(self, vector: np.ndarray, hidden: int = 0, n_beams: int = N_BEAMS) -> None:
        self.params = B2Params.from_vector(vector, hidden)
        self.hidden = hidden
        self.n_beams = n_beams
        # The scalar layout and the scan window are fixed for a loaded policy.
        # Precompute them once: this is the hot path on a small onboard CPU.
        self.scalars = _unit_to_params(self.params.scalars, B2_SCALARS)
        # Envelope gains are evolved: override the class defaults with the
        # genome values so CMA-ES can relax the hard-coded speed ceiling.
        self.min_clearance_m = float(self.scalars[10])
        self.clearance_speed_gain = float(self.scalars[11])
        self.steer_slow_gain = float(self.scalars[12])
        self.front_sector = float(np.radians(self.scalars[0]))
        self.gap_threshold = float(self.scalars[1])
        self.max_range = float(self.scalars[2])
        beam_step = 2.0 * self.front_sector / max(self.n_beams, 1)
        self.window = _coerce_odd(
            np.radians(float(self.scalars[9])) / max(beam_step, 1e-6),
            self.n_beams,
        )
        self.reset()

    def reset(self) -> None:
        self.steering = 0.0
        self.speed = 0.0
        self.last_throttle = 0.0
        self.last_selected_bearing = 0.0
        self.stuck_time = 0.0
        self.recovery_remaining = 0.0
        self.recovery_steering = 0.0
        self.last_debug: dict[str, object] = {}

    def _features(self, observation: Observation) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        scan = _front_scan(observation, self.front_sector)
        if scan is None:
            return None
        angles, ranges, valid = scan
        centers, resampled, resampled_valid = _resample_front(
            angles,
            ranges,
            valid,
            self.front_sector,
            self.n_beams,
            self.max_range,
        )
        features = beam_features(
            centers,
            resampled,
            resampled_valid,
            self.max_range,
            self.gap_threshold,
            window=self.window,
            previous_bearing=self.last_selected_bearing,
            front_sector=self.front_sector,
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
        scalars = self.scalars
        front_sector = self.front_sector
        max_range = self.max_range
        steer_gain = float(scalars[3])
        steer_bias = float(scalars[4])
        steer_smoothing = float(scalars[5])
        speed_smoothing = float(scalars[6])
        speed_min = float(scalars[7])
        speed_max = min(float(scalars[8]), float(observation.limits.max_speed_mps))
        dt = max(observation.dt, 1e-4)
        max_steer = max(float(observation.limits.max_steering_rad), 1e-6)

        # Reverse out if the car is pinned and still asking for throttle.
        if self.recovery_remaining > 0.0:
            self.recovery_remaining = max(0.0, self.recovery_remaining - dt)
            self.steering = _clip(self.recovery_steering, -max_steer, max_steer)
            self.speed = 0.0
            self.last_throttle = self.recovery_throttle
            self.last_debug = {
                "valid": 1.0,
                "recovering": 1.0,
                "steering_rad": self.steering,
                "throttle": self.recovery_throttle,
            }
            return Actuation(throttle=self.recovery_throttle, steering=self.steering)

        p = self.params
        beam_scores = features @ p.beam_linear_w + p.beam_linear_b
        if self.hidden:
            beam_hidden = np.tanh(features @ p.beam_res_w1 + p.beam_res_b1)
            beam_scores = beam_scores + beam_hidden @ p.beam_res_w2 + p.beam_res_b2
        clearance_m = features[:, 5] * max_range
        safe = clearance_m >= self.min_clearance_m
        if np.any(safe):
            scored = np.where(safe, beam_scores, -np.inf)
            best = int(np.argmax(scored))
            best_score = float(scored[best])
            forced_slow = False
        else:
            scored = clearance_m
            best = int(np.argmax(scored))
            best_score = float(beam_scores[best])
            forced_slow = True
        selected = features[best]
        selected_bearing = float(centers[best])
        selected_clearance = float(clearance_m[best])
        self.last_selected_bearing = selected_bearing

        speed_input = np.concatenate([selected, global_features])
        speed_logit = float(speed_input @ p.speed_linear_w + p.speed_linear_b)
        if self.hidden:
            speed_hidden = np.tanh(speed_input @ p.speed_res_w1 + p.speed_res_b1)
            speed_logit += float(speed_hidden @ p.speed_res_w2 + p.speed_res_b2)
        head_speed = speed_min + (speed_max - speed_min) * _sigmoid(speed_logit)

        # Safety envelope: slow for tight clearance and for large steering.
        path_clearance = selected_clearance
        clearance_cap = _clip(
            self.clearance_speed_offset + self.clearance_speed_gain * path_clearance,
            speed_min,
            speed_max,
        )
        steer_fraction = min(1.0, abs(selected_bearing) / max_steer)
        steer_cap = speed_max * (1.0 - self.steer_slow_gain * steer_fraction)
        target_speed = min(
            head_speed,
            clearance_cap,
            steer_cap,
            float(observation.limits.max_speed_mps),
        )
        if forced_slow:
            target_speed = min(target_speed, 0.60)
            if selected_clearance < 0.20:
                target_speed = 0.0
        target_speed = max(speed_min, target_speed)
        alpha = min(1.0, speed_smoothing * dt)
        self.speed += (target_speed - self.speed) * alpha

        desired = _clip(
            steer_gain * selected_bearing + steer_bias, -max_steer, max_steer
        )
        alpha = min(1.0, steer_smoothing * dt)
        self.steering = _clip(
            self.steering + (desired - self.steering) * alpha,
            -max_steer,
            max_steer,
        )
        throttle = _clip(
            self.speed / max(float(observation.limits.max_speed_mps), 1e-6),
            0.0,
            1.0,
        )
        self.last_throttle = throttle

        # Stuck detection happens after the command is computed so the next
        # tick can start the reverse manoeuvre.
        measured_speed = abs(float(observation.state.speed_mps))
        if measured_speed < self.stuck_speed_mps and self.speed > 0.30:
            self.stuck_time += dt
        else:
            self.stuck_time = 0.0
        if self.stuck_time >= self.stuck_time_s:
            self.stuck_time = 0.0
            self.recovery_remaining = self.recovery_time_s
            self.recovery_steering = (
                -self.recovery_steer * float(np.sign(self.steering))
                if abs(self.steering) > 0.05
                else 0.0
            )
            self.speed = 0.0
            self.last_throttle = self.recovery_throttle
            self.last_debug = {
                "valid": 1.0,
                "recovering": 1.0,
                "steering_rad": self.recovery_steering,
                "throttle": self.recovery_throttle,
            }
            return Actuation(
                throttle=self.recovery_throttle,
                steering=self.recovery_steering,
            )

        self.last_debug = {
            "valid": 1.0,
            "selected_index": best,
            "selected_bearing_rad": selected_bearing,
            "selected_range_m": float(selected[1]) * max_range,
            "selected_clearance_m": selected_clearance,
            "path_clearance_m": path_clearance,
            "head_speed_mps": head_speed,
            "clearance_speed_cap_mps": clearance_cap,
            "steer_speed_cap_mps": steer_cap,
            "forced_slow": float(forced_slow),
            "target_speed_mps": self.speed,
            "throttle": throttle,
            "steering_rad": self.steering,
            "beam_score": best_score,
            "front_sector_rad": float(front_sector),
            "max_range_m": max_range,
            "angles_rad": centers,
            "scores": scored,
            "recovering": 0.0,
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


def _load_fallback_vector(path: Path) -> np.ndarray | None:
    with np.load(path, allow_pickle=False) as data:
        if "fallback_vector" not in data:
            return None
        return np.asarray(data["fallback_vector"], dtype=float)


class LearnedGapController:
    """Dashboard/CLI controller that loads the current best evolved policy.

    The fast B2 policy is wrapped with a conservative B1 gap-follow fallback.
    The fallback engages when the learned policy's selected beam is tight or
    when every beam is inside the safety clearance; it disengages after a
    short clear stretch. This keeps the fast policy on open track while giving
    hard tracks a known-safe low-speed behaviour.
    """

    fallback_speed_cap = 1.6
    fallback_enter_clearance_m = 0.55
    fallback_exit_clearance_m = 0.75
    fallback_exit_ticks = 12

    def __init__(self, weights_path: Path | str | None = None) -> None:
        self.weights_path = Path(weights_path) if weights_path is not None else WEIGHTS_PATH
        self.policy: StructuredGapPolicy | BeamScorerPolicy
        self.fallback: StructuredGapPolicy | None = None
        self.policy_type = "b1"
        self.meta: dict[str, object] = {}
        self.fallback_active = False
        self.fallback_clear_ticks = 0
        self.last_mode = "learned"
        self.last_clearance_m = 0.0
        self._load()

    def _load(self) -> None:
        self.fallback = None
        if not self.weights_path.exists():
            self.policy = StructuredGapPolicy()
            self.policy_type = "b1"
            self.meta = {"source": "default", "path": str(self.weights_path)}
            return
        try:
            policy_type, vector, hidden, meta = _load_policy_file(self.weights_path)
            if policy_type == "b2":
                self.policy = BeamScorerPolicy(vector, hidden=hidden)
                fallback_vector = _load_fallback_vector(self.weights_path)
                if fallback_vector is None:
                    sibling = self.weights_path.with_name("policy_b1.npz")
                    if sibling.exists():
                        _, fallback_vector, _, _ = _load_policy_file(sibling)
                if (
                    fallback_vector is not None
                    and fallback_vector.size <= len(B1_DEFAULT)
                ):
                    fallback = StructuredGapPolicy(fallback_vector)
                    fallback.params = replace(
                        fallback.params,
                        base_speed_mps=min(
                            fallback.params.base_speed_mps,
                            self.fallback_speed_cap,
                        ),
                    )
                    self.fallback = fallback
            else:
                self.policy = StructuredGapPolicy(vector)
            self.policy_type = policy_type
            self.meta = meta
        except Exception as exc:  # noqa: BLE001 - fall back rather than break the dashboard
            self.policy = StructuredGapPolicy()
            self.policy_type = "b1"
            self.fallback = None
            self.meta = {"source": "fallback", "error": str(exc)}

    def load_weights(self, weights_path: Path | str) -> None:
        """Hot-swap the policy weights, e.g. a training run's best checkpoint."""
        self.weights_path = Path(weights_path)
        self._load()

    def reset(self) -> None:
        self.policy.reset()
        if self.fallback is not None:
            self.fallback.reset()
        self.fallback_active = False
        self.fallback_clear_ticks = 0
        self.last_mode = "learned"
        self.last_clearance_m = 0.0

    def update(self, observation: Observation, speed_limit: float | None = None) -> Actuation:
        # The evolved policy owns its speed plan; the generic dashboard speed
        # slider is intentionally not used as a hard cap. ``speed_limit`` is
        # accepted for interface compatibility with the controller registry.
        if not isinstance(self.policy, BeamScorerPolicy) or self.fallback is None:
            return self.policy.act(observation)

        command = self.policy.act(observation)
        debug = self.policy.last_debug
        clearance_value = debug.get("selected_clearance_m", 0.0)
        clearance = (
            float(clearance_value)
            if isinstance(clearance_value, (int, float))
            else 0.0
        )
        forced_value = debug.get("forced_slow", 0.0)
        forced_slow = (
            isinstance(forced_value, (int, float)) and float(forced_value) > 0.5
        )
        self.last_clearance_m = clearance

        if not self.fallback_active:
            if forced_slow or clearance < self.fallback_enter_clearance_m:
                self.fallback_active = True
                self.fallback_clear_ticks = 0
                self.fallback.reset()
        else:
            if not forced_slow and clearance > self.fallback_exit_clearance_m:
                self.fallback_clear_ticks += 1
                if self.fallback_clear_ticks >= self.fallback_exit_ticks:
                    self.fallback_active = False
            else:
                self.fallback_clear_ticks = 0

        if not self.fallback_active:
            self.last_mode = "learned"
            return command

        fallback_command = self.fallback.act(observation)
        # Keep the learned policy's internal state slow while the fallback is
        # driving, so it does not resume with a stale high speed or steering.
        self.policy.speed = min(self.policy.speed, self.fallback_speed_cap)
        self.policy.steering = self.fallback.steering
        self.last_mode = "fallback"
        return fallback_command


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
