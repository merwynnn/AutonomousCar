"""Small SE(2)/SO(3) helpers shared by the sensor models."""

from __future__ import annotations

import math

import numpy as np


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF-style rotation: Rz(yaw) @ Ry(pitch) @ Rx(roll)."""

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def yaw_matrix(yaw: float) -> np.ndarray:
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    return np.array(
        [
            [cos_y, -sin_y, 0.0],
            [sin_y, cos_y, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
