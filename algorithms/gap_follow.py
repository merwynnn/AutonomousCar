"""Lidar-only furthest-gap controller for the dashboard demo.

This is a normal algorithm: it sees only ``Observation`` and returns
``Actuation``. It uses no simulator internals and no ground-truth pose.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from algorithms.interface import Actuation, Observation


@dataclass
class GapDebug:
    angle_rad: float = 0.0
    steering_rad: float = 0.0
    range_m: float = 0.0
    clearance_m: float = 0.0
    gap_width_rad: float = 0.0
    valid: bool = False


class FurthestGapController:
    def __init__(
        self,
        max_steering: float = 0.50,
        front_sector_deg: float = 100.0,
        clearance_window_deg: float = 60.0,
        min_gap_m: float = 0.60,
        max_speed_for_throttle: float = 5.5,
    ) -> None:
        self.max_steering = max_steering
        self.front_sector = np.radians(front_sector_deg)
        self.clearance_window_deg = clearance_window_deg
        self.min_gap_m = min_gap_m
        self.max_speed_for_throttle = max_speed_for_throttle
        self.steering = 0.0
        self.debug = GapDebug()

    def reset(self) -> None:
        self.steering = 0.0
        self.debug = GapDebug()

    def update(self, observation: Observation, speed_limit: float) -> Actuation:
        scan = observation.lidar
        if scan is None:
            return Actuation(throttle=0.0, steering=self.steering)
        dt = max(observation.dt, 1e-4)
        angles = np.asarray(scan.angles_rad, dtype=float)
        ranges = np.asarray(scan.ranges_m, dtype=float)
        valid = scan.valid & np.isfinite(ranges) & (ranges > 0.05)
        front = valid & (np.abs(angles) <= self.front_sector)
        if not np.any(front):
            self.debug = GapDebug(valid=False)
            return Actuation(throttle=0.0, steering=0.0)

        open_range = float(np.nanmax(ranges[front]))
        work = np.where(front, ranges, open_range)
        kernel = np.ones(7) / 7.0
        padded = np.concatenate([work[-3:], work, work[:3]])
        work = np.convolve(padded, kernel, mode="same")[3:-3]

        step = abs(float(np.median(np.diff(angles))))
        window = max(1, int(round(np.radians(self.clearance_window_deg) / step)))
        if window % 2 == 0:
            window += 1
        window = min(window, angles.size if angles.size % 2 == 1 else angles.size - 1)
        pad = window // 2
        padded_work = (
            np.concatenate([work[-pad:], work, work[:pad]]) if pad > 0 else work
        )
        windows = np.lib.stride_tricks.sliding_window_view(padded_work, window)
        clearance = windows.min(axis=1)

        scores = clearance - 0.15 * np.abs(angles)
        scores[~front] = -np.inf
        best = int(np.argmax(scores))
        desired = float(angles[best])
        best_clearance = float(clearance[best])
        best_range = float(work[best])

        free = front & (work >= max(self.min_gap_m, 0.75 * best_clearance))
        if free[best]:
            lo = best
            hi = best
            while lo > 0 and free[lo - 1]:
                lo -= 1
            while hi < free.size - 1 and free[hi + 1]:
                hi += 1
            gap_width = float(abs(angles[hi] - angles[lo]))
        else:
            gap_width = float(window * abs(step))

        desired = max(-self.max_steering, min(self.max_steering, desired))
        smoothing = min(1.0, 10.0 * dt)
        self.steering += (desired - self.steering) * smoothing
        self.steering = max(-self.max_steering, min(self.max_steering, self.steering))

        speed = max(0.45, speed_limit)
        if best_clearance < 0.50:
            speed = min(speed, 0.55)
        elif best_clearance < 0.80:
            speed = min(speed, 1.0)
        throttle = max(0.0, min(1.0, speed / self.max_speed_for_throttle))
        self.debug = GapDebug(
            angle_rad=desired,
            steering_rad=self.steering,
            range_m=best_range,
            clearance_m=best_clearance,
            gap_width_rad=gap_width,
            valid=True,
        )
        return Actuation(throttle=throttle, steering=self.steering)
