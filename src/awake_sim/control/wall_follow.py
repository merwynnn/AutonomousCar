"""Simple reactive wall-follow controller using only lidar ranges."""

from __future__ import annotations

import math

import numpy as np

from awake_sim.sensors.lidar import LidarScan
from awake_sim.sim.vehicle import Actuation


def _sector_min(scan: LidarScan, low_deg: float, high_deg: float) -> float:
    low = math.radians(low_deg)
    high = math.radians(high_deg)
    mask = (scan.angles >= low) & (scan.angles <= high) & scan.valid
    if not np.any(mask):
        return float("inf")
    values = scan.ranges[mask]
    if values.size == 0:
        return float("inf")
    return float(np.nanmin(values))


class WallFollowController:
    """Keep roughly equal left/right clearance and slow down for the front wall."""

    def __init__(
        self,
        base_speed: float = 2.4,
        center_gain: float = 1.6,
        front_gain: float = 0.7,
        max_steering: float = 0.48,
        max_speed_for_throttle: float = 5.5,
    ) -> None:
        self.base_speed = base_speed
        self.center_gain = center_gain
        self.front_gain = front_gain
        self.max_steering = max_steering
        self.max_speed_for_throttle = max_speed_for_throttle
        self.steering = 0.0
        self.last_scan: LidarScan | None = None

    def reset(self) -> None:
        self.steering = 0.0
        self.last_scan = None

    def update(self, scan: LidarScan, dt: float) -> Actuation:
        self.last_scan = scan
        left = _sector_min(scan, 60.0, 120.0)
        right = _sector_min(scan, -120.0, -60.0)
        front = _sector_min(scan, -22.0, 22.0)
        front_left = _sector_min(scan, 22.0, 60.0)
        front_right = _sector_min(scan, -60.0, -22.0)

        if not math.isfinite(left):
            left = 1.2
        if not math.isfinite(right):
            right = 1.2
        if not math.isfinite(front):
            front = 4.0
        if not math.isfinite(front_left):
            front_left = front
        if not math.isfinite(front_right):
            front_right = front

        center_error = left - right  # positive if left wall is farther
        desired = self.center_gain * center_error + self.front_gain * (front_left - front_right)
        desired = max(-self.max_steering, min(self.max_steering, desired))
        # First-order steering servo smoothing; actual servo rate/lag is in the physics.
        smoothing = min(1.0, 6.0 * dt)
        self.steering += (desired - self.steering) * smoothing
        self.steering = max(-self.max_steering, min(self.max_steering, self.steering))

        speed = self.base_speed
        speed *= max(0.45, 1.0 - 0.8 * abs(self.steering) / self.max_steering)
        if front < 1.3:
            speed = min(speed, max(0.35, (front - 0.30) / 1.0 * self.base_speed))
        if front < 0.45:
            speed = 0.0
        if abs(center_error) > 0.22:
            speed = min(speed, 1.2)
        throttle = max(0.0, min(1.0, speed / self.max_speed_for_throttle))
        return Actuation(throttle=throttle, steering=self.steering)
