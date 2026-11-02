"""Motion compensation for 2D lidar scans using the shared state estimate.

A spinning lidar scan is measured over one sweep. The scan is delivered as a
set of angles and ranges in the lidar frame at the start of that sweep. By the
time the controller sees it, the car has moved and the scan no longer describes
the geometry around the *current* car pose.

This module keeps a short history of the estimated vehicle pose and re-expresses
the latest scan in the current lidar frame:

``p_now = T_lidar(now)^-1 * T_lidar(scan) * p_scan``

Only the estimated pose from ``StateEstimate`` is used, never simulator ground
truth. The transform is a rigid 2D transform, so it corrects the bulk motion
between scans. Per-beam motion distortion inside the sweep is not corrected
until the sensor drivers provide per-beam timestamps.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from onboard.api import LidarObservation, Pose2, StateEstimate

_TWO_PI = 2.0 * math.pi


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % _TWO_PI - math.pi


def _interpolate_pose(
    before: tuple[float, Pose2],
    after: tuple[float, Pose2],
    t: float,
) -> Pose2:
    span = after[0] - before[0]
    if span <= 1e-9:
        return after[1]
    alpha = min(1.0, max(0.0, (t - before[0]) / span))
    return Pose2(
        x=before[1].x + (after[1].x - before[1].x) * alpha,
        y=before[1].y + (after[1].y - before[1].y) * alpha,
        theta=_wrap_angle(
            before[1].theta + _wrap_angle(after[1].theta - before[1].theta) * alpha
        ),
    )


class ScanMotionCompensator:
    """Re-express a scan in the current lidar frame using estimated odometry.

    The mount is the lidar pose in the vehicle frame. ``mount_yaw_rad`` is the
    sensor yaw relative to the vehicle x-axis. Roll and pitch are ignored by
    this planar compensator; the current car and sensor definitions keep them
    at zero for the lidar.
    """

    def __init__(
        self,
        mount_position_m: Sequence[float],
        mount_yaw_rad: float,
        *,
        history_s: float = 1.0,
        max_scan_age_s: float = 0.5,
    ) -> None:
        if history_s <= 0.0:
            raise ValueError("history_s must be positive")
        if max_scan_age_s <= 0.0:
            raise ValueError("max_scan_age_s must be positive")
        if len(mount_position_m) < 2:
            raise ValueError("mount_position_m must contain x and y")
        self.mount_x_m = float(mount_position_m[0])
        self.mount_y_m = float(mount_position_m[1])
        self.mount_yaw_rad = float(mount_yaw_rad)
        self.history_s = float(history_s)
        self.max_scan_age_s = float(max_scan_age_s)
        self._history: list[tuple[float, Pose2]] = []

    @property
    def has_history(self) -> bool:
        return bool(self._history)

    def reset(self) -> None:
        self._history.clear()

    def update(self, state: StateEstimate) -> None:
        """Append one estimated pose sample, keeping a bounded history."""

        t = float(state.t)
        pose = Pose2(
            x=float(state.pose.x),
            y=float(state.pose.y),
            theta=_wrap_angle(float(state.pose.theta)),
        )
        if self._history and t < self._history[-1][0] - 1e-9:
            # A reset or a clock jump: do not mix two different timelines.
            self._history.clear()
        if self._history and abs(t - self._history[-1][0]) <= 1e-9:
            self._history[-1] = (t, pose)
        else:
            self._history.append((t, pose))
        cutoff = t - self.history_s
        while len(self._history) > 2 and self._history[0][0] < cutoff:
            self._history.pop(0)
        if len(self._history) > 2048:
            del self._history[: len(self._history) - 2048]

    def pose_at(self, t: float) -> Pose2 | None:
        """Estimated vehicle pose at ``t``, interpolated from the history."""

        if not self._history:
            return None
        t = float(t)
        if t < self._history[0][0] - 1e-9:
            return None
        if t <= self._history[0][0]:
            return self._history[0][1]
        if t >= self._history[-1][0]:
            return self._history[-1][1]
        low = 0
        high = len(self._history) - 1
        while low + 1 < high:
            middle = (low + high) // 2
            if self._history[middle][0] <= t:
                low = middle
            else:
                high = middle
        return _interpolate_pose(self._history[low], self._history[high], t)

    def _lidar_pose(self, body: Pose2) -> Pose2:
        c = math.cos(body.theta)
        s = math.sin(body.theta)
        return Pose2(
            x=body.x + c * self.mount_x_m - s * self.mount_y_m,
            y=body.y + s * self.mount_x_m + c * self.mount_y_m,
            theta=_wrap_angle(body.theta + self.mount_yaw_rad),
        )

    def compensate(
        self,
        scan: LidarObservation | None,
        now: StateEstimate,
    ) -> LidarObservation | None:
        """Warp ``scan`` into the current lidar frame.

        Returns ``None`` when there is no usable scan, when the scan predates
        the pose history, or when it is older than ``max_scan_age_s``.
        """

        if scan is None:
            return None
        age = float(now.t) - float(scan.t_measure)
        if age < -1e-6 or age > self.max_scan_age_s:
            return None
        scan_body = self.pose_at(scan.t_measure)
        now_body = self.pose_at(now.t)
        if scan_body is None or now_body is None:
            return None
        scan_lidar = self._lidar_pose(scan_body)
        now_lidar = self._lidar_pose(now_body)

        # p_now = R(theta_scan - theta_now) p_scan + R(theta_now)^T (t_scan - t_now)
        delta_world_x = scan_lidar.x - now_lidar.x
        delta_world_y = scan_lidar.y - now_lidar.y
        c_now = math.cos(now_lidar.theta)
        s_now = math.sin(now_lidar.theta)
        translation_x = c_now * delta_world_x + s_now * delta_world_y
        translation_y = -s_now * delta_world_x + c_now * delta_world_y
        delta_theta = _wrap_angle(scan_lidar.theta - now_lidar.theta)
        c_delta = math.cos(delta_theta)
        s_delta = math.sin(delta_theta)

        angles = np.asarray(scan.angles_rad, dtype=float)
        ranges = np.asarray(scan.ranges_m, dtype=float)
        intensities = np.asarray(scan.intensities, dtype=float)
        valid = np.asarray(scan.valid, dtype=bool)
        usable = valid & np.isfinite(ranges) & (ranges > 0.0) & np.isfinite(angles)

        # Keep the nominal angle for invalid beams. They still represent a
        # beam direction, and copying the raw angle avoids filling the scan
        # with bogus zero-degree entries that break angle-difference logic in
        # downstream consumers.
        out_angles = np.array(angles, copy=True)
        out_ranges = np.full_like(ranges, np.nan)
        out_intensities = np.zeros_like(intensities)
        if np.any(usable):
            local_x = np.cos(angles[usable]) * ranges[usable]
            local_y = np.sin(angles[usable]) * ranges[usable]
            current_x = c_delta * local_x - s_delta * local_y + translation_x
            current_y = s_delta * local_x + c_delta * local_y + translation_y
            out_angles[usable] = np.arctan2(current_y, current_x)
            out_ranges[usable] = np.hypot(current_x, current_y)
            out_intensities[usable] = intensities[usable]

        return LidarObservation(
            angles_rad=out_angles,
            ranges_m=out_ranges,
            intensities=out_intensities,
            valid=usable,
            t_measure=scan.t_measure,
            t_available=scan.t_available,
            frame_id=scan.frame_id,
            motion_compensated=True,
            t_compensated=float(now.t),
        )


__all__ = ["ScanMotionCompensator"]
