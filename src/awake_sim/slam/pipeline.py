"""Python wrapper around the C++ SLAM core with scan deskewing and map export."""

from __future__ import annotations

import base64
from dataclasses import dataclass

import numpy as np

from awake_sim.sensors.lidar import LidarScan
from awake_sim.slam import HAVE_CPP, core


@dataclass
class SlamUpdate:
    x: float
    y: float
    theta: float
    match_score: float
    inliers: int
    scan_count: int


def deskew_scan(
    scan: LidarScan,
    speed_mps: float,
    yaw_rate_rad_s: float,
    period_s: float = 0.1,
) -> tuple[np.ndarray, np.ndarray]:
    """Remove spinning-lidar motion distortion using the odometry/IMU twist.

    Points are measured at different times during the scan. Applying the
    incremental body motion to each point moves them into the scan-start frame.
    """

    angles = np.asarray(scan.angles, dtype=float)
    ranges = np.asarray(scan.ranges, dtype=float)
    valid = np.isfinite(ranges) & (ranges > 0.0)
    count = angles.size
    dt = (np.arange(count, dtype=float) / max(1, count)) * period_s
    dtheta = yaw_rate_rad_s * dt
    dx = speed_mps * dt
    local_x = np.cos(angles) * ranges
    local_y = np.sin(angles) * ranges
    start_x = np.cos(dtheta) * local_x - np.sin(dtheta) * local_y + dx
    start_y = np.sin(dtheta) * local_x + np.cos(dtheta) * local_y
    new_ranges = np.hypot(start_x, start_y)
    new_angles = np.arctan2(start_y, start_x)
    new_ranges[~valid] = np.nan
    return new_angles, new_ranges


class SlamPipeline:
    def __init__(
        self,
        resolution_m: float = 0.05,
        width_m: float = 15.0,
        height_m: float = 8.0,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
        lidar_offset_x: float = 0.10,
        lidar_offset_y: float = 0.0,
        max_range_m: float = 12.0,
        scan_period_s: float = 0.1,
    ) -> None:
        if not HAVE_CPP or core is None:
            raise RuntimeError("C++ SLAM extension is not built")
        self.width_cells = int(round(width_m / resolution_m))
        self.height_cells = int(round(height_m / resolution_m))
        self.resolution = resolution_m
        self.scan_period = scan_period_s
        self.impl = core.SlamCore(
            resolution_m,
            self.width_cells,
            self.height_cells,
            origin_x,
            origin_y,
            lidar_offset_x,
            lidar_offset_y,
            max_range_m,
        )

    def reset(self) -> None:
        self.impl.reset()

    def process(
        self,
        scan: LidarScan,
        odometry_x: float,
        odometry_y: float,
        odometry_theta: float,
        speed_mps: float,
        yaw_rate_rad_s: float,
    ) -> SlamUpdate:
        angles, ranges = deskew_scan(
            scan,
            speed_mps,
            yaw_rate_rad_s,
            period_s=self.scan_period,
        )
        core_ranges = [0.0 if not np.isfinite(r) else float(r) for r in ranges]
        core_scan = core.Scan(
            core_ranges,
            [float(a) for a in angles],
            float(scan.t_measure),
        )
        result = self.impl.process(
            core_scan,
            core.Pose2(float(odometry_x), float(odometry_y), float(odometry_theta)),
        )
        return SlamUpdate(
            x=float(result.pose.x),
            y=float(result.pose.y),
            theta=float(result.pose.theta),
            match_score=float(result.match_score),
            inliers=int(result.inliers),
            scan_count=int(result.scan_count),
        )

    def map_array(self) -> np.ndarray:
        values = np.asarray(self.impl.map_log_odds(), dtype=np.float32)
        return values.reshape((self.height_cells, self.width_cells))

    def map_payload(self) -> dict:
        """Compact occupancy map for the dashboard: -1 unknown, 0 free, 100 occupied."""

        log_odds = self.map_array()
        occupancy = np.full(log_odds.shape, -1, dtype=np.int8)
        occupancy[log_odds < -0.2] = 0
        occupancy[log_odds > 0.5] = 100
        encoded = base64.b64encode(occupancy.tobytes()).decode("ascii")
        return {
            "width": self.width_cells,
            "height": self.height_cells,
            "resolution": self.resolution,
            "origin_x": float(self.impl.map_origin_x),
            "origin_y": float(self.impl.map_origin_y),
            "data": encoded,
        }
