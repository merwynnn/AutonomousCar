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
    loop_closures: int = 0
    loop_closure_this_scan: bool = False
    graph_nodes: int = 0
    graph_edges: int = 0
    loop_candidate: int = -1
    loop_match_x: float = 0.0
    loop_match_y: float = 0.0
    loop_match_theta: float = 0.0
    mcl_x: float = 0.0
    mcl_y: float = 0.0
    mcl_theta: float = 0.0
    mcl_initialized: bool = False
    mcl_ess: float = 0.0
    mcl_spread: float = 0.0
    mcl_particle_count: int = 0
    global_relocalizations: int = 0
    relocalized_this_scan: bool = False


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
        self._last_odom: tuple[float, float, float] | None = None
        self._last_slam: tuple[float, float, float] | None = None
        self._mcl_enabled = False
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
        self._last_odom = None
        self._last_slam = None

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
        if self._last_odom is None or self._last_slam is None:
            prediction = (odometry_x, odometry_y, odometry_theta)
        else:
            # Use the odometry *increment* relative to the previous scan, applied
            # to the previous SLAM pose. This cancels accumulated global odometry
            # drift while keeping the local motion prediction accurate.
            last_ox, last_oy, last_otheta = self._last_odom
            dx = odometry_x - last_ox
            dy = odometry_y - last_oy
            c = np.cos(last_otheta)
            s = np.sin(last_otheta)
            body_dx = c * dx + s * dy
            body_dy = -s * dx + c * dy
            dtheta = np.arctan2(
                np.sin(odometry_theta - last_otheta),
                np.cos(odometry_theta - last_otheta),
            )
            sx, sy, stheta = self._last_slam
            prediction = (
                sx + np.cos(stheta) * body_dx - np.sin(stheta) * body_dy,
                sy + np.sin(stheta) * body_dx + np.cos(stheta) * body_dy,
                np.arctan2(np.sin(stheta + dtheta), np.cos(stheta + dtheta)),
            )
        result = self.impl.process(
            core_scan,
            core.Pose2(float(prediction[0]), float(prediction[1]), float(prediction[2])),
        )
        self._last_odom = (odometry_x, odometry_y, odometry_theta)
        self._last_slam = (float(result.pose.x), float(result.pose.y), float(result.pose.theta))
        return SlamUpdate(
            x=float(result.pose.x),
            y=float(result.pose.y),
            theta=float(result.pose.theta),
            match_score=float(result.match_score),
            inliers=int(result.inliers),
            scan_count=int(result.scan_count),
            loop_closures=int(result.loop_closures),
            loop_closure_this_scan=bool(result.loop_closure_this_scan),
            graph_nodes=int(result.graph_nodes),
            graph_edges=int(result.graph_edges),
            loop_candidate=int(result.loop_candidate),
            loop_match_x=float(result.loop_match_x),
            loop_match_y=float(result.loop_match_y),
            loop_match_theta=float(result.loop_match_theta),
            mcl_x=float(result.mcl_pose.x),
            mcl_y=float(result.mcl_pose.y),
            mcl_theta=float(result.mcl_pose.theta),
            mcl_initialized=bool(result.mcl_initialized),
            mcl_ess=float(result.mcl_ess),
            mcl_spread=float(result.mcl_spread),
            mcl_particle_count=int(result.mcl_particle_count),
            global_relocalizations=int(result.global_relocalizations),
            relocalized_this_scan=bool(result.relocalized_this_scan),
        )

    def set_mcl_enabled(self, enabled: bool) -> None:
        self._mcl_enabled = enabled
        self.impl.set_mcl_enabled(enabled)

    def set_loop_closure_enabled(self, enabled: bool) -> None:
        self.impl.set_loop_closure_enabled(enabled)

    def set_global_relocalization_enabled(self, enabled: bool) -> None:
        self.impl.set_global_relocalization_enabled(enabled)

    def request_global_relocalization(self) -> None:
        self.impl.request_global_relocalization()

    def set_mcl_params(self, **kwargs: float | int) -> None:
        params = core.MclParams()
        for key, value in kwargs.items():
            if not hasattr(params, key):
                raise TypeError(f"unknown MCL parameter: {key}")
            setattr(params, key, value)
        self.impl.set_mcl_params(params)

    def set_map_frozen(self, frozen: bool) -> None:
        self.impl.set_map_frozen(frozen)

    def reset_tracking(self) -> None:
        """Reset odometry prediction and MCL without deleting the map."""
        self._last_odom = None
        self._last_slam = None
        self.impl.set_mcl_enabled(False)
        if self._mcl_enabled:
            self.impl.set_mcl_enabled(True)

    def mcl_particles(self) -> list[tuple[float, float, float]]:
        particles = self.impl.mcl_particles()
        return [(float(p.x), float(p.y), float(p.theta)) for p in particles]

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
