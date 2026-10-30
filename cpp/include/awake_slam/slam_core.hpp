#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Dense>

namespace awake_slam {

struct Pose2 {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    Eigen::Vector3d as_vector() const { return Eigen::Vector3d(x, y, theta); }
};

struct Scan {
    std::vector<float> ranges;  // metres; NaN/<=0 means invalid
    std::vector<float> angles;  // radians in the scan frame
    double t_start = 0.0;       // seconds

    std::size_t size() const { return ranges.size(); }
    bool valid() const { return !ranges.empty() && ranges.size() == angles.size(); }
};

struct SlamResult {
    Pose2 pose;
    bool initialized = false;
    int scan_count = 0;
    double match_score = 0.0;
    int inliers = 0;
};

/// Log-odds occupancy grid with a chamfer likelihood field for scan matching.
class OccupancyGrid {
public:
    OccupancyGrid(double resolution, int width, int height, double origin_x, double origin_y);

    void reset();
    void update(const Pose2& pose,
                const Scan& scan,
                double lidar_offset_x,
                double lidar_offset_y,
                double max_range);
    void rebuild_likelihood(double sigma_m = 0.15);

    double score_local(const std::vector<float>& local_x,
                       const std::vector<float>& local_y,
                       const Pose2& pose) const;
    int inliers_local(const std::vector<float>& local_x,
                      const std::vector<float>& local_y,
                      const Pose2& pose,
                      double threshold_m) const;

    const std::vector<float>& log_odds() const { return log_odds_; }
    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }
    double origin_x() const { return origin_x_; }
    double origin_y() const { return origin_y_; }

private:
    bool world_to_grid(double x, double y, int& gx, int& gy) const;
    bool grid_to_world(int gx, int gy, double& x, double& y) const;
    float likelihood_at(double x, double y) const;

    double resolution_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;
    std::vector<float> log_odds_;
    std::vector<float> likelihood_;
};

/// 2D lidar SLAM: scan-to-map correlative matching + occupancy mapping.
class SlamCore {
public:
    explicit SlamCore(double map_resolution = 0.05,
                      int width_cells = 300,
                      int height_cells = 160,
                      double origin_x = 0.0,
                      double origin_y = 0.0,
                      double lidar_offset_x = 0.10,
                      double lidar_offset_y = 0.0,
                      double max_range = 12.0);

    SlamResult process(const Scan& scan, const Pose2& odometry);
    void reset();

    Pose2 pose() const { return pose_; }
    int scan_count() const { return scan_count_; }
    double map_resolution() const { return grid_.resolution(); }
    int map_width() const { return grid_.width(); }
    int map_height() const { return grid_.height(); }
    double map_origin_x() const { return grid_.origin_x(); }
    double map_origin_y() const { return grid_.origin_y(); }
    std::vector<float> map_log_odds() const { return grid_.log_odds(); }

private:
    SlamResult match_and_map(const Scan& scan, const Pose2& odometry);

    OccupancyGrid grid_;
    Pose2 pose_{};
    int scan_count_ = 0;
    double lidar_offset_x_;
    double lidar_offset_y_;
    double max_range_;
};

}  // namespace awake_slam
