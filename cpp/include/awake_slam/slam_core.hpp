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
};

// Skeleton for M0. The full pipeline (deskew, CSM, ICP, occupancy grid, pose
// graph, MCL) lands in M3. This class currently validates inputs and integrates
// the supplied odometry so the binding, build and protocol can be tested.
class SlamCore {
public:
    explicit SlamCore(double map_resolution = 0.05);

    SlamResult process(const Scan& scan, const Pose2& odometry);
    void reset();

    Pose2 pose() const { return pose_; }
    int scan_count() const { return scan_count_; }
    double map_resolution() const { return map_resolution_; }

private:
    Pose2 pose_{};
    int scan_count_ = 0;
    double map_resolution_ = 0.05;
};

}  // namespace awake_slam
