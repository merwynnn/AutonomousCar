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

}  // namespace awake_slam
