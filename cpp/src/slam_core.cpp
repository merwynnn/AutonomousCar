#include "awake_slam/slam_core.hpp"

#include <stdexcept>

namespace awake_slam {

SlamCore::SlamCore(double map_resolution) : map_resolution_(map_resolution) {
    if (map_resolution_ <= 0.0) {
        throw std::invalid_argument("map_resolution must be positive");
    }
}

SlamResult SlamCore::process(const Scan& scan, const Pose2& odometry) {
    if (!scan.valid()) {
        throw std::invalid_argument("scan ranges and angles must be non-empty and equal length");
    }

    // TODO(M3): deskew, correlative scan matching, ICP, occupancy grid update,
    // pose graph and loop closure. For now the supplied odometry is the pose;
    // this keeps the build/binding/protocol testable without pretending to be SLAM.
    pose_ = odometry;
    ++scan_count_;

    SlamResult result;
    result.pose = pose_;
    result.initialized = true;
    result.scan_count = scan_count_;
    return result;
}

void SlamCore::reset() {
    pose_ = Pose2{};
    scan_count_ = 0;
}

}  // namespace awake_slam
