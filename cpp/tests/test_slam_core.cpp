#include <cmath>
#include <iostream>
#include <vector>

#include "awake_slam/slam_core.hpp"

namespace {

int failures = 0;

void check(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        ++failures;
    }
}

bool close(double a, double b, double eps = 1e-9) { return std::abs(a - b) < eps; }

}  // namespace

int main() {
    using awake_slam::Pose2;
    using awake_slam::Scan;
    using awake_slam::SlamCore;

    SlamCore core(0.05, 300, 160, 0.0, 0.0, 0.10, 0.0, 12.0);
    check(close(core.map_resolution(), 0.05), "map resolution");
    check(core.map_width() == 300, "map width");
    check(core.map_height() == 160, "map height");
    check(core.scan_count() == 0, "initial scan count");

    // A simple 181-beam fan facing a wall 2 m ahead plus side returns.
    std::vector<float> ranges;
    std::vector<float> angles;
    for (int i = 0; i < 181; ++i) {
        const double angle = -1.5707963 + 3.1415926 * i / 180.0;
        ranges.push_back(2.0f);
        angles.push_back(static_cast<float>(angle));
    }
    Scan scan{ranges, angles, 0.0};
    check(scan.valid(), "scan valid");
    check(scan.size() == 181, "scan size");

    auto first = core.process(scan, Pose2{1.0, 2.0, 0.0});
    check(first.initialized, "first initialized");
    check(first.scan_count == 1, "first scan count");
    check(close(first.pose.x, 1.0), "first pose x");
    auto map = core.map_log_odds();
    check(map.size() == static_cast<std::size_t>(core.map_width() * core.map_height()),
          "map size");
    bool has_occupied = false;
    for (float value : map) {
        if (value > 0.5f) {
            has_occupied = true;
            break;
        }
    }
    check(has_occupied, "map has occupied cells");

    auto second = core.process(scan, Pose2{1.08, 2.02, 0.01});
    check(second.scan_count == 2, "second scan count");
    check(std::isfinite(second.pose.x) && std::isfinite(second.pose.theta), "second pose finite");
    check(second.match_score >= 0.0, "match score");
    check(second.inliers > 0, "inliers");

    core.reset();
    check(core.scan_count() == 0, "reset scan count");
    check(close(core.pose().x, 0.0), "reset pose");

    Scan invalid{{1.0f}, {}, 0.0};
    check(!invalid.valid(), "invalid scan detected");

    if (failures == 0) {
        std::cout << "awake_slam core tests passed\n";
        return 0;
    }
    std::cerr << failures << " test(s) failed\n";
    return 1;
}
