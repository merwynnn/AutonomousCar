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

    SlamCore core(0.025);
    check(close(core.map_resolution(), 0.025), "map resolution");
    check(core.scan_count() == 0, "initial scan count");

    Scan scan{{1.0f, 2.0f, 3.0f}, {0.0f, 0.1f, 0.2f}, 0.0};
    check(scan.valid(), "scan valid");
    check(scan.size() == 3, "scan size");

    auto result = core.process(scan, Pose2{0.5, 1.0, 0.2});
    check(result.initialized, "result initialized");
    check(result.scan_count == 1, "result scan count");
    check(close(result.pose.x, 0.5), "pose x");
    check(close(result.pose.y, 1.0), "pose y");
    check(close(result.pose.theta, 0.2), "pose theta");

    auto second = core.process(scan, Pose2{0.6, 1.1, 0.25});
    check(second.scan_count == 2, "second scan count");
    check(close(second.pose.x, 0.6), "second pose x");

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
