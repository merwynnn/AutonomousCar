#include <cmath>
#include <iostream>

#include "awake_slam/pose_graph.hpp"

namespace {

int failures = 0;

void check(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        ++failures;
    }
}

bool close(double a, double b, double eps) { return std::abs(a - b) < eps; }

}  // namespace

int main() {
    using awake_slam::GraphEdge;
    using awake_slam::Pose2;
    using awake_slam::PoseGraph;
    using awake_slam::relative;

    const double pi = 3.14159265358979323846;
    const Pose2 truth[4] = {
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {1.0, 1.0, pi / 2.0},
        {0.0, 1.0, pi},
    };
    // Drifted initial poses: yaw error and translation error accumulate.
    const Pose2 drifted[4] = {
        {0.00, 0.00, 0.00},
        {1.10, 0.00, 0.02},
        {1.15, 1.02, 1.55},
        {0.12, 1.12, 3.12},
    };

    PoseGraph graph;
    for (const Pose2& pose : drifted) graph.add_node(pose);

    for (int i = 0; i < 3; ++i) {
        const Pose2 rel = relative(truth[i], truth[i + 1]);
        GraphEdge edge;
        edge.from = i;
        edge.to = i + 1;
        edge.dx = rel.x;
        edge.dy = rel.y;
        edge.dtheta = rel.theta;
        edge.information = 1.0 / (0.03 * 0.03);
        graph.add_edge(edge);
    }
    // The loop closure measurement is taken from ground truth and closes 3 -> 0.
    {
        const Pose2 rel = relative(truth[3], truth[0]);
        GraphEdge edge;
        edge.from = 3;
        edge.to = 0;
        edge.dx = rel.x;
        edge.dy = rel.y;
        edge.dtheta = rel.theta;
        edge.information = 1.0 / (0.05 * 0.05);
        edge.loop = true;
        graph.add_edge(edge);
    }

    graph.optimize(20);
    const auto& poses = graph.poses();
    check(graph.node_count() == 4, "node count");
    check(graph.edge_count() == 4, "edge count");
    check(close(poses[0].x, truth[0].x, 1e-9), "node 0 fixed x");
    check(close(poses[1].x, truth[1].x, 0.05), "node 1 corrected x");
    check(close(poses[2].x, truth[2].x, 0.05), "node 2 corrected x");
    check(close(poses[3].x, truth[3].x, 0.05), "node 3 corrected x");
    check(close(poses[3].y, truth[3].y, 0.05), "node 3 corrected y");
    check(std::abs(std::sin(poses[3].theta - truth[3].theta)) < 0.08, "node 3 corrected yaw");

    if (failures == 0) {
        std::cout << "pose graph tests passed\n";
        return 0;
    }
    std::cerr << failures << " test(s) failed\n";
    return 1;
}
