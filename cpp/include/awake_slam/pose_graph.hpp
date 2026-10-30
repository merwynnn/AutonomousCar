#pragma once

#include <vector>

#include "awake_slam/types.hpp"

namespace awake_slam {

struct GraphEdge {
    int from = 0;
    int to = 0;
    double dx = 0.0;
    double dy = 0.0;
    double dtheta = 0.0;
    double information = 1.0;
    bool loop = false;
};

/// Small dense SE(2) pose graph for educational loop closure.
class PoseGraph {
public:
    int add_node(const Pose2& pose);
    void add_edge(const GraphEdge& edge);
    void clear();

    /// Robust Gauss-Newton on the SE(2) graph. Node 0 is fixed.
    void optimize(int iterations = 8);

    const std::vector<Pose2>& poses() const { return poses_; }
    std::vector<Pose2>& poses() { return poses_; }
    const std::vector<GraphEdge>& edges() const { return edges_; }
    int node_count() const { return static_cast<int>(poses_.size()); }
    int edge_count() const { return static_cast<int>(edges_.size()); }

private:
    std::vector<Pose2> poses_;
    std::vector<GraphEdge> edges_;
};

Pose2 compose(const Pose2& a, const Pose2& b);
Pose2 inverse(const Pose2& a);
Pose2 relative(const Pose2& from, const Pose2& to);
double wrap_angle(double angle);

}  // namespace awake_slam
