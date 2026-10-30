#include "awake_slam/pose_graph.hpp"

#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

namespace awake_slam {
namespace {
constexpr double kPi = 3.14159265358979323846;
}

double wrap_angle(double angle) {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

Pose2 compose(const Pose2& a, const Pose2& b) {
    const double c = std::cos(a.theta);
    const double s = std::sin(a.theta);
    return Pose2{
        a.x + c * b.x - s * b.y,
        a.y + s * b.x + c * b.y,
        wrap_angle(a.theta + b.theta),
    };
}

Pose2 inverse(const Pose2& a) {
    const double c = std::cos(a.theta);
    const double s = std::sin(a.theta);
    return Pose2{
        -c * a.x - s * a.y,
        s * a.x - c * a.y,
        wrap_angle(-a.theta),
    };
}

Pose2 relative(const Pose2& from, const Pose2& to) {
    return compose(inverse(from), to);
}

int PoseGraph::add_node(const Pose2& pose) {
    poses_.push_back(pose);
    return static_cast<int>(poses_.size()) - 1;
}

void PoseGraph::add_edge(const GraphEdge& edge) {
    if (edge.from < 0 || edge.to < 0 || edge.from >= node_count() || edge.to >= node_count()) {
        throw std::out_of_range("pose graph edge references an unknown node");
    }
    edges_.push_back(edge);
}

void PoseGraph::clear() {
    poses_.clear();
    edges_.clear();
}

void PoseGraph::optimize(int iterations) {
    const int n = node_count();
    if (n < 2 || edges_.empty()) return;
    const int dim = 3 * n;

    for (int iteration = 0; iteration < iterations; ++iteration) {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dim, dim);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(dim);

        for (const GraphEdge& edge : edges_) {
            const Pose2& pi = poses_[edge.from];
            const Pose2& pj = poses_[edge.to];
            const double c = std::cos(pi.theta);
            const double s = std::sin(pi.theta);
            const double wx = pj.x - pi.x;
            const double wy = pj.y - pi.y;
            const double dx = c * wx + s * wy;
            const double dy = -s * wx + c * wy;
            const double dtheta = wrap_angle(pj.theta - pi.theta);

            Eigen::Vector3d residual(
                dx - edge.dx,
                dy - edge.dy,
                wrap_angle(dtheta - edge.dtheta));

            double weight = edge.information;
            if (edge.loop) {
                const double norm = residual.norm();
                constexpr double huber = 0.20;
                if (norm > huber) weight *= huber / norm;
            }

            Eigen::Matrix<double, 3, 3> Ji = Eigen::Matrix<double, 3, 3>::Zero();
            Eigen::Matrix<double, 3, 3> Jj = Eigen::Matrix<double, 3, 3>::Zero();

            Ji(0, 0) = -c;
            Ji(0, 1) = -s;
            Ji(0, 2) = dy;
            Ji(1, 0) = s;
            Ji(1, 1) = -c;
            Ji(1, 2) = -dx;
            Ji(2, 2) = -1.0;

            Jj(0, 0) = c;
            Jj(0, 1) = s;
            Jj(1, 0) = -s;
            Jj(1, 1) = c;
            Jj(2, 2) = 1.0;

            const int i = 3 * edge.from;
            const int j = 3 * edge.to;
            H.block<3, 3>(i, i) += weight * Ji.transpose() * Ji;
            H.block<3, 3>(i, j) += weight * Ji.transpose() * Jj;
            H.block<3, 3>(j, i) += weight * Jj.transpose() * Ji;
            H.block<3, 3>(j, j) += weight * Jj.transpose() * Jj;
            b.segment<3>(i) += weight * Ji.transpose() * residual;
            b.segment<3>(j) += weight * Jj.transpose() * residual;
        }

        // Fix node 0 by removing its rows/columns and setting a unit prior.
        for (int k = 0; k < 3; ++k) {
            H.row(k).setZero();
            H.col(k).setZero();
            H(k, k) = 1.0;
            b(k) = 0.0;
        }

        const Eigen::VectorXd delta = H.ldlt().solve(-b);
        if (!delta.allFinite()) break;
        for (int i = 0; i < n; ++i) {
            poses_[i].x += delta[3 * i];
            poses_[i].y += delta[3 * i + 1];
            poses_[i].theta = wrap_angle(poses_[i].theta + delta[3 * i + 2]);
        }
        if (delta.norm() < 1e-5) break;
    }
}

}  // namespace awake_slam
