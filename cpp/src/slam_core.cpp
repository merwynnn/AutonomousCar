#include "awake_slam/slam_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace awake_slam {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kLikelihoodSigma = 0.15;  // metres
constexpr double kFreeLogOdds = -0.35;
constexpr double kOccupiedLogOdds = 0.90;
constexpr double kMinLogOdds = -4.0;
constexpr double kMaxLogOdds = 4.0;

inline double clamp(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}

inline double wrap_angle(double angle) {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

}  // namespace

OccupancyGrid::OccupancyGrid(double resolution,
                             int width,
                             int height,
                             double origin_x,
                             double origin_y)
    : resolution_(resolution),
      width_(width),
      height_(height),
      origin_x_(origin_x),
      origin_y_(origin_y),
      log_odds_(static_cast<std::size_t>(width) * height, 0.0f),
      likelihood_(static_cast<std::size_t>(width) * height, 0.0f) {
    if (resolution <= 0.0 || width <= 0 || height <= 0) {
        throw std::invalid_argument("invalid occupancy grid dimensions");
    }
}

void OccupancyGrid::reset() {
    std::fill(log_odds_.begin(), log_odds_.end(), 0.0f);
    std::fill(likelihood_.begin(), likelihood_.end(), 0.0f);
}

bool OccupancyGrid::world_to_grid(double x, double y, int& gx, int& gy) const {
    gx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    gy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

bool OccupancyGrid::grid_to_world(int gx, int gy, double& x, double& y) const {
    if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) return false;
    x = origin_x_ + (gx + 0.5) * resolution_;
    y = origin_y_ + (gy + 0.5) * resolution_;
    return true;
}

void OccupancyGrid::update(const Pose2& pose,
                           const Scan& scan,
                           double lidar_offset_x,
                           double lidar_offset_y,
                           double max_range) {
    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    const double start_x = pose.x + c * lidar_offset_x - s * lidar_offset_y;
    const double start_y = pose.y + s * lidar_offset_x + c * lidar_offset_y;
    int sx = 0;
    int sy = 0;
    if (!world_to_grid(start_x, start_y, sx, sy)) return;

    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
        const float raw = scan.ranges[i];
        if (!std::isfinite(raw) || raw <= 0.03f || raw > max_range) continue;
        const double angle = pose.theta + scan.angles[i];
        const double ex = start_x + std::cos(angle) * raw;
        const double ey = start_y + std::sin(angle) * raw;
        int gx = 0;
        int gy = 0;
        if (!world_to_grid(ex, ey, gx, gy)) continue;

        int x = sx;
        int y = sy;
        const int dx = std::abs(gx - x);
        const int dy = -std::abs(gy - y);
        const int step_x = x < gx ? 1 : -1;
        const int step_y = y < gy ? 1 : -1;
        int error = dx + dy;

        while (x != gx || y != gy) {
            const std::size_t idx = static_cast<std::size_t>(y) * width_ + x;
            log_odds_[idx] = static_cast<float>(
                clamp(log_odds_[idx] + kFreeLogOdds, kMinLogOdds, kMaxLogOdds));
            const int e2 = 2 * error;
            if (e2 >= dy) {
                error += dy;
                x += step_x;
            }
            if (e2 <= dx) {
                error += dx;
                y += step_y;
            }
        }
        const std::size_t end_idx = static_cast<std::size_t>(gy) * width_ + gx;
        log_odds_[end_idx] = static_cast<float>(
            clamp(log_odds_[end_idx] + kOccupiedLogOdds, kMinLogOdds, kMaxLogOdds));
    }
}

void OccupancyGrid::rebuild_likelihood(double sigma_m) {
    const float inf = std::numeric_limits<float>::infinity();
    std::vector<float> distance(static_cast<std::size_t>(width_) * height_, inf);
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            const std::size_t idx = static_cast<std::size_t>(y) * width_ + x;
            if (log_odds_[idx] > 0.5f) distance[idx] = 0.0f;
        }
    }
    constexpr float diagonal = 1.41421356f;
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            const std::size_t idx = static_cast<std::size_t>(y) * width_ + x;
            float d = distance[idx];
            if (x > 0) d = std::min(d, distance[idx - 1] + 1.0f);
            if (y > 0) {
                d = std::min(d, distance[idx - width_] + 1.0f);
                if (x > 0) d = std::min(d, distance[idx - width_ - 1] + diagonal);
                if (x + 1 < width_) d = std::min(d, distance[idx - width_ + 1] + diagonal);
            }
            distance[idx] = d;
        }
    }
    for (int y = height_ - 1; y >= 0; --y) {
        for (int x = width_ - 1; x >= 0; --x) {
            const std::size_t idx = static_cast<std::size_t>(y) * width_ + x;
            float d = distance[idx];
            if (x + 1 < width_) d = std::min(d, distance[idx + 1] + 1.0f);
            if (y + 1 < height_) {
                d = std::min(d, distance[idx + width_] + 1.0f);
                if (x > 0) d = std::min(d, distance[idx + width_ - 1] + diagonal);
                if (x + 1 < width_) d = std::min(d, distance[idx + width_ + 1] + diagonal);
            }
            distance[idx] = d;
        }
    }
    const double denom = 2.0 * sigma_m * sigma_m;
    for (std::size_t i = 0; i < distance.size(); ++i) {
        if (!std::isfinite(distance[i])) {
            likelihood_[i] = 0.0f;
        } else {
            const double metres = distance[i] * resolution_;
            likelihood_[i] = static_cast<float>(std::exp(-metres * metres / denom));
        }
    }
}

float OccupancyGrid::likelihood_at(double x, double y) const {
    int gx = 0;
    int gy = 0;
    if (!world_to_grid(x, y, gx, gy)) return 0.0f;
    return likelihood_[static_cast<std::size_t>(gy) * width_ + gx];
}

double OccupancyGrid::score_local(const std::vector<float>& local_x,
                                  const std::vector<float>& local_y,
                                  const Pose2& pose) const {
    if (local_x.empty()) return 0.0;
    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    double sum = 0.0;
    for (std::size_t i = 0; i < local_x.size(); ++i) {
        const double wx = pose.x + c * local_x[i] - s * local_y[i];
        const double wy = pose.y + s * local_x[i] + c * local_y[i];
        sum += likelihood_at(wx, wy);
    }
    return sum / static_cast<double>(local_x.size());
}

int OccupancyGrid::inliers_local(const std::vector<float>& local_x,
                                 const std::vector<float>& local_y,
                                 const Pose2& pose,
                                 double threshold_m) const {
    if (local_x.empty()) return 0;
    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    const double threshold =
        std::exp(-0.5 * (threshold_m / kLikelihoodSigma) * (threshold_m / kLikelihoodSigma));
    int count = 0;
    for (std::size_t i = 0; i < local_x.size(); ++i) {
        const double wx = pose.x + c * local_x[i] - s * local_y[i];
        const double wy = pose.y + s * local_x[i] + c * local_y[i];
        if (likelihood_at(wx, wy) >= threshold) ++count;
    }
    return count;
}

SlamCore::SlamCore(double map_resolution,
                   int width_cells,
                   int height_cells,
                   double origin_x,
                   double origin_y,
                   double lidar_offset_x,
                   double lidar_offset_y,
                   double max_range)
    : grid_(map_resolution, width_cells, height_cells, origin_x, origin_y),
      lidar_offset_x_(lidar_offset_x),
      lidar_offset_y_(lidar_offset_y),
      max_range_(max_range) {
    if (max_range <= 0.0) throw std::invalid_argument("max_range must be positive");
}

SlamResult SlamCore::process(const Scan& scan, const Pose2& odometry) {
    if (!scan.valid()) {
        throw std::invalid_argument("scan ranges and angles must be non-empty and equal length");
    }
    if (scan_count_ == 0) {
        pose_ = odometry;
        grid_.update(pose_, scan, lidar_offset_x_, lidar_offset_y_, max_range_);
        grid_.rebuild_likelihood();
        ++scan_count_;
        SlamResult result;
        result.pose = pose_;
        result.initialized = true;
        result.scan_count = scan_count_;
        result.match_score = 1.0;
        result.inliers = static_cast<int>(scan.ranges.size());
        return result;
    }
    return match_and_map(scan, odometry);
}

SlamResult SlamCore::match_and_map(const Scan& scan, const Pose2& odometry) {
    std::vector<float> local_x;
    std::vector<float> local_y;
    local_x.reserve(scan.ranges.size());
    local_y.reserve(scan.ranges.size());
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
        const float r = scan.ranges[i];
        if (!std::isfinite(r) || r <= 0.03f || r > max_range_) continue;
        const double angle = scan.angles[i];
        local_x.push_back(static_cast<float>(lidar_offset_x_ + r * std::cos(angle)));
        local_y.push_back(static_cast<float>(lidar_offset_y_ + r * std::sin(angle)));
    }
    if (local_x.empty()) {
        return {pose_, true, scan_count_, 0.0, 0};
    }

    // A motion prior keeps the matcher from jumping between repetitive wall
    // segments when odometry is already a good local estimate.
    const double sigma_xy = 0.25;
    const double sigma_theta = 0.15;
    auto objective = [&](const Pose2& candidate,
                         const std::vector<float>& px,
                         const std::vector<float>& py) {
        const double dx = candidate.x - odometry.x;
        const double dy = candidate.y - odometry.y;
        const double dth = wrap_angle(candidate.theta - odometry.theta);
        const double prior = 0.5 * (dx * dx / (sigma_xy * sigma_xy) +
                                    dy * dy / (sigma_xy * sigma_xy) +
                                    dth * dth / (sigma_theta * sigma_theta));
        return grid_.score_local(px, py, candidate) - prior;
    };

    // Coarse search on every second point, then fine search on all points.
    std::vector<float> sub_x;
    std::vector<float> sub_y;
    sub_x.reserve(local_x.size() / 2 + 1);
    sub_y.reserve(local_y.size() / 2 + 1);
    for (std::size_t i = 0; i < local_x.size(); i += 2) {
        sub_x.push_back(local_x[i]);
        sub_y.push_back(local_y[i]);
    }

    Pose2 best = odometry;
    double best_objective = objective(best, sub_x, sub_y);
    constexpr double deg = kPi / 180.0;
    for (int ix = -5; ix <= 5; ++ix) {
        for (int iy = -5; iy <= 5; ++iy) {
            for (int it = -5; it <= 5; ++it) {
                Pose2 candidate{
                    odometry.x + ix * 0.05,
                    odometry.y + iy * 0.05,
                    wrap_angle(odometry.theta + it * 1.5 * deg),
                };
                const double value = objective(candidate, sub_x, sub_y);
                if (value > best_objective) {
                    best_objective = value;
                    best = candidate;
                }
            }
        }
    }
    best_objective = objective(best, local_x, local_y);
    for (int ix = -4; ix <= 4; ++ix) {
        for (int iy = -4; iy <= 4; ++iy) {
            for (int it = -3; it <= 3; ++it) {
                Pose2 candidate{
                    best.x + ix * 0.01,
                    best.y + iy * 0.01,
                    wrap_angle(best.theta + it * 0.5 * deg),
                };
                const double value = objective(candidate, local_x, local_y);
                if (value > best_objective) {
                    best_objective = value;
                    best = candidate;
                }
            }
        }
    }

    const double best_score = grid_.score_local(local_x, local_y, best);
    if (best_score < 0.03) {
        best = odometry;
    }
    pose_ = best;
    grid_.update(pose_, scan, lidar_offset_x_, lidar_offset_y_, max_range_);
    grid_.rebuild_likelihood();
    ++scan_count_;
    SlamResult result;
    result.pose = pose_;
    result.initialized = true;
    result.scan_count = scan_count_;
    result.match_score = best_score;
    result.inliers = grid_.inliers_local(local_x, local_y, pose_, 0.20);
    return result;
}

void SlamCore::reset() {
    grid_.reset();
    pose_ = Pose2{};
    scan_count_ = 0;
}

}  // namespace awake_slam
