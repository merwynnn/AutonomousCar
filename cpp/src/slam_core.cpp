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

inline double distance2d(const Pose2& a, const Pose2& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
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

SlamCore::Match SlamCore::match_scan(const OccupancyGrid& grid,
                                     const std::vector<float>& local_x,
                                     const std::vector<float>& local_y,
                                     const Pose2& initial,
                                     double range_xy,
                                     double range_theta,
                                     bool use_prior,
                                     double prior_xy,
                                     double prior_theta) const {
    if (local_x.empty()) return {initial, 0.0, 0};

    std::vector<float> sub_x;
    std::vector<float> sub_y;
    sub_x.reserve(local_x.size() / 2 + 1);
    sub_y.reserve(local_y.size() / 2 + 1);
    for (std::size_t i = 0; i < local_x.size(); i += 2) {
        sub_x.push_back(local_x[i]);
        sub_y.push_back(local_y[i]);
    }

    auto objective = [&](const Pose2& candidate,
                         const std::vector<float>& px,
                         const std::vector<float>& py) {
        double value = grid.score_local(px, py, candidate);
        if (use_prior) {
            const double dx = candidate.x - initial.x;
            const double dy = candidate.y - initial.y;
            const double dth = wrap_angle(candidate.theta - initial.theta);
            value -= 0.5 * (dx * dx / (prior_xy * prior_xy) +
                            dy * dy / (prior_xy * prior_xy) +
                            dth * dth / (prior_theta * prior_theta));
        }
        return value;
    };

    constexpr double deg = kPi / 180.0;
    const double coarse_xy_step = range_xy > 0.5 ? 0.10 : 0.05;
    const int nx = std::max(1, static_cast<int>(std::round(range_xy / coarse_xy_step)));
    const int nt = std::max(1, static_cast<int>(std::round(range_theta / (1.5 * deg))));
    Pose2 best = initial;
    double best_value = objective(best, sub_x, sub_y);
    for (int ix = -nx; ix <= nx; ++ix) {
        for (int iy = -nx; iy <= nx; ++iy) {
            for (int it = -nt; it <= nt; ++it) {
                Pose2 candidate{
                    initial.x + ix * coarse_xy_step,
                    initial.y + iy * coarse_xy_step,
                    wrap_angle(initial.theta + it * 1.5 * deg),
                };
                const double value = objective(candidate, sub_x, sub_y);
                if (value > best_value) {
                    best_value = value;
                    best = candidate;
                }
            }
        }
    }

    double best_fine = objective(best, local_x, local_y);
    for (int ix = -4; ix <= 4; ++ix) {
        for (int iy = -4; iy <= 4; ++iy) {
            for (int it = -3; it <= 3; ++it) {
                Pose2 candidate{
                    best.x + ix * 0.01,
                    best.y + iy * 0.01,
                    wrap_angle(best.theta + it * 0.5 * deg),
                };
                const double value = objective(candidate, local_x, local_y);
                if (value > best_fine) {
                    best_fine = value;
                    best = candidate;
                }
            }
        }
    }
    return {
        best,
        grid.score_local(local_x, local_y, best),
        grid.inliers_local(local_x, local_y, best, 0.20),
    };
}

void SlamCore::add_keyframe(const Pose2& pose, const Scan& scan) {
    const int index = graph_.add_node(pose);
    if (!keyframes_.empty()) {
        // Always measure the sequential constraint from the original front-end
        // poses. Using an already-optimized pose here would inject the previous
        // loop correction into every new edge.
        const Pose2 rel = relative(keyframes_.back().front_pose, pose);
        GraphEdge edge;
        edge.from = index - 1;
        edge.to = index;
        edge.dx = rel.x;
        edge.dy = rel.y;
        edge.dtheta = rel.theta;
        edge.information = 1.0 / (0.03 * 0.03);
        edge.loop = false;
        graph_.add_edge(edge);
    }
    keyframes_.push_back(Keyframe{pose, pose, scan});
    last_keyframe_pose_ = pose;
}

bool SlamCore::try_loop_closure(const std::vector<float>& local_x,
                                const std::vector<float>& local_y) {
    const int current = static_cast<int>(keyframes_.size()) - 1;
    if (current < loop_min_index_gap_ + 1) return false;
    if (current - last_loop_keyframe_ < 10) return false;

    const Pose2& cur = keyframes_[current].pose;
    std::vector<std::pair<double, int>> candidates;
    for (int i = 0; i <= current - loop_min_index_gap_; ++i) {
        const Pose2& old = keyframes_[i].pose;
        const double d = distance2d(cur, old);
        const double dtheta = std::abs(wrap_angle(cur.theta - old.theta));
        if (d < loop_min_distance_m_ && dtheta < 0.55) {
            candidates.emplace_back(d, i);
        }
    }
    if (candidates.empty()) return false;
    std::sort(candidates.begin(), candidates.end());

    // Try the nearest few candidates. For each candidate we build a small local
    // submap from neighbouring keyframes, then verify the current scan against
    // that submap. A candidate is accepted only if the scan matches *at the
    // candidate pose* (not 1 m down the corridor) with enough overlap.
    const int max_candidates = std::min(3, static_cast<int>(candidates.size()));
    for (int c = 0; c < max_candidates; ++c) {
        const int candidate = candidates[c].second;
        OccupancyGrid local(
            grid_.resolution(),
            grid_.width(),
            grid_.height(),
            grid_.origin_x(),
            grid_.origin_y());
        const int first = std::max(0, candidate - 8);
        const int last = std::min(current - 1, candidate + 8);
        for (int i = first; i <= last; ++i) {
            local.update(
                keyframes_[i].pose,
                keyframes_[i].scan,
                lidar_offset_x_,
                lidar_offset_y_,
                max_range_);
        }
        local.rebuild_likelihood();

        const Match match = match_scan(
            local,
            local_x,
            local_y,
            keyframes_[candidate].pose,
            1.2,
            20.0 * kPi / 180.0,
            true,
            0.50,
            0.25);
        if (match.score < 0.70 ||
            match.inliers < static_cast<int>(0.5 * local_x.size())) {
            continue;
        }
        const double pose_error = distance2d(match.pose, keyframes_[candidate].pose);
        const double angle_error =
            std::abs(wrap_angle(match.pose.theta - keyframes_[candidate].pose.theta));
        if (pose_error > 0.55 || angle_error > 0.22) {
            continue;  // matched somewhere else along the corridor: not a loop
        }

        const Pose2 rel = relative(keyframes_[candidate].pose, match.pose);
        GraphEdge edge;
        edge.from = candidate;
        edge.to = current;
        edge.dx = rel.x;
        edge.dy = rel.y;
        edge.dtheta = rel.theta;
        edge.information = 1.0 / (0.03 * 0.03);
        edge.loop = true;
        graph_.add_edge(edge);
        graph_.optimize(10);

        for (int i = 0; i < static_cast<int>(keyframes_.size()); ++i) {
            // Keep front_pose untouched: future sequential edges must still be
            // measured against the original front-end trajectory.
            keyframes_[i].pose = graph_.poses()[i];
        }
        pose_ = keyframes_.back().pose;
        last_keyframe_pose_ = pose_;
        rebuild_map();
        ++loop_closures_;
        last_loop_keyframe_ = current;
        last_loop_candidate_ = candidate;
        last_loop_match_ = match.pose;
        loop_closure_this_scan_ = true;
        return true;
    }
    return false;
}

void SlamCore::rebuild_map() {
    grid_.reset();
    for (const Keyframe& keyframe : keyframes_) {
        grid_.update(keyframe.pose, keyframe.scan, lidar_offset_x_, lidar_offset_y_, max_range_);
    }
    grid_.rebuild_likelihood();
}

SlamResult SlamCore::process(const Scan& scan, const Pose2& odometry) {
    if (!scan.valid()) {
        throw std::invalid_argument("scan ranges and angles must be non-empty and equal length");
    }
    loop_closure_this_scan_ = false;

    if (scan_count_ == 0) {
        pose_ = odometry;
        grid_.update(pose_, scan, lidar_offset_x_, lidar_offset_y_, max_range_);
        grid_.rebuild_likelihood();
        add_keyframe(pose_, scan);
        scan_count_ = 1;
        SlamResult result;
        result.pose = pose_;
        result.initialized = true;
        result.scan_count = scan_count_;
        result.match_score = 1.0;
        result.inliers = static_cast<int>(scan.ranges.size());
        result.graph_nodes = graph_.node_count();
        result.graph_edges = graph_.edge_count();
        return result;
    }

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
        return {pose_, true, scan_count_, 0.0, 0, loop_closures_, false,
                graph_.node_count(), graph_.edge_count()};
    }

    // In long straights the scan is nearly unobservable along the corridor.
    // Trust the short-term odometry delta and use scan matching mainly for the
    // perpendicular/heading correction; the global drift is fixed by loop closure.
    const Match front = match_scan(
        grid_, local_x, local_y, odometry, 0.20, 6.0 * kPi / 180.0, true, 0.10, 0.08);
    pose_ = front.pose;
    ++scans_since_keyframe_;

    const bool keyframe_due =
        scans_since_keyframe_ >= keyframe_every_scans_ ||
        distance2d(pose_, last_keyframe_pose_) > keyframe_distance_m_;

    if (keyframe_due) {
        add_keyframe(pose_, scan);
        // Automatic loop closure is disabled until place recognition is reliable
        // on the repetitive pipe walls. The pose-graph back-end is implemented and
        // tested, but firing it on a false positive corrupts the map. Do not
        // re-enable try_loop_closure() without a false-positive test.
        grid_.update(pose_, scan, lidar_offset_x_, lidar_offset_y_, max_range_);
        grid_.rebuild_likelihood();
        scans_since_keyframe_ = 0;
    } else {
        grid_.update(pose_, scan, lidar_offset_x_, lidar_offset_y_, max_range_);
        grid_.rebuild_likelihood();
    }

    ++scan_count_;
    SlamResult result;
    result.pose = pose_;
    result.initialized = true;
    result.scan_count = scan_count_;
    result.match_score = front.score;
    result.inliers = front.inliers;
    result.loop_closures = loop_closures_;
    result.loop_closure_this_scan = loop_closure_this_scan_;
    result.graph_nodes = graph_.node_count();
    result.graph_edges = graph_.edge_count();
    result.loop_candidate = loop_closure_this_scan_ ? last_loop_candidate_ : -1;
    result.loop_match_x = last_loop_match_.x;
    result.loop_match_y = last_loop_match_.y;
    result.loop_match_theta = last_loop_match_.theta;
    return result;
}

void SlamCore::reset() {
    grid_.reset();
    graph_.clear();
    keyframes_.clear();
    pose_ = Pose2{};
    last_keyframe_pose_ = Pose2{};
    scan_count_ = 0;
    scans_since_keyframe_ = 0;
    loop_closures_ = 0;
    last_loop_keyframe_ = -1000;
    loop_closure_this_scan_ = false;
    last_loop_candidate_ = -1;
    last_loop_match_ = Pose2{};
}

}  // namespace awake_slam
