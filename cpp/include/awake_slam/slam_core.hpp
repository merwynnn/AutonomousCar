#pragma once

#include <cstddef>
#include <random>
#include <vector>

#include "awake_slam/pose_graph.hpp"
#include "awake_slam/types.hpp"

namespace awake_slam {

struct SlamResult {
    Pose2 pose;
    bool initialized = false;
    int scan_count = 0;
    double match_score = 0.0;
    int inliers = 0;
    int loop_closures = 0;
    bool loop_closure_this_scan = false;
    int graph_nodes = 0;
    int graph_edges = 0;
    int loop_candidate = -1;
    double loop_match_x = 0.0;
    double loop_match_y = 0.0;
    double loop_match_theta = 0.0;
    Pose2 mcl_pose;
    bool mcl_initialized = false;
    double mcl_ess = 0.0;
    double mcl_spread = 0.0;
    int mcl_particle_count = 0;
};

/// Log-odds occupancy grid with a chamfer likelihood field for scan matching.
class OccupancyGrid {
public:
    OccupancyGrid(double resolution, int width, int height, double origin_x, double origin_y);

    void reset();
    void update(const Pose2& pose,
                const Scan& scan,
                double lidar_offset_x,
                double lidar_offset_y,
                double max_range);
    void rebuild_likelihood(double sigma_m = 0.15);

    double score_local(const std::vector<float>& local_x,
                       const std::vector<float>& local_y,
                       const Pose2& pose) const;
    int inliers_local(const std::vector<float>& local_x,
                      const std::vector<float>& local_y,
                      const Pose2& pose,
                      double threshold_m) const;

    const std::vector<float>& log_odds() const { return log_odds_; }
    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }
    double origin_x() const { return origin_x_; }
    double origin_y() const { return origin_y_; }

private:
    bool world_to_grid(double x, double y, int& gx, int& gy) const;
    bool grid_to_world(int gx, int gy, double& x, double& y) const;
    float likelihood_at(double x, double y) const;

    double resolution_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;
    std::vector<float> log_odds_;
    std::vector<float> likelihood_;
};

struct Keyframe {
    Pose2 pose;        // optimized / current map pose
    Pose2 front_pose;  // front-end pose when the keyframe was created
    Scan scan;
};

/// 2D lidar SLAM: deskew, scan matching, occupancy grid, keyframes, SE(2) pose
/// graph and loop closure.
class SlamCore {
public:
    explicit SlamCore(double map_resolution = 0.05,
                      int width_cells = 300,
                      int height_cells = 160,
                      double origin_x = 0.0,
                      double origin_y = 0.0,
                      double lidar_offset_x = 0.10,
                      double lidar_offset_y = 0.0,
                      double max_range = 12.0);

    SlamResult process(const Scan& scan, const Pose2& odometry);
    void reset();

    void set_mcl_enabled(bool enabled);
    void set_loop_closure_enabled(bool enabled);
    bool mcl_enabled() const { return mcl_enabled_; }
    bool loop_closure_enabled() const { return loop_closure_enabled_; }
    std::vector<Pose2> mcl_particles() const { return mcl_particles_; }

    Pose2 pose() const { return pose_; }
    int scan_count() const { return scan_count_; }
    double map_resolution() const { return grid_.resolution(); }
    int map_width() const { return grid_.width(); }
    int map_height() const { return grid_.height(); }
    double map_origin_x() const { return grid_.origin_x(); }
    double map_origin_y() const { return grid_.origin_y(); }
    std::vector<float> map_log_odds() const { return grid_.log_odds(); }

private:
    struct Match {
        Pose2 pose;
        double score = 0.0;
        int inliers = 0;
    };

    Match match_scan(const OccupancyGrid& grid,
                     const std::vector<float>& local_x,
                     const std::vector<float>& local_y,
                     const Pose2& initial,
                     double range_xy,
                     double range_theta,
                     bool use_prior,
                     double prior_xy,
                     double prior_theta) const;
    void add_keyframe(const Pose2& pose, const Scan& scan);
    bool try_loop_closure(const std::vector<float>& local_x,
                          const std::vector<float>& local_y);
    void rebuild_map();
    void mcl_update(const std::vector<float>& local_x,
                    const std::vector<float>& local_y,
                    const Pose2& odometry);

    OccupancyGrid grid_;
    PoseGraph graph_;
    std::vector<Keyframe> keyframes_;
    Pose2 pose_{};
    Pose2 last_keyframe_pose_{};
    int scan_count_ = 0;
    int scans_since_keyframe_ = 0;
    int loop_closures_ = 0;
    int last_loop_keyframe_ = -1000;
    bool loop_closure_this_scan_ = false;
    int last_loop_candidate_ = -1;
    Pose2 last_loop_match_{};
    double lidar_offset_x_;
    double lidar_offset_y_;
    double max_range_;
    int keyframe_every_scans_ = 5;
    double keyframe_distance_m_ = 0.45;
    double loop_min_distance_m_ = 2.0;
    int loop_min_index_gap_ = 12;

    bool mcl_enabled_ = false;
    bool loop_closure_enabled_ = false;
    bool mcl_initialized_ = false;
    Pose2 mcl_pose_{};
    Pose2 last_mcl_odom_{};
    std::vector<Pose2> mcl_particles_;
    std::vector<double> mcl_weights_;
    double mcl_ess_ = 0.0;
    double mcl_spread_ = 0.0;
    std::mt19937 mcl_rng_{0x5A17u};
};

}  // namespace awake_slam
