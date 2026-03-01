#pragma once

#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>

#include "automap_pro/core/data_types.h"
#include "automap_pro/backend/pose_graph.h"
#include "automap_pro/backend/optimizer.h"

namespace automap_pro {

// ──────────────────────────────────────────────────────────
// HBAWrapper: Hierarchical Bundle Adjustment
// Level 2: Submap-level pose graph
// Level 1: Keyframe-level per-submap pose graph
// Level 0: Point cloud BA (optional, expensive)
// ──────────────────────────────────────────────────────────
class HBAWrapper {
public:
    using PoseUpdateCallback = std::function<void(int submap_id, const Pose3d& new_pose)>;
    using OptimizationDoneCallback = std::function<void()>;

    HBAWrapper();
    ~HBAWrapper();

    void init(rclcpp::Node::SharedPtr node);
    void start();
    void stop();

    // Add factors
    void addOdomFactor(int kf_from, int kf_to,
                        const Pose3d& relative_pose,
                        const Mat66d& information);

    void addLoopFactor(const LoopConstraint::Ptr& lc);

    void addGPSFactor(int kf_id, const Eigen::Vector3d& gps_pos,
                       const Eigen::Matrix3d& gps_cov);

    void addSubmapOdomFactor(int sm_from, int sm_to,
                              const Pose3d& relative_pose,
                              const Mat66d& information);

    void addSubmapNode(int sm_id, const Pose3d& pose, bool fixed = false);
    void addKeyFrameNode(int kf_id, const Pose3d& pose, bool fixed = false);

    // Trigger optimization
    void triggerOptimization(bool full = false);

    void registerPoseUpdateCallback(PoseUpdateCallback cb);
    void registerDoneCallback(OptimizationDoneCallback cb);

    // Get result
    std::map<int, Pose3d> getOptimizedSubmapPoses() const;
    std::map<int, Pose3d> getOptimizedKeyFramePoses() const;

    bool isOptimizing() const;

    // Exposed for system-level access
    PoseGraph level2_graph_;
    PoseGraph level1_graph_;

private:
    void workerLoop();
    void runLevel2Optimization();   // Submap level
    void runLevel1Optimization();   // Keyframe level
    void broadcastPoseUpdates();

    Optimizer optimizer_;

    std::queue<bool>          opt_queue_;   // true = full, false = incremental
    std::mutex                queue_mutex_;
    std::condition_variable   queue_cv_;
    std::thread               worker_thread_;
    std::atomic<bool>         running_{false};
    std::atomic<bool>         optimizing_{false};

    std::vector<PoseUpdateCallback>        pose_update_cbs_;
    std::vector<OptimizationDoneCallback>  done_cbs_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_path_pub_;

    mutable std::mutex result_mutex_;
    std::map<int, Pose3d> optimized_submap_poses_;
    std::map<int, Pose3d> optimized_kf_poses_;

    int    max_iterations_     = 100;
    double convergence_thresh_ = 1e-4;
    bool   robust_kernel_      = true;
    double robust_delta_       = 1.0;
};

}  // namespace automap_pro
