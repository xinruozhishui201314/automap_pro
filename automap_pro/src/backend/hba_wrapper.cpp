#include "automap_pro/backend/hba_wrapper.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace automap_pro {

HBAWrapper::HBAWrapper() {
    const auto& cfg = ConfigManager::instance();
    max_iterations_     = cfg.hbaMaxIterations();
    convergence_thresh_ = cfg.hbaConvergenceThreshold();
    robust_kernel_      = cfg.hbaRobustKernel();
    robust_delta_       = cfg.hbaRobustKernelDelta();
}

HBAWrapper::~HBAWrapper() {
    stop();
}

void HBAWrapper::init(rclcpp::Node::SharedPtr node) {
    node_ = node;
    Optimizer::Options opts;
    opts.max_iterations        = max_iterations_;
    opts.convergence_threshold = convergence_thresh_;
    opts.use_robust_kernel     = robust_kernel_;
    opts.robust_kernel_delta   = robust_delta_;
    optimizer_.setOptions(opts);

    optimized_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/automap/optimized_path", 1);
    RCLCPP_INFO(node->get_logger(), "[HBAWrapper][TOPIC] publish: /automap/optimized_path | Initialized (max_iter=%d).", max_iterations_);
}

void HBAWrapper::start() {
    running_ = true;
    worker_thread_ = std::thread(&HBAWrapper::workerLoop, this);
    RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[HBAWrapper] Worker thread started.");
}

void HBAWrapper::stop() {
    running_ = false;
    queue_cv_.notify_all();
    if (worker_thread_.joinable()) worker_thread_.join();
}

void HBAWrapper::addOdomFactor(int kf_from, int kf_to,
                                 const Pose3d& rel_pose, const Mat66d& info) {
    level1_graph_.addOdomEdge(kf_from, kf_to, rel_pose, info);
}

void HBAWrapper::addLoopFactor(const LoopConstraint::Ptr& lc) {
    // Level 2: submap loop
    Mat66d info = lc->information;
    level2_graph_.addLoopEdge(lc->submap_i, lc->submap_j, lc->delta_T, info);
    // Level 1：与 iSAM2 一致，使用 KeyFrame::id（keyframe_global_id_*），禁止子图内下标
    if (lc->keyframe_global_id_i >= 0 && lc->keyframe_global_id_j >= 0) {
        level1_graph_.addLoopEdge(lc->keyframe_global_id_i, lc->keyframe_global_id_j, lc->delta_T, info);
    }

    // Trigger optimization
    triggerOptimization(false);
}

void HBAWrapper::addGPSFactor(int kf_id, const Eigen::Vector3d& gps_pos,
                                const Eigen::Matrix3d& gps_cov) {
    level1_graph_.addGPSEdge(kf_id, gps_pos, gps_cov);
}

void HBAWrapper::addSubmapOdomFactor(int sm_from, int sm_to,
                                      const Pose3d& rel_pose, const Mat66d& info) {
    level2_graph_.addOdomEdge(sm_from, sm_to, rel_pose, info);
}

void HBAWrapper::addSubmapNode(int sm_id, const Pose3d& pose, bool fixed) {
    level2_graph_.addNode(sm_id, NodeType::SUBMAP, pose, fixed);
}

void HBAWrapper::addKeyFrameNode(int kf_id, const Pose3d& pose, bool fixed) {
    level1_graph_.addNode(kf_id, NodeType::KEYFRAME, pose, fixed);
}

void HBAWrapper::triggerOptimization(bool full) {
    {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        opt_queue_.push(full);
    }
    queue_cv_.notify_one();
}

void HBAWrapper::registerPoseUpdateCallback(PoseUpdateCallback cb) {
    pose_update_cbs_.push_back(std::move(cb));
}

void HBAWrapper::registerDoneCallback(OptimizationDoneCallback cb) {
    done_cbs_.push_back(std::move(cb));
}

void HBAWrapper::workerLoop() {
    while (running_) {
        bool full;
        {
            std::unique_lock<std::mutex> lk(queue_mutex_);
            queue_cv_.wait(lk, [this] { return !opt_queue_.empty() || !running_; });
            if (!running_ && opt_queue_.empty()) break;
            // Drain queue, use most recent "full" flag
            full = false;
            while (!opt_queue_.empty()) {
                if (opt_queue_.front()) full = true;
                opt_queue_.pop();
            }
        }

        optimizing_ = true;
        utils::Timer t;

        RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[HBAWrapper] Starting %s optimization (L2: %d nodes, %d edges, %d loops)",
                 full ? "FULL" : "incremental",
                 level2_graph_.numNodes(), level2_graph_.numEdges(),
                 level2_graph_.numLoopEdges());

        runLevel2Optimization();

        if (full) {
            runLevel1Optimization();
        }

        broadcastPoseUpdates();

        for (auto& cb : done_cbs_) cb();
        optimizing_ = false;

        RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[HBAWrapper] Optimization done in %.1f ms.", t.elapsedMs());
    }
}

void HBAWrapper::runLevel2Optimization() {
    if (level2_graph_.numNodes() < 2) return;

    // Fix first node
    auto nodes = level2_graph_.allNodes();
    if (!nodes.empty() && !nodes[0].fixed) {
        level2_graph_.addNode(nodes[0].id, nodes[0].type, nodes[0].pose, true);
    }

    auto result = optimizer_.optimize(level2_graph_);
    RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[HBAWrapper] Level2: iter=%d cost=%.6f time=%.1f ms",
             result.iterations, result.final_cost, result.time_ms);

    // Collect results
    std::lock_guard<std::mutex> lk(result_mutex_);
    optimized_submap_poses_ = level2_graph_.getOptimizedPoses();
}

void HBAWrapper::runLevel1Optimization() {
    if (level1_graph_.numNodes() < 2) return;

    auto nodes = level1_graph_.allNodes();
    if (!nodes.empty() && !nodes[0].fixed) {
        level1_graph_.addNode(nodes[0].id, nodes[0].type, nodes[0].pose, true);
    }

    auto result = optimizer_.optimize(level1_graph_);
    RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[HBAWrapper] Level1: iter=%d cost=%.6f time=%.1f ms",
             result.iterations, result.final_cost, result.time_ms);

    std::lock_guard<std::mutex> lk(result_mutex_);
    optimized_kf_poses_ = level1_graph_.getOptimizedPoses();
}

void HBAWrapper::broadcastPoseUpdates() {
    std::map<int, Pose3d> sm_poses;
    {
        std::lock_guard<std::mutex> lk(result_mutex_);
        sm_poses = optimized_submap_poses_;
    }

    for (const auto& [sm_id, pose] : sm_poses) {
        for (auto& cb : pose_update_cbs_) cb(sm_id, pose);
    }

    // Publish optimized path
    nav_msgs::msg::Path path;
    path.header.stamp    = node_->now();
    path.header.frame_id = "map";  // 与后端 odom_path/optimized_path 统一，RViz Fixed Frame=map
    {
        std::lock_guard<std::mutex> lk(result_mutex_);
        for (const auto& [kf_id, pose] : optimized_kf_poses_) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            ps.pose.position.x = pose.translation().x();
            ps.pose.position.y = pose.translation().y();
            ps.pose.position.z = pose.translation().z();
            Eigen::Quaterniond q(pose.rotation());
            ps.pose.orientation.x = q.x();
            ps.pose.orientation.y = q.y();
            ps.pose.orientation.z = q.z();
            ps.pose.orientation.w = q.w();
            path.poses.push_back(ps);
        }
    }
    if (!path.poses.empty()) optimized_path_pub_->publish(path);
}

std::map<int, Pose3d> HBAWrapper::getOptimizedSubmapPoses() const {
    std::lock_guard<std::mutex> lk(result_mutex_);
    return optimized_submap_poses_;
}

std::map<int, Pose3d> HBAWrapper::getOptimizedKeyFramePoses() const {
    std::lock_guard<std::mutex> lk(result_mutex_);
    return optimized_kf_poses_;
}

bool HBAWrapper::isOptimizing() const {
    return optimizing_.load();
}

}  // namespace automap_pro
