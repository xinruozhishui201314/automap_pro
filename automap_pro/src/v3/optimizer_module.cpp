#include "automap_pro/v3/optimizer_module.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"

namespace automap_pro::v3 {

OptimizerModule::OptimizerModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node)
    : ModuleBase("OptimizerModule", event_bus, map_registry), node_(node) {

    optimizer_.registerPoseUpdateCallback([this](const OptimizationResult& res) {
        RCLCPP_DEBUG(node_->get_logger(),
            "[V3][DIAG] step=Optimizer_poseCallback_enter sm=%zu kf=%zu (grep V3 DIAG)",
            res.submap_poses.size(), res.keyframe_poses.size());
        uint64_t version = map_registry_->updatePoses(res.submap_poses, res.keyframe_poses);
        OptimizationResultEvent ev;
        ev.version = version;
        ev.submap_poses = res.submap_poses;
        ev.keyframe_poses = res.keyframe_poses;
        event_bus_->publish(ev);
        RCLCPP_DEBUG(node_->get_logger(),
            "[V3][DIAG] step=Optimizer_poseCallback_done version=%lu (grep V3 DIAG)",
            static_cast<unsigned long>(version));
    });

    onEvent<LoopConstraintEvent>([this](const LoopConstraintEvent& ev) {
        std::lock_guard<std::mutex> lock(task_mutex_);
        OptTaskItem task;
        task.type = OptTaskItem::Type::LOOP_FACTOR;
        task.loop_constraint = ev.constraint;
        task_queue_.push_back(task);
        cv_.notify_one();
    });

    onEvent<GraphTaskEvent>([this](const GraphTaskEvent& ev) {
        std::lock_guard<std::mutex> lock(task_mutex_);
        task_queue_.push_back(ev.task);
        cv_.notify_one();
    });

    onEvent<MapUpdateEvent>([](const MapUpdateEvent& /*ev*/) {
        // 预留：在 run() / MapRegistry 中处理新关键帧等
    });
    RCLCPP_INFO(node_->get_logger(),
                "[PIPELINE][OPT] ctor OK IncrementalOptimizer+Loop/Graph/MapUpdate tasks");
}

void OptimizerModule::run() {
    RCLCPP_INFO(node_->get_logger(), "[V3][OptimizerModule] Started worker thread");
    
    while (running_) {
        OptTaskItem task;
        {
            std::unique_lock<std::mutex> lock(task_mutex_);
            cv_.wait_for(lock, std::chrono::milliseconds(100), 
                [this] { return !running_ || !task_queue_.empty(); });
            
            if (!running_) break;
            if (task_queue_.empty()) continue;
            
            task = task_queue_.front();
            task_queue_.pop_front();
        }

        // 处理优化任务
        processTask(task);
    }
}

void OptimizerModule::processTask(const OptTaskItem& task) {
    const char* type_str = "UNKNOWN";
    switch (task.type) {
        case OptTaskItem::Type::LOOP_FACTOR: type_str = "LOOP_FACTOR"; break;
        case OptTaskItem::Type::FORCE_UPDATE: type_str = "FORCE_UPDATE"; break;
        case OptTaskItem::Type::RESET: type_str = "RESET"; break;
        case OptTaskItem::Type::GPS_FACTOR: type_str = "GPS_FACTOR"; break;
        case OptTaskItem::Type::SUBMAP_NODE: type_str = "SUBMAP_NODE"; break;
        case OptTaskItem::Type::ODOM_FACTOR: type_str = "ODOM_FACTOR"; break;
        case OptTaskItem::Type::KEYFRAME_CREATE: type_str = "KEYFRAME_CREATE"; break;
        case OptTaskItem::Type::GPS_BATCH_KF: type_str = "GPS_BATCH_KF"; break;
        default: break;
    }
    RCLCPP_DEBUG(node_->get_logger(),
        "[V3][DIAG] step=processTask type=%s (grep V3 DIAG)", type_str);
    switch (task.type) {
        case OptTaskItem::Type::LOOP_FACTOR:
            optimizer_.addLoopFactor(task.loop_constraint);
            return;
        case OptTaskItem::Type::FORCE_UPDATE:
            optimizer_.forceUpdate();
            return;
        case OptTaskItem::Type::RESET:
            optimizer_.reset();
            return;
        case OptTaskItem::Type::GPS_FACTOR:
            optimizer_.addGPSFactor(task.to_id, task.gps_pos, task.gps_cov);
            break;
        case OptTaskItem::Type::SUBMAP_NODE:
            optimizer_.addSubMapNode(task.to_id, task.rel_pose, task.fixed);
            break;
        case OptTaskItem::Type::ODOM_FACTOR:
            optimizer_.addOdomFactor(task.from_id, task.to_id, task.rel_pose, task.info_matrix);
            break;
        case OptTaskItem::Type::KEYFRAME_CREATE:
            processKeyframeCreate(task);
            break;
        case OptTaskItem::Type::GPS_BATCH_KF:
            processGPSBatchKF(task);
            return;
        default:
            break;
    }
    optimizer_.forceUpdate();
}

void OptimizerModule::processGPSBatchKF(const OptTaskItem& task) {
    const Eigen::Matrix3d& R = task.R_enu_to_map;
    const Eigen::Vector3d& t = task.t_enu_to_map;
    auto all_kfs = map_registry_->getAllKeyFrames();
    std::vector<IncrementalOptimizer::GPSFactorItemKF> factors;
    for (const auto& kf : all_kfs) {
        if (!kf || !kf->has_valid_gps) continue;
        if (!kf->gps.position_enu.allFinite() || !kf->gps.covariance.allFinite()) continue;
        Eigen::Vector3d pos_map = R * kf->gps.position_enu + t;
        factors.push_back({static_cast<int>(kf->id), pos_map, kf->gps.covariance});
    }
    if (factors.empty()) {
        RCLCPP_DEBUG(node_->get_logger(),
            "[V3][DIAG] step=processGPSBatchKF factors=0 (no KF with valid GPS)");
        return;
    }
    optimizer_.addGPSFactorsForKeyFramesBatch(factors);
    RCLCPP_DEBUG(node_->get_logger(),
        "[V3][DIAG] step=processGPSBatchKF done count=%zu (grep V3 DIAG)", factors.size());
}

void OptimizerModule::processKeyframeCreate(const OptTaskItem& task) {
    if (!task.keyframe) return;
    auto kf = task.keyframe;
    
    // 1. 添加关键帧节点
    bool is_first_kf_of_submap = (kf->id % MAX_KF_PER_SUBMAP == 0);
    optimizer_.addKeyFrameNode(kf->id, kf->T_odom_b, false, is_first_kf_of_submap);
    
    // 2. 添加里程计因子
    if (task.has_prev_kf && task.prev_keyframe) {
        auto prev_kf = task.prev_keyframe;
        Pose3d rel = prev_kf->T_odom_b.inverse() * kf->T_odom_b;
        Mat66d info = computeOdomInfoMatrixForKeyframes(prev_kf, kf, rel);
        optimizer_.addOdomFactorBetweenKeyframes(prev_kf->id, kf->id, rel, info);
    }
    
    // 3. 添加 GPS 因子 (如果已对齐)，pos_map = R_enu_to_map * position_enu + t_enu_to_map
    if (task.gps_aligned && kf->has_valid_gps) {
        Eigen::Vector3d pos_map = task.gps_transform_R * kf->gps.position_enu + task.gps_transform_t;
        optimizer_.addGPSFactorForKeyFrame(kf->id, pos_map, kf->gps.covariance);
    }
}

Mat66d OptimizerModule::computeOdomInfoMatrixForKeyframes(const KeyFrame::Ptr& prev_kf, const KeyFrame::Ptr& curr_kf, const Pose3d& rel) const {
    Mat66d info = Mat66d::Identity();
    const auto& cfg = ConfigManager::instance();
    double trans_noise = cfg.kfOdomTransNoise();
    double rot_noise = cfg.kfOdomRotNoise();
    for (int i = 0; i < 3; ++i) info(i, i) = 1.0 / (trans_noise * trans_noise);
    for (int i = 3; i < 6; ++i) info(i, i) = 1.0 / (rot_noise * rot_noise);
    return info;
}

} // namespace automap_pro::v3
