#include "automap_pro/v3/optimizer_module.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/metrics.h"
#include <algorithm>
#include <vector>

namespace {
uint64_t computeBatchHash(const std::unordered_map<int, automap_pro::Pose3d>& sm_poses,
                          const std::unordered_map<uint64_t, automap_pro::Pose3d>& kf_poses) {
    uint64_t h = 1469598103934665603ull;
    auto mix = [&h](uint64_t v) {
        h ^= v;
        h *= 1099511628211ull;
    };
    mix(static_cast<uint64_t>(sm_poses.size()));
    mix(static_cast<uint64_t>(kf_poses.size()));
    std::vector<int> sm_ids;
    sm_ids.reserve(sm_poses.size());
    for (const auto& [id, _] : sm_poses) sm_ids.push_back(id);
    std::sort(sm_ids.begin(), sm_ids.end());
    for (int id : sm_ids) {
        const auto& pose = sm_poses.at(id);
        mix(static_cast<uint64_t>(id));
        mix(static_cast<uint64_t>(pose.translation().x() * 1000.0));
        mix(static_cast<uint64_t>(pose.translation().y() * 1000.0));
        mix(static_cast<uint64_t>(pose.translation().z() * 1000.0));
    }
    std::vector<uint64_t> kf_ids;
    kf_ids.reserve(kf_poses.size());
    for (const auto& [id, _] : kf_poses) kf_ids.push_back(id);
    std::sort(kf_ids.begin(), kf_ids.end());
    for (uint64_t id : kf_ids) {
        const auto& pose = kf_poses.at(id);
        mix(id);
        mix(static_cast<uint64_t>(pose.translation().x() * 1000.0));
        mix(static_cast<uint64_t>(pose.translation().y() * 1000.0));
        mix(static_cast<uint64_t>(pose.translation().z() * 1000.0));
    }
    return h;
}
} // namespace

namespace automap_pro::v3 {

OptimizerModule::OptimizerModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node)
    : ModuleBase("OptimizerModule", event_bus, map_registry), node_(node) {

    optimizer_.registerPoseUpdateCallback([this](const OptimizationResult& res) {
        if (!running_.load()) {
            RCLCPP_WARN(node_->get_logger(), "[V3][CONTRACT] Reject optimizer callback: module is stopping/stopped");
            return;
        }
        if (res.pose_frame == PoseFrame::UNKNOWN) {
            RCLCPP_ERROR(node_->get_logger(), "[V3][CONTRACT] Reject optimization result: success-path pose_frame=UNKNOWN");
            METRICS_INCREMENT(metrics::UNKNOWN_FRAME_RESULT_TOTAL);
            return;
        }
        if (!res.isFinite() || !res.isReasonable()) {
            RCLCPP_ERROR(node_->get_logger(), "[V3][CONTRACT] Reject optimization result: non-finite or unreasonable pose");
            METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
            return;
        }
        if (ConfigManager::instance().contractFramePolicy() == "strict_map_only" && res.pose_frame != PoseFrame::MAP) {
            RCLCPP_ERROR(node_->get_logger(), "[V3][CONTRACT] strict_map_only rejects non-MAP optimizer result");
            METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
            return;
        }
        RCLCPP_DEBUG(node_->get_logger(),
            "[V3][DIAG] step=Optimizer_poseCallback_enter sm=%zu kf=%zu (grep V3 DIAG)",
            res.submap_poses.size(), res.keyframe_poses.size());
        uint64_t batch_hash = computeBatchHash(res.submap_poses, res.keyframe_poses);
        uint64_t version = map_registry_->updatePoses(
            res.submap_poses, res.keyframe_poses, res.pose_frame, "OptimizerModule",
            static_cast<uint32_t>(OptimizationTransformFlags::NONE), batch_hash);
        RCLCPP_DEBUG(node_->get_logger(),
            "[V3][DIAG] step=Optimizer_poseCallback_done version=%lu (grep V3 DIAG)",
            static_cast<unsigned long>(version));
    });

    onEvent<LoopConstraintEvent>([this](const LoopConstraintEvent& ev) {
        if (!running_.load()) return;
        std::lock_guard<std::mutex> lock(task_mutex_);
        OptTaskItem task;
        task.type = OptTaskItem::Type::LOOP_FACTOR;
        task.loop_constraint = ev.constraint;
        task_queue_.push_back(task);
        cv_.notify_one();
    });

    onEvent<GraphTaskEvent>([this](const GraphTaskEvent& ev) {
        if (!running_.load()) return;
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

bool OptimizerModule::isIdle() const {
    std::lock_guard<std::mutex> lock(task_mutex_);
    return task_queue_.empty();
}

void OptimizerModule::run() {
    RCLCPP_INFO(node_->get_logger(), "[V3][OptimizerModule] Started worker thread");
    
    while (running_) {
        updateHeartbeat();
        std::deque<OptTaskItem> tasks;
        {
            std::unique_lock<std::mutex> lock(task_mutex_);
            cv_.wait_for(lock, std::chrono::milliseconds(100), 
                [this] { return !running_ || !task_queue_.empty(); });
            
            if (!running_) break;
            if (task_queue_.empty()) continue;
            
            // 🏛️ [P0 性能优化] 任务合并 (Task Coalescing)
            // 一次性取出当前队列中所有任务进行批量处理，显著减少 iSAM2::update() 的调用频率
            // 对于高频关键帧流，这能极大降低 GTSAM 重线性化带来的 CPU 抖动
            tasks = std::move(task_queue_);
            task_queue_.clear();
        }

        // 处理一批优化任务
        bool needs_update = false;
        bool has_reset = false;

        for (const auto& task : tasks) {
            if (task.type == OptTaskItem::Type::RESET) {
                optimizer_.reset();
                has_reset = true;
                needs_update = false; // reset 后不需要立即 update
                continue;
            }
            
            if (task.type == OptTaskItem::Type::FORCE_UPDATE) {
                needs_update = true;
            }

            // 如果已经 reset，后续非关键任务在同一批次中可能不再有意义，但为了逻辑简单继续处理
            processTaskInternal(task);
            
            // 保证版本推进：任何非 RESET 批次都至少触发一次优化提交
            needs_update = true;
        }

        if (needs_update && running_) {
            optimizer_.forceUpdate();
        }
    }
}

void OptimizerModule::processTaskInternal(const OptTaskItem& task) {
    switch (task.type) {
        case OptTaskItem::Type::LOOP_FACTOR:
            optimizer_.addLoopFactor(task.loop_constraint);
            break;
        case OptTaskItem::Type::FORCE_UPDATE:
            // 仅标记需要 update，循环末尾会统一执行
            break;
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
            break;
        default:
            break;
    }
}

void OptimizerModule::processTask(const OptTaskItem& task) {
    // 兼容旧接口：立即处理并更新
    if (task.type == OptTaskItem::Type::RESET) {
        optimizer_.reset();
        return;
    }
    processTaskInternal(task);
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
    RCLCPP_INFO(node_->get_logger(),
        "[V3][CONTRACT] processKeyframeCreate kf_id=%lu pose_frame=%d use_pose=T_odom_b gps_aligned=%d has_gps=%d",
        static_cast<unsigned long>(kf->id), static_cast<int>(kf->pose_frame),
        task.gps_aligned ? 1 : 0, kf->has_valid_gps ? 1 : 0);
    
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
