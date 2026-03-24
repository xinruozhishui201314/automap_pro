#include "automap_pro/v3/map_registry.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/metrics.h"
#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#include <limits>

namespace fs = std::filesystem;

namespace automap_pro::v3 {

void MapRegistry::addKeyFrame(KeyFrame::Ptr kf) {
    if (!kf) return;
    if (kf->pose_frame == PoseFrame::UNKNOWN) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_pro"),
            "[V3][CONTRACT] Reject addKeyFrame: kf_id=%lu pose_frame=UNKNOWN",
            static_cast<unsigned long>(kf->id));
        METRICS_INCREMENT(metrics::UNKNOWN_FRAME_RESULT_TOTAL);
        return;
    }
    if (!kf->T_odom_b.matrix().allFinite() || !kf->T_map_b_optimized.matrix().allFinite()) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_pro"),
            "[V3][CONTRACT] Reject addKeyFrame: kf_id=%lu non-finite pose detected",
            static_cast<unsigned long>(kf->id));
        METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
        return;
    }
    RCLCPP_DEBUG(rclcpp::get_logger("automap_pro"),
        "[V3][CONTRACT] addKeyFrame kf_id=%lu frame=%d odom_norm=%.3f map_norm=%.3f has_gps=%d",
        static_cast<unsigned long>(kf->id), static_cast<int>(kf->pose_frame),
        kf->T_odom_b.translation().norm(), kf->T_map_b_optimized.translation().norm(),
        kf->has_valid_gps ? 1 : 0);
    {
        std::lock_guard<std::mutex> lock(kf_mutex_);
        keyframes_[kf->id] = kf;
        keyframes_count_ = keyframes_.size();
    }
    
    // 发送 KeyFrame 添加事件
    MapUpdateEvent event;
    event.version = ++next_version_;
    current_version_.store(event.version);
    event.type = MapUpdateEvent::ChangeType::KEYFRAME_ADDED;
    event.affected_ids = {kf->id};
    event_bus_->publish(event);
}

KeyFrame::Ptr MapRegistry::getKeyFrame(int id) const {
    std::lock_guard<std::mutex> lock(kf_mutex_);
    auto it = keyframes_.find(id);
    return (it != keyframes_.end()) ? it->second : nullptr;
}

KeyFrame::Ptr MapRegistry::getKeyFrameByTimestamp(double timestamp, double tolerance_s) const {
    std::lock_guard<std::mutex> lock(kf_mutex_);
    KeyFrame::Ptr best = nullptr;
    double best_dt = std::numeric_limits<double>::infinity();
    for (const auto& [id, kf] : keyframes_) {
        (void)id;
        if (!kf) continue;
        const double dt = std::abs(kf->timestamp - timestamp);
        if (dt <= tolerance_s && dt < best_dt) {
            best = kf;
            best_dt = dt;
        }
    }
    return best;
}

KeyFrame::Ptr MapRegistry::getLatestKeyFrameByTimestamp() const {
    std::lock_guard<std::mutex> lock(kf_mutex_);
    KeyFrame::Ptr best;
    double best_ts = -std::numeric_limits<double>::infinity();
    for (const auto& [id, kf] : keyframes_) {
        (void)id;
        if (!kf) continue;
        if (kf->timestamp > best_ts) {
            best_ts = kf->timestamp;
            best = kf;
        }
    }
    return best;
}

std::vector<KeyFrame::Ptr> MapRegistry::getAllKeyFrames() const {
    std::lock_guard<std::mutex> lock(kf_mutex_);
    std::vector<KeyFrame::Ptr> all;
    all.reserve(keyframes_.size());
    for (const auto& [id, kf] : keyframes_) {
        all.push_back(kf);
    }
    return all;
}

void MapRegistry::addSubMap(SubMap::Ptr sm) {
    if (!sm) return;
    if (sm->pose_frame == PoseFrame::UNKNOWN) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_pro"),
            "[V3][CONTRACT] Reject addSubMap: sm_id=%d pose_frame=UNKNOWN", sm->id);
        METRICS_INCREMENT(metrics::UNKNOWN_FRAME_RESULT_TOTAL);
        return;
    }
    if (!sm->pose_odom_anchor.matrix().allFinite() || !sm->pose_map_anchor_optimized.matrix().allFinite()) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_pro"),
            "[V3][CONTRACT] Reject addSubMap: sm_id=%d non-finite anchor pose detected", sm->id);
        METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
        return;
    }
    RCLCPP_DEBUG(rclcpp::get_logger("automap_pro"),
        "[V3][CONTRACT] addSubMap sm_id=%d frame=%d odom_norm=%.3f map_norm=%.3f state=%d",
        sm->id, static_cast<int>(sm->pose_frame),
        sm->pose_odom_anchor.translation().norm(),
        sm->pose_map_anchor_optimized.translation().norm(),
        static_cast<int>(sm->state));
    {
        std::lock_guard<std::mutex> lock(sm_mutex_);
        submaps_[sm->id] = sm;
        submaps_count_ = submaps_.size();
    }
    
    // 发送 SubMap 添加事件
    MapUpdateEvent event;
    event.version = ++next_version_;
    current_version_.store(event.version);
    event.type = MapUpdateEvent::ChangeType::SUBMAP_ADDED;
    event.affected_ids = {sm->id};
    event_bus_->publish(event);
}

SubMap::Ptr MapRegistry::getSubMap(int id) const {
    std::lock_guard<std::mutex> lock(sm_mutex_);
    auto it = submaps_.find(id);
    return (it != submaps_.end()) ? it->second : nullptr;
}

std::vector<SubMap::Ptr> MapRegistry::getAllSubMaps() const {
    std::lock_guard<std::mutex> lock(sm_mutex_);
    std::vector<SubMap::Ptr> all;
    all.reserve(submaps_.size());
    for (const auto& [id, sm] : submaps_) {
        all.push_back(sm);
    }
    return all;
}

uint64_t MapRegistry::updatePoses(const std::unordered_map<int, Pose3d>& sm_updates,
                                 const std::unordered_map<uint64_t, Pose3d>& kf_updates,
                                 PoseFrame pose_frame,
                                 const std::string& source_module,
                                 uint64_t source_alignment_epoch,
                                 uint32_t transform_applied_flags,
                                 uint64_t batch_hash) {
    if (pose_frame == PoseFrame::UNKNOWN) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_pro"),
            "[V3][CONTRACT] Reject updatePoses: success-path pose_frame=UNKNOWN is forbidden");
        METRICS_INCREMENT(metrics::UNKNOWN_FRAME_RESULT_TOTAL);
        return current_version_.load();
    }
    if (pose_frame != PoseFrame::MAP) {
        if (gps_aligned_.load()) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_pro"),
                "[V3][CONTRACT] Reject updatePoses: non-MAP frame is forbidden after GPS alignment");
            METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
            return current_version_.load();
        }
        // Expected until GPS alignment: optimizer reports ODOM semantics; registry stores as MAP-equivalent.
        RCLCPP_DEBUG(rclcpp::get_logger("automap_pro"),
            "[V3][CONTRACT] Accepting non-MAP frame in pre-alignment stage as MAP-equivalent");
    }
    for (const auto& [id, pose] : sm_updates) {
        if (!pose.matrix().allFinite()) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_pro"),
                "[V3][CONTRACT] Reject updatePoses: submap pose %d has non-finite value", id);
            METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
            return current_version_.load();
        }
    }
    for (const auto& [id, pose] : kf_updates) {
        if (!pose.matrix().allFinite()) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_pro"),
                "[V3][CONTRACT] Reject updatePoses: keyframe pose %lu has non-finite value", id);
            METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
            return current_version_.load();
        }
    }

    // 🏛️ [产品化加固] Reasonableness Check (数值安全网)
    // 防止优化器因离群值导致的数值爆炸损毁地图
    const double max_trans = ConfigManager::instance().mappingMaxReasonableTranslationM();
    for (const auto& [id, pose] : sm_updates) {
        if (pose.translation().norm() > max_trans) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_pro"),
                "[V3][CRASH_GUARD] Reject updatePoses: submap %d translation too large (%.1fm > %.1fm) from %s",
                id, pose.translation().norm(), max_trans, source_module.c_str());
            return current_version_.load();
        }
    }
    for (const auto& [id, pose] : kf_updates) {
        if (pose.translation().norm() > max_trans) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_pro"),
                "[V3][CRASH_GUARD] Reject updatePoses: keyframe %lu translation too large (%.1fm > %.1fm) from %s",
                id, pose.translation().norm(), max_trans, source_module.c_str());
            return current_version_.load();
        }
    }

    std::vector<int> affected_kf_ids;
    std::vector<int> affected_sm_ids;

    uint64_t version = ++next_version_;
    auto new_snapshot = std::make_shared<PoseSnapshot>();
    new_snapshot->version = version;

    uint64_t expected_epoch = 0;
    {
        std::lock_guard<std::mutex> kf_lock(kf_mutex_);
        std::lock_guard<std::mutex> sm_lock(sm_mutex_);
        std::lock_guard<std::mutex> gps_lock(gps_state_mutex_);
        expected_epoch = alignment_epoch_.load(std::memory_order_relaxed);
        if (source_alignment_epoch != expected_epoch) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_pro"),
                "[V3][CONTRACT] Reject updatePoses: alignment_epoch mismatch source=%lu expected=%lu src=%s",
                static_cast<unsigned long>(source_alignment_epoch),
                static_cast<unsigned long>(expected_epoch),
                source_module.c_str());
            METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
            return current_version_.load();
        }
        
        // 1. 更新内部 KeyFrame/SubMap 对象 (SSoT)
        for (const auto& [id, pose] : sm_updates) {
            auto it = submaps_.find(id);
            if (it != submaps_.end()) {
                it->second->pose_map_anchor_optimized = pose;
                it->second->pose_frame = pose_frame; // 🏛️ [架构加固] 尊重优化器报告的坐标系语义
                affected_sm_ids.push_back(id);
            }
        }
        
        for (const auto& [id, pose] : kf_updates) {
            auto it = keyframes_.find(static_cast<int>(id));
            if (it != keyframes_.end()) {
                it->second->T_map_b_optimized = pose;
                it->second->pose_frame = pose_frame; // 🏛️ [架构加固] 尊重优化器报告的坐标系语义
                affected_kf_ids.push_back(static_cast<int>(id));
            }
        }

        // 2. 构建新快照 (Immutable Snapshot)
        // 继承未更新的位姿，确保快照完整
        PoseSnapshot::Ptr old_snapshot;
        {
            std::lock_guard<std::mutex> snap_lk(snapshot_mutex_);
            old_snapshot = current_snapshot_;
        }
        
        new_snapshot->submap_poses = old_snapshot->submap_poses;
        new_snapshot->keyframe_poses = old_snapshot->keyframe_poses;
        
        for (const auto& [id, pose] : sm_updates) new_snapshot->submap_poses[id] = pose;
        for (const auto& [id, pose] : kf_updates) new_snapshot->keyframe_poses[id] = pose;

        // 继承 GPS 状态
        new_snapshot->gps_aligned = gps_aligned_.load();
        new_snapshot->R_enu_to_map = R_enu_to_map_;
        new_snapshot->t_enu_to_map = t_enu_to_map_;
        new_snapshot->gps_rmse = gps_rmse_;
        new_snapshot->alignment_epoch = expected_epoch;

        // 3. 原子切换快照
        {
            std::lock_guard<std::mutex> snap_lk(snapshot_mutex_);
            current_snapshot_ = new_snapshot;
        }
    }

    current_version_.store(version);

    RCLCPP_INFO(rclcpp::get_logger("automap_pro"),
        "[V3][POSE_DIAG] MapRegistry version %lu update: sm_cnt=%zu kf_cnt=%zu | GPS_aligned=%d T_map_odom=[%.2f,%.2f,%.2f]",
        static_cast<unsigned long>(version), affected_sm_ids.size(), affected_kf_ids.size(),
        gps_aligned_.load() ? 1 : 0, t_enu_to_map_.x(), t_enu_to_map_.y(), t_enu_to_map_.z());

    // 发布位姿图优化结果事件（用于同步各模块内部缓存，如 VisualizationModule）
    OptimizationResultEvent result_ev;
    result_ev.version = version;
    result_ev.event_id = version;
    result_ev.alignment_epoch = expected_epoch;
    result_ev.submap_poses = sm_updates;
    result_ev.keyframe_poses = kf_updates;
    result_ev.pose_frame = pose_frame; // 🏛️ [架构加固] 透传优化结果的坐标系语义
    result_ev.source_module = source_module;
    result_ev.transform_applied_flags = transform_applied_flags;
    result_ev.batch_hash = batch_hash;
    event_bus_->publish(result_ev);

    // 发布地图变更事件
    MapUpdateEvent event;
    event.version = version;
    event.type = MapUpdateEvent::ChangeType::POSES_OPTIMIZED;
    event.affected_ids = affected_sm_ids;
    event.affected_ids.insert(event.affected_ids.end(), affected_kf_ids.begin(), affected_kf_ids.end());
    event_bus_->publish(event);

    return version;
}

void MapRegistry::addConstraint(const LoopConstraint::Ptr& lc) {
    if (!lc) return;
    {
        std::lock_guard<std::mutex> lock(constraint_mutex_);
        constraints_.push_back(lc);
    }
    
    // 发送约束添加事件
    MapUpdateEvent event;
    event.version = ++next_version_;
    current_version_.store(event.version);
    event.type = MapUpdateEvent::ChangeType::CONSTRAINT_ADDED;
    event_bus_->publish(event);
}

std::vector<LoopConstraint::Ptr> MapRegistry::getConstraints() const {
    std::lock_guard<std::mutex> lock(constraint_mutex_);
    return constraints_;
}

void MapRegistry::loadSession(const std::string& session_dir, uint64_t session_id) {
    if (session_dir.empty()) return;
    
    try {
        for (auto& entry : fs::directory_iterator(session_dir)) {
            if (!entry.is_directory()) continue;
            std::string name = entry.path().filename().string();
            if (name.find("submap_") != 0) continue;

            int sm_id = 0;
            try {
                sm_id = std::stoi(name.substr(7));
            } catch (...) { continue; }
            
            SubMap::Ptr sm;
            SubMapManager archive_loader;
            if (archive_loader.loadArchivedSubmap(session_dir, sm_id, sm)) {
                if (sm) {
                    sm->session_id = session_id;
                    addSubMap(sm);
                    
                    // 加载关联的关键帧
                    for (auto& kf : sm->keyframes) {
                        addKeyFrame(kf);
                    }
                }
            }
        }
    } catch (...) {}
}

} // namespace automap_pro::v3
