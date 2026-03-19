// 模块8: 定时任务与工具函数
// 包含: publishStatus, publishGlobalMap, publishDataFlowSummary, collectKeyframesFromSubmaps
//       getOutputDir, saveMapToFiles, stateToString, checkThreadHeartbeats
//       computeOdomInfoMatrix, computeOdomInfoMatrixForKeyframes

#include "automap_pro/system/automap_system.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/utils.h"

#include <filesystem>
#include <utility>

namespace automap_pro {

namespace fs = std::filesystem;

// ─────────────────────────────────────────────────────────────────────────────
// 定时任务：发布状态
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::publishStatus() {
    try {
        const int kf_count = submap_manager_.keyframeCount();
        const int sm_count = submap_manager_.submapCount();
        const int loop_ok  = loop_detector_.loopDetectedCount();
        const int hba_trig = hba_optimizer_.triggerCount();
        const bool hba_busy = hba_optimizer_.isRunning();
        const double last_ts = livo_bridge_.lastOdomTs();

        automap_pro::msg::MappingStatusMsg msg;
        rclcpp::Time ts;
        try { ts = now(); } catch (...) { ts = rclcpp::Time(0); }
        msg.header.stamp  = ts;
        msg.state         = stateToString(state_.load());
        msg.session_id    = current_session_id_;
        msg.keyframe_count = kf_count;
        msg.submap_count  = sm_count;
        msg.gps_aligned   = gps_aligned_.load();
        msg.gps_alignment_score = gps_manager_.alignResult().rmse_m > 0 ?
            static_cast<float>(1.0 / gps_manager_.alignResult().rmse_m) : 0.0f;
        if (status_pub_) {
            status_pub_->publish(msg);
            pub_status_count_++;
        }

        const size_t hba_queue = hba_optimizer_.queueDepth();
        const size_t gps_win = gps_manager_.getGpsWindowSize();
        const size_t frame_q = frame_processor_.frameQueueSize();
        const int force_drops = frame_processor_.backpressureDropCount();
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][BACKEND] state=%s kf=%d sm=%d loop=%d gps_win=%zu frame_queue=%zu force_drops=%d | [HBA] trig=%d busy=%d queue=%zu last_ts=%.1f",
            msg.state.c_str(), kf_count, sm_count, loop_ok, gps_win, frame_q, force_drops, hba_trig, hba_busy ? 1 : 0, hba_queue, last_ts);

        if (++status_publish_count_ >= 5) {
            status_publish_count_ = 0;
            RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][PIPELINE] event=heartbeat state=%s kf=%d sm=%d gps=%d gps_win=%zu",
                msg.state.c_str(), kf_count, sm_count, gps_aligned_.load() ? 1 : 0, gps_win);
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][STATUS] publishStatus exception: %s", e.what());
    } catch (...) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][STATUS] publishStatus unknown exception");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 定时任务：发布全局地图
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::publishGlobalMap() {
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=enter");
    const float voxel_size = map_voxel_size_;
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=using_cached_voxel voxel_size=%.3f", voxel_size);
    size_t pts = 0;
    try {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=buildGlobalMap_enter");
        CloudXYZIPtr global;
        if (ConfigManager::instance().asyncGlobalMapBuild()) {
            global = submap_manager_.buildGlobalMapAsync(voxel_size).get();
        } else {
            global = submap_manager_.buildGlobalMap(voxel_size);
        }
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=buildGlobalMap_done pts=%zu", global ? global->size() : 0u);
        if (!global || global->empty()) {
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][MAP] global map empty, skip publish");
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=map_publish_skipped reason=empty");
            return;
        }
        pts = global->size();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=map_published points=%zu voxel=%.3f", pts, voxel_size);

        const std::string map_frame_id = "map";
        RCLCPP_INFO(get_logger(), "[GLOBAL_MAP_DIAG] publish /automap/global_map frame_id=%s pts=%zu", map_frame_id.c_str(), pts);
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=toROSMsg_enter pts=%zu", pts);
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*global, cloud_msg);
        cloud_msg.header.stamp    = now();
        cloud_msg.header.frame_id = map_frame_id;
        if (global_map_pub_) {
            try { global_map_pub_->publish(cloud_msg); } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "[AutoMapSystem][MAP] global_map publish: %s", e.what());
            } catch (...) {}
            pub_map_count_++;
        }
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=global_map_pub_done");

    // 综合可视化
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=rviz_global_map_enter");
    rviz_publisher_.publishGlobalMap(global);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=rviz_global_map_done");

    auto all_sm = submap_manager_.getAllSubmaps();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap step=getAllSubmaps_done sm_count=%zu", all_sm.size());

    // 🔧 V2 修复：在访问子图与关键帧时持有 submap_update_mutex_，避免与 opt_worker 的位姿更新冲突
    {
        std::lock_guard<std::mutex> lk(submap_update_mutex_);
        rviz_publisher_.publishSubmapBoundaries(all_sm);
        rviz_publisher_.publishSubmapBoundingBoxes(all_sm);
        rviz_publisher_.publishSubmapGraph(all_sm);
        rviz_publisher_.publishOptimizedPath(all_sm);
        rviz_publisher_.publishKeyframePoses(collectKeyframesFromSubmaps(all_sm));
    }

        // GPS 约束可视化
        if (gps_aligned_.load()) {
            try {
                std::vector<Eigen::Vector3d> gps_positions_map;
                for (const auto& sm : all_sm) {
                    if (!sm) continue;
                    for (const auto& kf : sm->keyframes) {
                        if (!kf || !kf->has_valid_gps) continue;
                        // 🔧 修复：若系统已全球化，GPS 坐标本身就在 "map" 系，无需再次变换
                        if (gps_aligned_.load()) {
                            gps_positions_map.push_back(kf->gps.position_enu);
                        } else {
                            gps_positions_map.push_back(gps_manager_.enu_to_map(kf->gps.position_enu));
                        }
                    }
                }
                if (!gps_positions_map.empty()) {
                    rviz_publisher_.publishGPSPositionsInMap(gps_positions_map);
                }
                std::vector<Eigen::Vector3d> kf_gps_path = buildKeyframeGpsPathPointsForRviz(all_sm);
                if (!kf_gps_path.empty()) {
                    rviz_publisher_.publishGpsKeyframePath(kf_gps_path);
                }
            } catch (const std::exception& e) {
                RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][MAP][GPS] exception: %s", e.what());
            } catch (...) {}
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem][MAP] publishGlobalMap exception: %s", e.what());
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 定时任务：发布数据流汇总
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::publishDataFlowSummary() {
    try {
        const int livo_cloud = livo_bridge_.cloudCount();
        const int livo_odom = livo_bridge_.odomCount();
        const int backend_pop = backend_cloud_frames_processed_.load();
        const int backend_proc = backend_frames_actually_processed_.load();
        const int loop_det = loop_detector_.loopDetectedCount();
        const int hba_trig = hba_optimizer_.triggerCount();
        const size_t frame_q = frame_processor_.frameQueueSize();
        const size_t ingress_q = frame_processor_.ingressQueueSize();
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][DATA_FLOW] livo_cloud=%d livo_odom=%d backend_pop=%d backend_proc=%d loop=%d hba=%d frame_q=%zu ingress=%zu",
            livo_cloud, livo_odom, backend_pop, backend_proc, loop_det, hba_trig, frame_q, ingress_q);
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][PUB_STATS] odom_path=%d opt_path=%d global_map=%d status=%d",
            pub_odom_path_count_.load(), pub_opt_path_count_.load(), pub_map_count_.load(), pub_status_count_.load());
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][DATA_FLOW] exception: %s", e.what());
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 辅助函数：从子图收集关键帧
// ─────────────────────────────────────────────────────────────────────────────
std::vector<KeyFrame::Ptr> AutoMapSystem::collectKeyframesFromSubmaps(const std::vector<SubMap::Ptr>& submaps) {
    std::vector<KeyFrame::Ptr> all_kfs;
    for (const auto& sm : submaps) {
        if (!sm) continue;
        for (const auto& kf : sm->keyframes) {
            if (kf) all_kfs.push_back(kf);
        }
    }
    std::sort(all_kfs.begin(), all_kfs.end(),
        [](const KeyFrame::Ptr& a, const KeyFrame::Ptr& b) {
            return a->timestamp < b->timestamp;
        });
    return all_kfs;
}

std::vector<Eigen::Vector3d> AutoMapSystem::buildKeyframeGpsPathPointsForRviz(
    const std::vector<SubMap::Ptr>& submaps) const {
    std::vector<std::pair<double, KeyFrame::Ptr>> sorted;
    for (const auto& sm : submaps) {
        if (!sm) continue;
        for (const auto& kf : sm->keyframes) {
            if (kf) sorted.push_back({kf->timestamp, kf});
        }
    }
    std::sort(sorted.begin(), sorted.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });
    std::vector<Eigen::Vector3d> out;
    out.reserve(sorted.size());
    for (const auto& pr : sorted) {
        const KeyFrame::Ptr& kf = pr.second;
        if (kf->has_valid_gps) {
            // 🔧 修复：若系统已全球化，GPS 坐标本身就在 "map" 系，无需再次变换
            if (gps_aligned_.load()) {
                out.push_back(kf->gps.position_enu);
            } else {
                out.push_back(gps_manager_.enu_to_map(kf->gps.position_enu));
            }
        } else {
            out.push_back(kf->T_w_b_optimized.translation());
        }
    }
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// 工具函数：获取输出目录
// ─────────────────────────────────────────────────────────────────────────────
std::string AutoMapSystem::getOutputDir() const {
    if (!output_dir_override_.empty()) {
        return output_dir_override_;
    }
    return ConfigManager::instance().outputDir();
}

// ─────────────────────────────────────────────────────────────────────────────
// 工具函数：保存地图到文件
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::saveMapToFiles(const std::string& output_dir) {
    auto all_submaps = submap_manager_.getAllSubmaps();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Saving %zu submaps to %s", all_submaps.size(), output_dir.c_str());

    fs::create_directories(output_dir);
    for (const auto& sm : submap_manager_.getAllSubmaps()) {
        if (sm) submap_manager_.archiveSubmap(sm, output_dir);
    }

    // 保存优化后的轨迹到 CSV
    std::string traj_path = output_dir + "/trajectory_odom.csv";
    writeTrajectoryOdomAfterMapping(output_dir);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Trajectory saved to %s", traj_path.c_str());

    // GPS vs HBA/优化位姿精度报告（CSV + 摘要 + 偏差曲线图，与 output_dir 同目录，如 run_YYYYMMDD_HHMMSS）
    writeMappingAccuracyGpsVsHba(output_dir);

    // 保存全局合并点云地图 (PCD)
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Building global map for saving...");
    float voxel_size = map_voxel_size_;
    CloudXYZIPtr global_map;
    if (ConfigManager::instance().asyncGlobalMapBuild()) {
        global_map = submap_manager_.buildGlobalMapAsync(voxel_size).get();
    } else {
        global_map = submap_manager_.buildGlobalMap(voxel_size);
    }

    if (global_map && !global_map->empty()) {
        std::string map_path = output_dir + "/global_map.pcd";
        try {
            if (pcl::io::savePCDFileBinary(map_path, *global_map) == 0) {
                RCLCPP_INFO(get_logger(), "[AutoMapSystem] Global map saved to %s (points: %zu)", map_path.c_str(), global_map->size());
            } else {
                RCLCPP_ERROR(get_logger(), "[AutoMapSystem] Failed to save global map to %s", map_path.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem] Exception saving global map: %s", e.what());
        }
    } else {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem] Global map is empty, skip saving.");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 工具函数：状态转字符串
// ─────────────────────────────────────────────────────────────────────────────
std::string AutoMapSystem::stateToString(SystemState s) const {
    switch (s) {
        case SystemState::IDLE: return "IDLE";
        case SystemState::MAPPING: return "MAPPING";
        case SystemState::LOOP_CLOSING: return "LOOP_CLOSING";
        case SystemState::OPTIMIZING: return "OPTIMIZING";
        case SystemState::SAVING: return "SAVING";
        default: return "UNKNOWN";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 线程心跳监控
// ─────────────────────────────────────────────────────────────────────────────
int64_t AutoMapSystem::nowMs() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

const char* AutoMapSystem::backendStepName(int id) const {
    switch (id) {
        case BACKEND_STEP_IDLE: return "idle";
        case BACKEND_STEP_TRY_CREATE_KF_ENTER: return "tryCreateKeyFrame_enter";
        case BACKEND_STEP_ADD_KEYFRAME_ENTER: return "submap_manager_addKeyFrame_enter";
        case BACKEND_STEP_INTRA_LOOP_ENTER: return "detectIntraSubmapLoop_enter";
        case BACKEND_STEP_GPS_FACTOR_ENTER: return "addGPSFactor_enter";
        case BACKEND_STEP_FORCE_UPDATE: return "forceUpdate_commitAndUpdate";
        default: return "unknown";
    }
}

void AutoMapSystem::checkThreadHeartbeats() {
    const int64_t now = nowMs();
    const auto check_thread = [&](const char* name, std::atomic<int64_t>& ts_ms, int64_t warn_thresh_ms, int64_t err_thresh_ms) {
        const int64_t age = now - ts_ms.load(std::memory_order_acquire);
        if (age > err_thresh_ms) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][HEARTBEAT] %s heartbeat stale >%lds, possible deadlock!", name, age / 1000);
        } else if (age > warn_thresh_ms) {
            RCLCPP_WARN(get_logger(), "[AutoMapSystem][HEARTBEAT] %s heartbeat stale >%lds", name, age / 1000);
        }
    };
    check_thread("Feeder", feeder_heartbeat_ts_ms_, kHeartbeatWarnThresholdMs, kHeartbeatErrorThresholdMs);
    check_thread("Backend", backend_heartbeat_ts_ms_, kHeartbeatWarnThresholdMs, kHeartbeatErrorThresholdMs);
    check_thread("MapPublish", map_pub_heartbeat_ts_ms_, kHeartbeatWarnThresholdMs, kHeartbeatErrorThresholdMs);
    // LoopOpt 已删除，回环约束在 onLoopDetected 中直接处理
    check_thread("GPSWorker", gps_worker_heartbeat_ts_ms_, kHeartbeatWarnThresholdMs, kHeartbeatErrorThresholdMs);
    check_thread("OptWorker", opt_worker_heartbeat_ts_ms_, kHeartbeatWarnThresholdMs, kHeartbeatErrorThresholdMs);
    check_thread("LoopTrigger", loop_trigger_heartbeat_ts_ms_, kHeartbeatWarnThresholdMs, kHeartbeatErrorThresholdMs);
    check_thread("IntraLoopWorker", intra_loop_worker_heartbeat_ts_ms_, kHeartbeatWarnThresholdMs, kHeartbeatErrorThresholdMs);
    check_thread("GPSAlign", gps_align_heartbeat_ts_ms_, kHeartbeatWarnThresholdMs, kHeartbeatErrorThresholdMs);
    // Viz 已删除
    check_thread("StatusPub", status_pub_heartbeat_ts_ms_, kHeartbeatWarnThresholdMs, kHeartbeatErrorThresholdMs);
}

// ─────────────────────────────────────────────────────────────────────────────
// 里程计信息矩阵计算
// ─────────────────────────────────────────────────────────────────────────────
Mat66d AutoMapSystem::computeOdomInfoMatrix(
    const SubMap::Ptr& prev,
    const SubMap::Ptr& curr,
    const Pose3d& rel) const
{
    Mat66d info = Mat66d::Identity();
    if (!prev || !curr) return info * 1e-4;

    double trans_noise = 0.01;
    double rot_noise = 0.05;

    for (int i = 0; i < 3; ++i) info(i, i) = 1.0 / (trans_noise * trans_noise);
    for (int i = 3; i < 6; ++i) info(i, i) = 1.0 / (rot_noise * rot_noise);

    return info;
}

Mat66d AutoMapSystem::computeOdomInfoMatrixForKeyframes(
    const KeyFrame::Ptr& prev_kf,
    const KeyFrame::Ptr& curr_kf,
    const Pose3d& rel) const
{
    Mat66d info = Mat66d::Identity();
    if (!prev_kf || !curr_kf) return info * 1e-4;

    double trans_noise = 0.01;
    double rot_noise = 0.05;

    if (prev_kf->livo_info.esikf_cov_norm < 0.01 && curr_kf->livo_info.esikf_cov_norm < 0.01) {
        trans_noise = 0.005;
        rot_noise = 0.02;
    } else if (prev_kf->livo_info.esikf_cov_norm > 0.1 || curr_kf->livo_info.esikf_cov_norm > 0.1) {
        trans_noise = 0.05;
        rot_noise = 0.2;
    }

    for (int i = 0; i < 3; ++i) info(i, i) = 1.0 / (trans_noise * trans_noise);
    for (int i = 3; i < 6; ++i) info(i, i) = 1.0 / (rot_noise * rot_noise);

    return info;
}

}  // namespace automap_pro
