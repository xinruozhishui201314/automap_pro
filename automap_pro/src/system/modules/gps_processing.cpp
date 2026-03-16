// 模块6: GPS处理与后端同步
// 包含: onGPSAligned, transformAllPosesAfterGPSAlign, addBatchGPSFactors, ensureBackendCompletedAndFlushBeforeHBA

#include "automap_pro/system/automap_system.h"

namespace automap_pro {

// ─────────────────────────────────────────────────────────────────────────────
// GPS 对齐回调（统一投递到队列，由gps_align_thread处理）
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onGPSAligned(const GPSAlignResult& result) {
    if (!result.success) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][GPS_ALIGN] alignment failed: %s", result.message.c_str());
        return;
    }

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN] success! R_enu_to_map:\n%s\nt_enu_to_map: [%.3f, %.3f, %.3f]",
                matrixToString(result.R_enu_to_map).c_str(),
                result.t_enu_to_map.x(), result.t_enu_to_map.y(), result.t_enu_to_map.z());
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=gps_aligned success=1 used_measurements=%zu", result.used_measurements);

    // 投递GPS对齐任务到队列，由gpsAlignWorkerLoop处理
    GPSAlignTaskItem align_task;
    align_task.R_enu_to_map = result.R_enu_to_map;
    align_task.t_enu_to_map = result.t_enu_to_map;

    {
        std::lock_guard<std::mutex> lk(gps_align_mutex_);
        if (gps_align_queue_.size() < kMaxGPSAlignQueueSize) {
            gps_align_queue_.push_back(align_task);
            gps_align_cv_.notify_one();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN] enqueued GPS align task");
        } else {
            RCLCPP_WARN(get_logger(), "[AutoMapSystem][GPS_ALIGN] gps_align_queue full, processing directly");
            // 队列满时直接处理（降级）
            processGPSAlignDirectly(result);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 直接处理GPS对齐（队列满时的降级方案）
// 注意：此函数仅在极端情况下调用（队列满），应该优先等待队列可用
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::processGPSAlignDirectly(const GPSAlignResult& result) {
    // 使用 gps_transform_mutex_ 保护所有 GPS 状态变量的一致性
    {
        std::lock_guard<std::mutex> lk(gps_transform_mutex_);
        gps_aligned_.store(true);
        gps_transform_R_ = result.R_enu_to_map;
        gps_transform_t_ = result.t_enu_to_map;
    }

    // 1. 转换所有位姿到MAP坐标系
    transformAllPosesAfterGPSAlign(result);

    // 2. 等待后端优化完成
    isam2_optimizer_.waitForPendingTasks();

    // 3. 获取所有约束数据用于重建
    auto submap_data = isam2_optimizer_.getAllSubmapData();
    auto odom_factors = isam2_optimizer_.getOdomFactors();
    auto loop_factors = isam2_optimizer_.getLoopFactors();

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN] rebuild data: submaps=%zu odom=%zu loop=%zu",
                submap_data.size(), odom_factors.size(), loop_factors.size());

    // 4. 投递GPS对齐任务到opt_worker线程（等待队列可用）
    {
        std::lock_guard<std::mutex> lk(opt_task_mutex_);
        const int max_waits = 50;  // 最多等待50次（总共5秒）
        const int wait_ms = 100;
        bool enqueued = false;
        for (int wait_count = 0; wait_count < max_waits; ++wait_count) {
            if (opt_task_queue_.size() < kMaxOptTaskQueueSize) {
                OptTaskItem task;
                task.type = OptTaskItem::Type::GPS_ALIGN_COMPLETE;
                task.R_enu_to_map = result.R_enu_to_map;
                task.t_enu_to_map = result.t_enu_to_map;
                task.submap_data = std::move(submap_data);
                task.odom_factors = std::move(odom_factors);
                task.loop_factors = std::move(loop_factors);
                opt_task_queue_.push_back(task);
                opt_task_cv_.notify_one();
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN_DIRECT] enqueued GPS_ALIGN_COMPLETE task after %d waits", wait_count);
                enqueued = true;
                break;
            }
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
            lock.lock();
        }
        if (!enqueued) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][GPS_ALIGN_DIRECT] opt_task_queue full after %d waits, GPS align FAILED", max_waits);
        } else {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN] GPS align task enqueued, waiting for opt_worker to complete...");
            // 等待opt_worker处理完成
            isam2_optimizer_.waitForPendingTasks();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN] all poses transformed and iSAM2 rebuilt with full constraints");
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// GPS对齐后转换所有位姿
// 注意：此函数必须在 waitForPendingTasks() 之后调用，确保没有其他线程正在更新位姿
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::transformAllPosesAfterGPSAlign(const GPSAlignResult& result) {
    // 使用 submap_update_mutex_ 保护，与 onPoseUpdated 一致，避免数据竞争
    std::lock_guard<std::mutex> lk(submap_update_mutex_);
    
    auto all_submaps = submap_manager_.getAllSubmaps();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_TRANSFORM] transforming %zu submaps", all_submaps.size());

    for (const auto& sm : all_submaps) {
        if (!sm) continue;
        sm->pose_w_anchor = Pose3d(result.R_enu_to_map * sm->pose_w_anchor.linear(),
                                    result.R_enu_to_map * sm->pose_w_anchor.translation() + result.t_enu_to_map);
        sm->pose_w_anchor_optimized = sm->pose_w_anchor;

        for (const auto& kf : sm->keyframes) {
            if (!kf) continue;
            kf->T_w_b = Pose3d(result.R_enu_to_map * kf->T_w_b.linear(),
                                  result.R_enu_to_map * kf->T_w_b.translation() + result.t_enu_to_map);
            kf->T_w_b_optimized = kf->T_w_b;
        }
    }

    // 更新当前里程计位姿
    last_odom_pose_ = Pose3d(result.R_enu_to_map * last_odom_pose_.linear(),
                              result.R_enu_to_map * last_odom_pose_.translation() + result.t_enu_to_map);

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_TRANSFORM] done");
}

// ─────────────────────────────────────────────────────────────────────────────
// 批量添加GPS因子
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::addBatchGPSFactors() {
    if (gps_batch_added_.load()) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_BATCH] GPS factors already added, skipping");
        return;
    }

    auto frozen_submaps = submap_manager_.getFrozenSubmaps();
    if (frozen_submaps.empty()) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_BATCH] no frozen submaps, skipping");
        return;
    }

    // 复制 GPS 变换矩阵，确保在循环中使用一致的值
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    {
        std::lock_guard<std::mutex> lk(gps_transform_mutex_);
        R = gps_transform_R_;
        t = gps_transform_t_;
    }

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_BATCH] adding GPS factors to %zu frozen submaps", frozen_submaps.size());

    int added_count = 0;
    for (const auto& sm : frozen_submaps) {
        if (!sm || !sm->has_valid_gps) continue;
        if (sm->gps_enu_pose.translation().norm() < 1e-6) continue;

        Eigen::Vector3d pos_map = R * sm->gps_enu_pose.translation() + t;
        Eigen::Matrix3d cov = sm->gps_cov;

        // 投队列而不是直接调用，确保GTSAM调用只在opt_worker线程执行
        OptTaskItem task;
        task.type = OptTaskItem::Type::GPS_FACTOR;
        task.to_id = sm->id;
        task.gps_pos = pos_map;
        task.gps_cov = cov;
        {
            std::lock_guard<std::mutex> lk(opt_task_mutex_);
            if (opt_task_queue_.size() < kMaxOptTaskQueueSize) {
                opt_task_queue_.push_back(task);
                opt_task_cv_.notify_one();
                added_count++;
            } else {
                RCLCPP_WARN(get_logger(), "[AutoMapSystem][GPS_BATCH] opt_task_queue full, dropping GPS factor for sm_id=%d", sm->id);
            }
        }
    }

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_BATCH] enqueued %d GPS factors", added_count);
    gps_batch_added_.store(true);
}

// ─────────────────────────────────────────────────────────────────────────────
// HBA前确保后端完成
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::ensureBackendCompletedAndFlushBeforeHBA() {
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND] ensuring backend completed before HBA...");

    // 等待 ISAM2 队列处理完成（最多等待 10 秒）
    isam2_optimizer_.waitForPendingTasks();

    // 额外等待一段时间确保所有 pending 的更新完成
    // 增加等待时间以确保 GTSAM 资源完全释放
    constexpr int kExtraWaitMs = 500;  // 从 100ms 增加到 500ms
    std::this_thread::sleep_for(std::chrono::milliseconds(kExtraWaitMs));

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND] backend flush done (waited extra %dms), ready for HBA", kExtraWaitMs);
}

}  // namespace automap_pro
