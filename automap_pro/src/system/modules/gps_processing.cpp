// 模块6: GPS处理与后端同步
// 包含: onGPSAligned, transformAllPosesAfterGPSAlign, addBatchGPSFactors, ensureBackendCompletedAndFlushBeforeHBA

#include "automap_pro/system/automap_system.h"

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

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
// GPS对齐后转换所有位姿和点云
// 注意：此函数必须在 waitForPendingTasks() 之后调用，确保没有其他线程正在更新位姿
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::transformAllPosesAfterGPSAlign(const GPSAlignResult& result) {
    // 使用 submap_update_mutex_ 保护，与 onPoseUpdated 一致，避免数据竞争
    std::lock_guard<std::mutex> lk(submap_update_mutex_);
    
    RCLCPP_INFO(get_logger(),
        "[POSE_JUMP_CAUSE] GPS 对齐：即将对所有子图/关键帧应用 ENU→map 变换 → RViz 轨迹与 GPS 显示将整体跳变（预期行为）；查跳变: grep POSE_JUMP");
    auto all_submaps = submap_manager_.getAllSubmaps();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_TRANSFORM] transforming %zu submaps", all_submaps.size());

    // 构建变换矩阵
    Eigen::Affine3d T_enu_to_map = Eigen::Affine3d::Identity();
    T_enu_to_map.linear() = result.R_enu_to_map;
    T_enu_to_map.translation() = result.t_enu_to_map;
    Eigen::Affine3f T_enu_to_map_f = T_enu_to_map.cast<float>();

    int transformed_kf_count = 0;
    int transformed_cloud_count = 0;

    for (const auto& sm : all_submaps) {
        if (!sm) continue;
        sm->pose_w_anchor = Pose3d(result.R_enu_to_map * sm->pose_w_anchor.linear(),
                                    result.R_enu_to_map * sm->pose_w_anchor.translation() + result.t_enu_to_map);
        sm->pose_w_anchor_optimized = sm->pose_w_anchor;

        for (const auto& kf : sm->keyframes) {
            if (!kf) continue;
            // 1. 变换位姿
            kf->T_w_b = Pose3d(result.R_enu_to_map * kf->T_w_b.linear(),
                                  result.R_enu_to_map * kf->T_w_b.translation() + result.t_enu_to_map);
            kf->T_w_b_optimized = kf->T_w_b;
            transformed_kf_count++;

            // 2. 变换点云坐标系（cloud_body 是 body frame 下的点云，不需要变换）
            //    但如果 cloud_body 已经被配准到世界坐标系，需要变换
            //    这里我们变换的是已配准的点云 (world frame)
            // 注意：cloud_body 实际存储的是配准后的世界坐标点云
            if (kf->cloud_body && !kf->cloud_body->empty()) {
                // 变换点云到新的坐标系
                // 原始点云在旧的世界坐标系，现在需要变换到新的世界坐标系
                // new_points = R_enu_to_map * old_points + t_enu_to_map
                pcl::transformPointCloud(*kf->cloud_body, *kf->cloud_body, T_enu_to_map_f);
                transformed_cloud_count++;
            }
        }
    }

    // 更新当前里程计位姿
    last_odom_pose_ = Pose3d(result.R_enu_to_map * last_odom_pose_.linear(),
                              result.R_enu_to_map * last_odom_pose_.translation() + result.t_enu_to_map);

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_TRANSFORM] done: %d keyframes, %d clouds transformed",
                transformed_kf_count, transformed_cloud_count);

    // ═══════════════════════════════════════════════════════════════════════════════
    // 修复: GPS对齐成功后，立即为当前活跃子图的所有keyframes添加GPS因子
    // 这样可以避免GTSAM single-node问题导致后续子图未被优化的问题
    // ═══════════════════════════════════════════════════════════════════════════════
    addGPSFactorsToActiveSubmapOnAlign(result);
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

// ═══════════════════════════════════════════════════════════════════════════════
// GPS对齐成功后，为当前活跃子图的所有keyframes添加GPS因子
// 这样可以避免GTSAM single-node问题导致后续子图未被优化的问题
// ═══════════════════════════════════════════════════════════════════════════════
void AutoMapSystem::addGPSFactorsToActiveSubmapOnAlign(const GPSAlignResult& result) {
    // 获取GPS变换矩阵
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    {
        std::lock_guard<std::mutex> lk(gps_transform_mutex_);
        R = gps_transform_R_;
        t = gps_transform_t_;
    }

    // 获取当前活跃子图
    auto active_sm = submap_manager_.getActiveSubmap();
    if (!active_sm) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN_KF] no active submap, skipping");
        return;
    }

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN_KF] active submap sm_id=%d has %zu keyframes",
                active_sm->id, active_sm->keyframes.size());

    int added_count = 0;
    for (const auto& kf : active_sm->keyframes) {
        if (!kf) continue;

        // 查询该keyframe对应的GPS位置
        auto gps_opt = gps_manager_.queryByTimestamp(kf->timestamp);
        if (!gps_opt.has_value()) {
            continue;
        }
        const auto& gps = gps_opt.value();

        // 计算在map坐标系下的位置
        Eigen::Vector3d pos_enu = gps.position_enu;
        Eigen::Vector3d pos_map = R * pos_enu + t;

        // 计算协方差（基于HDOP）
        Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
        double hdop_scale = gps.hdop / 10.0;  // 归一化HDOP
        double sigma_h = 0.5 * hdop_scale;    // 水平 sigma
        double sigma_v = 1.0 * hdop_scale;    // 垂直 sigma
        cov(0, 0) = sigma_h * sigma_h;
        cov(1, 1) = sigma_h * sigma_h;
        cov(2, 2) = sigma_v * sigma_v;

        // 1. 添加keyframe节点到ISAM2
        bool kf_fixed = (kf->id == 0);  // 第一个keyframe设为fixed
        isam2_optimizer_.addKeyFrameNode(static_cast<int>(kf->id), kf->T_w_b, kf_fixed);

        // 2. 添加keyframe级别的GPS因子
        isam2_optimizer_.addGPSFactorForKeyFrame(static_cast<int>(kf->id), pos_map, cov);

        added_count++;

        RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][GPS_ALIGN_KF] added GPS factor for kf_id=%d pos=[%.2f,%.2f,%.2f]",
                    kf->id, pos_map.x(), pos_map.y(), pos_map.z());
    }

    // 触发forceUpdate以立即优化
    if (added_count > 0) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN_KF] added %d GPS factors to active submap, triggering optimization", added_count);
        isam2_optimizer_.forceUpdate();
    } else {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN_KF] no GPS factors added (no matching GPS data)");
    }
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
