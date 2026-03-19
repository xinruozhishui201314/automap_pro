// 模块6: GPS处理与后端同步
// 包含: onGPSAligned, transformAllPosesAfterGPSAlign, addBatchGPSFactors, ensureBackendCompletedAndFlushBeforeHBA

#include "automap_pro/system/automap_system.h"

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <sstream>
#include <iomanip>

namespace automap_pro {

static std::string matrixToString(const Eigen::Matrix3d& m) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    for (int i = 0; i < 3; ++i) {
        ss << "[" << m(i, 0) << ", " << m(i, 1) << ", " << m(i, 2) << "]" << (i == 2 ? "" : "\n");
    }
    return ss.str();
}

// ─────────────────────────────────────────────────────────────────────────────
// GPS 对齐回调（统一投递到队列，由gps_align_thread处理）
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onGPSAligned(const GPSAlignResult& result) {
    if (gps_aligned_.load()) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN] system already aligned, ignoring new result");
        return;
    }
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
        // 保存 Map -> ENU 的变换，用于后续新关键帧的全球化
        gps_transform_R_ = result.R_enu_to_map.transpose();
        gps_transform_t_ = -gps_transform_R_ * result.t_enu_to_map;
    }

    // 1. 转换所有位姿到MAP坐标系
    transformAllPosesAfterGPSAlign(result);

    // 2. 等待后端优化完成
    isam2_optimizer_.waitForPendingTasks();

    // 3. 获取所有约束数据用于重建
    auto submap_data = isam2_optimizer_.getAllSubmapData();
    auto odom_factors = isam2_optimizer_.getOdomFactors();
    auto loop_factors = isam2_optimizer_.getLoopFactors();
    auto keyframe_data = isam2_optimizer_.getKeyFrameData();
    auto kf_odom_factors = isam2_optimizer_.getKFOdomFactors();
    auto kf_loop_factors = isam2_optimizer_.getKFLoopFactors();

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN] rebuild data: submaps=%zu odom=%zu loop=%zu kf=%zu kf_odom=%zu kf_loop=%zu",
                submap_data.size(), odom_factors.size(), loop_factors.size(),
                keyframe_data.size(), kf_odom_factors.size(), kf_loop_factors.size());

    // 4. 投递GPS对齐任务到opt_worker线程（等待队列可用）
    {
        std::unique_lock<std::mutex> lk(opt_task_mutex_);
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
                task.keyframe_data = std::move(keyframe_data);
                task.kf_odom_factors = std::move(kf_odom_factors);
                task.kf_loop_factors = std::move(kf_loop_factors);
                opt_task_queue_.push_back(task);
                opt_task_cv_.notify_one();
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN_DIRECT] enqueued GPS_ALIGN_COMPLETE task after %d waits", wait_count);
                enqueued = true;
                break;
            }
            lk.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
            lk.lock();
        }
        if (!enqueued) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][GPS_ALIGN_DIRECT] opt_task_queue full after %d waits, GPS align FAILED", max_waits);
        } else {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN] GPS align task enqueued, waiting for opt_worker to complete...");
            // 🔧 V2 修复：等待 opt_worker 处理完当前所有任务，确保重建拿到一致的图状态
            ensureBackendCompletedAndFlushBeforeHBA();
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

    // 构建变换矩阵：使用 Map -> ENU 的逆变换，将地图整体平移/旋转到 GPS 坐标系
    // result.R_enu_to_map 是 ENU 到 Map 的旋转，其转置就是 Map 到 ENU
    Eigen::Matrix3d R_map_to_enu = result.R_enu_to_map.transpose();
    Eigen::Vector3d t_map_to_enu = -R_map_to_enu * result.t_enu_to_map;
    
    Eigen::Affine3d T_map_to_enu = Eigen::Affine3d::Identity();
    T_map_to_enu.linear() = R_map_to_enu;
    T_map_to_enu.translation() = t_map_to_enu;
    Eigen::Affine3f T_map_to_enu_f = T_map_to_enu.cast<float>();

    int transformed_kf_count = 0;
    int transformed_cloud_count = 0;

    for (const auto& sm : all_submaps) {
        if (!sm) continue;
        {
            Pose3d T = Pose3d::Identity();
            T.linear() = R_map_to_enu * sm->pose_w_anchor.linear();
            T.translation() = R_map_to_enu * sm->pose_w_anchor.translation() + t_map_to_enu;
            sm->pose_w_anchor = T;
        }
        sm->pose_w_anchor_optimized = sm->pose_w_anchor;

        for (const auto& kf : sm->keyframes) {
            if (!kf) continue;
            // 1. 变换位姿
            {
                Pose3d T = Pose3d::Identity();
                T.linear() = R_map_to_enu * kf->T_w_b.linear();
                T.translation() = R_map_to_enu * kf->T_w_b.translation() + t_map_to_enu;
                kf->T_w_b = T;
            }
            kf->T_w_b_optimized = kf->T_w_b;
            transformed_kf_count++;

            // 2. 变换点云坐标系 - [修复] cloud_body 始终保持在 body 系，不在此变换
            //    点云合并到子图或显示时会动态使用 T_w_b 变换到世界系。
            /*
            if (kf->cloud_body && !kf->cloud_body->empty()) {
                pcl::transformPointCloud(*kf->cloud_body, *kf->cloud_body, T_map_to_enu_f);
                transformed_cloud_count++;
            }
            */
        }
    }

    // 更新当前里程计位姿
    {
        Pose3d T = Pose3d::Identity();
        T.linear() = R_map_to_enu * last_odom_pose_.linear();
        T.translation() = R_map_to_enu * last_odom_pose_.translation() + t_map_to_enu;
        last_odom_pose_ = T;
    }

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

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_BATCH] adding GPS factors to %zu frozen submaps (V2 direct)", frozen_submaps.size());

    int added_count = 0;
    for (const auto& sm : frozen_submaps) {
        if (!sm || !sm->has_valid_gps) continue;
        if (sm->gps_enu_pose.translation().norm() < 1e-6) continue;

        // 由于地图已整体变换到 ENU 坐标系，GPS 因子直接使用原 ENU 坐标即可，无需再次变换
        Eigen::Vector3d pos_map = sm->gps_enu_pose.translation();
        Eigen::Matrix3d cov = sm->gps_cov;

        // 🔧 V2 修复：由于 addBatchGPSFactors 已在 opt_worker 线程中执行，
        // 直接操作 isam2_optimizer_ 而非通过 TaskDispatcher 再次投递，避免效率低下和死锁风险。
        isam2_optimizer_.addGPSFactor(sm->id, pos_map, cov);
        added_count++;
    }

    if (added_count > 0) {
        // 批量添加后立即触发优化
        isam2_optimizer_.forceUpdate();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_BATCH] added %d GPS factors and triggered forceUpdate", added_count);
    }
    
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

    // 投递任务到 opt_worker 线程执行，避免直接操作 isam2_optimizer_
    if (task_dispatcher_) {
        if (task_dispatcher_->submitActiveSubmapGPSBind(R, t)) {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN_KF] enqueued ACTIVE_SUBMAP_GPS_BIND task");
        } else {
            RCLCPP_WARN(get_logger(), "[AutoMapSystem][GPS_ALIGN_KF] task_dispatcher failed to submit ACTIVE_SUBMAP_GPS_BIND (queue full?)");
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// HBA前确保后端完成
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::ensureBackendCompletedAndFlushBeforeHBA() {
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND] ensuring backend completed before HBA (V2 OptWorker)...");

    // 1. 等待 opt_task_queue_ 清空且当前无任务正在执行
    auto t0 = std::chrono::steady_clock::now();
    constexpr int kMaxWaitSec = 10;
    while (true) {
        bool empty = false;
        {
            std::lock_guard<std::mutex> lk(opt_task_mutex_);
            empty = opt_task_queue_.empty();
        }
        bool in_progress = opt_task_in_progress_.load(std::memory_order_acquire);
        
        if (empty && !in_progress) break;
        
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - t0).count() >= kMaxWaitSec) {
            RCLCPP_WARN(get_logger(), "[AutoMapSystem][BACKEND] ensureBackendCompleted timeout (%ds)! queue_empty=%d in_progress=%d",
                        kMaxWaitSec, empty ? 1 : 0, in_progress ? 1 : 0);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // 2. 额外等待一段时间确保所有 pending 的更新完成（释放 GTSAM 资源）
    constexpr int kExtraWaitMs = 500;
    std::this_thread::sleep_for(std::chrono::milliseconds(kExtraWaitMs));

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND] backend flush done (waited extra %dms), ready for HBA", kExtraWaitMs);
}

}  // namespace automap_pro
