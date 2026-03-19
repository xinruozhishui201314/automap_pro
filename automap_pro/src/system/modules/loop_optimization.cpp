// 模块5: 回环与优化
// 包含: onLoopDetected, onPoseUpdated, onHBADone

#include "automap_pro/system/automap_system.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"

namespace automap_pro {

// ─────────────────────────────────────────────────────────────────────────────
// 回环检测回调
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onLoopDetected(const LoopConstraint::Ptr& lc) {
    if (!lc) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem][LOOP] onLoopDetected: null constraint ignored");
        return;
    }
    {
        std::lock_guard<std::mutex> lk(loop_constraints_mutex_);
        loop_constraints_.push_back(lc);
        // 使用循环缓冲区策略：当超过 500 时，删除最旧的 100 个（减少频繁删除的开销）
        if (loop_constraints_.size() > 500) {
            loop_constraints_.erase(loop_constraints_.begin(), loop_constraints_.begin() + 100);
        }
    }
    state_ = SystemState::LOOP_CLOSING;
    const double tx = lc->delta_T.translation().x();
    const double ty = lc->delta_T.translation().y();
    const double tz = lc->delta_T.translation().z();
    const double info_norm = lc->information.norm();

    // 同子图同节点回环无效
    if (lc->submap_i == lc->submap_j) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][LOOP] skip same-submap loop sm_i=sm_j=%d", lc->submap_i);
        state_ = SystemState::MAPPING;
        return;
    }
    // 回环质量过滤
    const double trans_norm = std::sqrt(tx*tx + ty*ty + tz*tz);
    const double min_trans_m = ConfigManager::instance().loopMinRelativeTranslationM();
    if (min_trans_m > 0 && trans_norm < min_trans_m) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][LOOP] skip trivial loop sm_i=%d sm_j=%d trans_norm=%.3fm < %.2fm",
            lc->submap_i, lc->submap_j, trans_norm, min_trans_m);
        state_ = SystemState::MAPPING;
        return;
    }

    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][LOOP] detected sm_i=%d sm_j=%d score=%.3f inlier=%.3f rmse=%.3f trans=[%.2f,%.2f,%.2f] info_norm=%.2f",
        lc->submap_i, lc->submap_j, lc->overlap_score, lc->inlier_ratio, lc->rmse, tx, ty, tz, info_norm);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=loop_detected sm_i=%d sm_j=%d score=%.3f rmse=%.3f", lc->submap_i, lc->submap_j, lc->overlap_score, lc->rmse);

    // 始终通过 TaskDispatcher 投递任务
    if (task_dispatcher_) {
        if (task_dispatcher_->submitLoopFactor(lc)) {
            // 🔧 V2 修复：提交回环后立即触发强制优化，确保及时反映回环修正
            task_dispatcher_->submitForceUpdate();
            RCLCPP_INFO(get_logger(),
                "[LOOP_ACCEPTED] onLoopDetected enqueue sm_i=%d sm_j=%d (将入因子图并触发优化，减轻结构重影；grep LOOP_ACCEPTED)",
                lc->submap_i, lc->submap_j);
        } else {
            RCLCPP_WARN(get_logger(), "[AutoMapSystem][LOOP] task_dispatcher failed to submit loop sm_i=%d sm_j=%d (queue full?)", lc->submap_i, lc->submap_j);
        }
    }
    state_ = SystemState::MAPPING;
}

// ─────────────────────────────────────────────────────────────────────────────
// 位姿更新（来自 iSAM2）
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onPoseUpdated(const std::unordered_map<int, Pose3d>& poses) {
    const size_t n = poses.size();
    int first_id = -1;
    double first_x = 0, first_y = 0, first_z = 0;
    
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][POSE_UPDATE_DEBUG] onPoseUpdated called, poses=%zu", n);
    
    // 区分子图级节点和关键帧级节点
    std::vector<std::pair<int, Pose3d>> submap_poses;
    std::vector<std::pair<int, Pose3d>> kf_poses;
    
    for (const auto& [node_id, pose] : poses) {
        if (node_id >= MAX_KF_PER_SUBMAP) {
            kf_poses.emplace_back(node_id, pose);
        } else {
            submap_poses.emplace_back(node_id, pose);
        }
    }
    
    if (n > 0) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][POSE_UPDATE_DEBUG] submap_nodes=%zu, kf_nodes=%zu",
            submap_poses.size(), kf_poses.size());
    }

    // 使用锁保护 submap/kf 的并发更新
    std::lock_guard<std::mutex> lk(submap_update_mutex_);
    
    // 处理子图级节点
    for (const auto& [sm_id, pose] : submap_poses) {
        if (first_id < 0) {
            first_id = sm_id;
            first_x = pose.translation().x();
            first_y = pose.translation().y();
            first_z = pose.translation().z();
        }
        submap_manager_.updateSubmapPose(sm_id, pose);
    }
    
    // 处理关键帧级节点
    if (!kf_poses.empty()) {
        auto active_sm = submap_manager_.getActiveSubmap();
        // 使用 shared_ptr 确保对象在处理期间有效
        if (active_sm && active_sm->state == SubMapState::ACTIVE) {
            int updated_kf_count = 0;
            for (const auto& [node_id, kf_pose] : kf_poses) {
                int kf_sm_id = node_id / MAX_KF_PER_SUBMAP;
                int kf_idx = node_id % MAX_KF_PER_SUBMAP;

                // 再次检查状态，确保子图在处理过程中未被冻结
                if (active_sm->state != SubMapState::ACTIVE) {
                    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][POSE][KF] submap %d state changed during update, skipping",
                                active_sm->id);
                    break;
                }

                if (kf_sm_id == active_sm->id && kf_idx < static_cast<int>(active_sm->keyframes.size())) {
                    auto& kf = active_sm->keyframes[kf_idx];
                    if (kf) {
                        kf->T_w_b_optimized = kf_pose;
                        updated_kf_count++;
                    }
                }
            }
            if (updated_kf_count > 0) {
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][POSE][KF] updated %d keyframes in active submap %d", updated_kf_count, active_sm->id);
            }
        }
    }
    
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][POSE] updated count=%zu first_sm_id=%d pos=[%.2f,%.2f,%.2f]", n, first_id, first_x, first_y, first_z);
    
    // GPS 显示更新
    if (gps_aligned_.load()) {
        try {
            auto all_sm = submap_manager_.getAllSubmaps();
            std::vector<Eigen::Vector3d> gps_positions_map_for_submaps;
            for (const auto& sm : all_sm) {
                if (!sm || !sm->has_valid_gps) continue;
                gps_positions_map_for_submaps.push_back(sm->pose_w_anchor_optimized.translation());
            }
            if (!gps_positions_map_for_submaps.empty()) {
                rviz_publisher_.publishGPSMarkersWithConstraintLines(all_sm, gps_positions_map_for_submaps);
            }
            std::vector<Eigen::Vector3d> gps_positions_map;
            for (const auto& sm : all_sm) {
                if (!sm) continue;
                for (const auto& kf : sm->keyframes) {
                    if (!kf || !kf->has_valid_gps) continue;
                    gps_positions_map.push_back(kf->T_w_b_optimized.translation());
                }
            }
            if (!gps_positions_map.empty()) {
                rviz_publisher_.publishGPSPositionsInMap(gps_positions_map);
            }
        } catch (const std::exception& e) {
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][POSE][GPS] update exception: %s", e.what());
        } catch (...) {}
    }

    // 🔧 V2 修复：删除冗余的 opt_path_ 直接发布，统一通过 rviz_publisher_ 处理。
    // HBA 后不再发布 iSAM2 的轨迹，避免覆盖 HBA 轨迹导致 map(HBA)+path(iSAM2) 重影。
    /*
    if (!odom_path_stopped_after_hba_.load(std::memory_order_acquire)) {
        opt_path_.header.stamp    = now();
        opt_path_.header.frame_id = "map";
        opt_path_.poses.clear();
        std::vector<std::pair<int, Pose3d>> sorted(poses.begin(), poses.end());
        std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
        for (const auto& [sm_id, pose] : sorted) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = opt_path_.header;
            ps.pose.position.x = pose.translation().x();
            ps.pose.position.y = pose.translation().y();
            ps.pose.position.z = pose.translation().z();
            Eigen::Quaterniond q(pose.rotation());
            ps.pose.orientation.w = q.w(); ps.pose.orientation.x = q.x();
            ps.pose.orientation.y = q.y(); ps.pose.orientation.z = q.z();
            opt_path_.poses.push_back(ps);
        }
        if (opt_path_pub_) {
            try { opt_path_pub_->publish(opt_path_); } catch (const std::exception& e) {
                RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][POSE] opt_path publish: %s", e.what());
            } catch (...) {}
            pub_opt_path_count_++;
        }
    }
    */

    try {
        auto all_sm = submap_manager_.getAllSubmaps();
        // 🔧 V2 修复：在 HBA 完成后不再发布 iSAM2 轨迹
        if (!odom_path_stopped_after_hba_.load(std::memory_order_acquire)) {
            rviz_publisher_.publishOptimizedPath(all_sm);
        }
        rviz_publisher_.publishKeyframePoses(collectKeyframesFromSubmaps(all_sm));
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][EXCEPTION] publishOptimizedPath: %s", e.what());
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// HBA 完成处理
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onHBADone(const HBAResult& result) {
    if (!result.success) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem][HBA][EXCEPTION] optimization failed");
        return;
    }
    const size_t pose_count = result.optimized_poses.size();
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][HBA] done success=1 MME=%.4f poses=%zu iter_layer=%d elapsed=%.1fms",
        result.final_mme, pose_count, result.iterations_per_layer, result.elapsed_ms);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=hba_done MME=%.4f poses=%zu elapsed=%.0fms", result.final_mme, pose_count, result.elapsed_ms);

    auto all_sm = submap_manager_.getFrozenSubmaps();
    std::unordered_map<int, Pose3d> isam2_poses_before_hba;
    for (const auto& sm : all_sm) {
        if (!sm) continue;
        isam2_poses_before_hba[sm->id] = isam2_optimizer_.getPose(sm->id);
    }

    // 使用锁保护 HBA 结果更新与点云重建
    {
        std::lock_guard<std::mutex> lk(submap_update_mutex_);
        submap_manager_.updateAllFromHBA(result);
        
        // 🔧 V2 修复：将 HBA 结果同步回 iSAM2 内部状态，解决双轨脱节问题
        std::unordered_map<int, Pose3d> hba_poses;
        for (const auto& sm : all_sm) {
            if (!sm) continue;
            hba_poses[sm->id] = sm->pose_w_anchor_optimized;
        }
        isam2_optimizer_.updateSubMapNodePosesBatch(hba_poses);

        // 🔧 V2 修复：HBA 后必须重建 merged_cloud，否则 fallback 路径的点云将出现严重重影
        submap_manager_.rebuildMergedCloudFromOptimizedPoses();
    }

    // 🔧 V2 修复：HBA 后强制触发一次地图发布，反映优化后的结果
    map_publish_pending_.store(true, std::memory_order_release);
    map_publish_cv_.notify_one();
    
    try {
        rviz_publisher_.publishHBAResult(result);
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][EXCEPTION] publishHBAResult: %s", e.what());
    }

    if (gps_aligned_.load()) {
        try {
            std::vector<Eigen::Vector3d> gps_positions_map_for_submaps;
            for (const auto& sm : all_sm) {
                if (!sm || !sm->has_valid_gps) continue;
                gps_positions_map_for_submaps.push_back(sm->pose_w_anchor_optimized.translation());
            }
            if (!gps_positions_map_for_submaps.empty()) {
                rviz_publisher_.publishGPSMarkersWithConstraintLines(all_sm, gps_positions_map_for_submaps);
            }
        } catch (const std::exception& e) {
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][HBA][GPS] update exception: %s", e.what());
        } catch (...) {}
    }
}

}  // namespace automap_pro
