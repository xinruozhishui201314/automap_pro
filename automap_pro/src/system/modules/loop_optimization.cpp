// 模块5: 回环与优化
// 包含: onLoopDetected, onPoseUpdated, onHBADone

#include "automap_pro/system/automap_system.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include <unordered_set>
#include <chrono>

namespace automap_pro {

static double poseRotationDeltaDeg(const Pose3d& before, const Pose3d& after) {
    const Eigen::Matrix3d dR = before.linear().transpose() * after.linear();
    const Eigen::AngleAxisd aa(dR);
    return std::abs(aa.angle() * 180.0 / M_PI);
}

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

    // [GHOSTING_DIAG] 详细记录回环属性，便于分析重影
    const Eigen::Vector3d t_lc = lc->delta_T.translation();
    const double trans_norm = t_lc.norm();
    const double min_trans_m = ConfigManager::instance().loopMinRelativeTranslationM();
    
    const Eigen::Matrix3d R_lc = lc->delta_T.linear();
    const double rot_deg = Eigen::AngleAxisd(R_lc).angle() * 180.0 / M_PI;
    
    if (lc->submap_i == lc->submap_j) {
        // 🔧 [核心改变] 不再跳过子图内回环，因为它们对于消除子图内部漂移（重影主因之一）至关重要
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][LOOP][INTRA] detected intra-submap loop sm_id=%d kf_%d->%d trans=%.2fm rot=%.2fdeg",
            lc->submap_i, lc->keyframe_i, lc->keyframe_j, trans_norm, rot_deg);
    }

    // 诊断: 记录回环约束对应关键帧的当前全局位姿，便于分析“约束坐标系不一致”问题
    if (lc->keyframe_i >= 0 && lc->keyframe_j >= 0) {
        auto sm_i_ptr = submap_manager_.getSubmap(lc->submap_i);
        auto sm_j_ptr = submap_manager_.getSubmap(lc->submap_j);
        if (sm_i_ptr && sm_j_ptr &&
            lc->keyframe_i < static_cast<int>(sm_i_ptr->keyframes.size()) &&
            lc->keyframe_j < static_cast<int>(sm_j_ptr->keyframes.size())) {
            auto kf_i_ptr = sm_i_ptr->keyframes[lc->keyframe_i];
            auto kf_j_ptr = sm_j_ptr->keyframes[lc->keyframe_j];
            if (kf_i_ptr && kf_j_ptr) {
                const Eigen::Vector3d pi = kf_i_ptr->T_w_b_optimized.translation();
                const Eigen::Vector3d pj = kf_j_ptr->T_w_b_optimized.translation();
                const Eigen::Vector3d p_delta = pj - pi;
                RCLCPP_INFO(get_logger(),
                    "[AutoMapSystem][LOOP][POSE_DIAG] sm_i=%d kf_i=%d kf_id_i=%lu pos_i=[%.3f,%.3f,%.3f] | "
                    "sm_j=%d kf_j=%d kf_id_j=%lu pos_j=[%.3f,%.3f,%.3f] | current_delta=[%.3f,%.3f,%.3f] norm=%.3fm | constraint_delta=[%.3f,%.3f,%.3f] norm=%.3fm",
                    lc->submap_i, lc->keyframe_i, static_cast<unsigned long>(kf_i_ptr->id), pi.x(), pi.y(), pi.z(),
                    lc->submap_j, lc->keyframe_j, static_cast<unsigned long>(kf_j_ptr->id), pj.x(), pj.y(), pj.z(),
                    p_delta.x(), p_delta.y(), p_delta.z(), p_delta.norm(),
                    t_lc.x(), t_lc.y(), t_lc.z(), t_lc.norm());
            }
        }
    }

    if (min_trans_m > 0 && trans_norm < min_trans_m) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][LOOP] skip trivial loop sm_i=%d sm_j=%d trans_norm=%.3fm < %.2fm",
            lc->submap_i, lc->submap_j, trans_norm, min_trans_m);
        state_ = SystemState::MAPPING;
        return;
    }

    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][LOOP] detected sm_i=%d sm_j=%d score=%.3f inlier=%.3f rmse=%.3f trans=[%.2f,%.2f,%.2f] info_norm=%.2f",
        lc->submap_i, lc->submap_j, lc->overlap_score, lc->inlier_ratio, lc->rmse, t_lc.x(), t_lc.y(), t_lc.z(), info_norm);
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
void AutoMapSystem::onPoseUpdated(const OptimizationResult& res) {
    PoseUpdateTransaction tx;
    tx.version_id = pose_tx_version_counter_.fetch_add(1, std::memory_order_acq_rel) + 1;
    tx.source = PoseUpdateSource::LOOP_OPTIMIZATION;
    tx.frame = gps_aligned_.load(std::memory_order_acquire) ? PoseFrameSemantics::ENU_GLOBAL
                                                             : PoseFrameSemantics::MAP_LOCAL;
    if (!res.keyframe_poses.empty()) {
        tx.mode = PoseWriteMode::KEYFRAME_ABSOLUTE_SET;
    } else if (!res.submap_poses.empty()) {
        tx.mode = PoseWriteMode::SUBMAP_DELTA_PROPAGATE;
    } else {
        tx.mode = PoseWriteMode::KEYFRAME_ABSOLUTE_SET;
    }
    tx.result = res;

    if (pose_update_barrier_active_.load(std::memory_order_acquire)) {
        std::lock_guard<std::mutex> lk(deferred_pose_updates_mutex_);
        deferred_pose_updates_.push_back(tx);
        const size_t deferred_size = deferred_pose_updates_.size();
        RCLCPP_WARN(get_logger(),
            "[AutoMapSystem][POSE_TX][DEFER] tx_version=%lu source=%s mode=%s frame=%s reason=barrier_active deferred_size=%zu submap_count=%zu kf_count=%zu",
            static_cast<unsigned long>(tx.version_id),
            toString(tx.source), toString(tx.mode), toString(tx.frame),
            deferred_size, res.submap_poses.size(), res.keyframe_poses.size());
        if (deferred_size > 20) {
            RCLCPP_WARN(get_logger(),
                "[AutoMapSystem][POSE_TX][DEFER_BACKLOG] deferred_size=%zu barrier_active=1 last_applied_tx=%lu",
                deferred_size, static_cast<unsigned long>(last_applied_pose_tx_version_.load(std::memory_order_acquire)));
        }
        return;
    }
    applyPoseTransaction(tx);
}

void AutoMapSystem::applyPoseTransaction(const PoseUpdateTransaction& tx) {
    const auto apply_t0 = std::chrono::steady_clock::now();
    if (ConfigManager::instance().backendVerboseTrace()) {
        RCLCPP_INFO(get_logger(), "[BACKEND_TRACE] applyPoseTransaction ENTER tx_version=%lu source=%s mode=%s",
            static_cast<unsigned long>(tx.version_id), toString(tx.source), toString(tx.mode));
    }
    if (tx.source == PoseUpdateSource::HBA_WRITEBACK) {
        if (!tx.hba_result) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][POSE_TX][HBA] tx_version=%lu missing hba_result",
                         static_cast<unsigned long>(tx.version_id));
            return;
        }
        const HBAResult& result = *tx.hba_result;
        if (!result.success) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][HBA][EXCEPTION] optimization failed");
            return;
        }
        const size_t pose_count = result.optimized_poses.size();
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][POSE_TX][APPLY_BEGIN] tx_version=%lu source=%s mode=%s frame=%s hba_poses=%zu",
            static_cast<unsigned long>(tx.version_id), toString(tx.source), toString(tx.mode), toString(tx.frame), pose_count);
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
                    // 🔧 修复：若系统已对齐且全球化，GPS 坐标本身就在全局 "map" 系，无需再次变换
                    Eigen::Vector3d pos_map;
                    if (gps_aligned_.load()) {
                        pos_map = sm->gps_center;
                    } else {
                        pos_map = gps_manager_.enu_to_map(sm->gps_center);
                    }
                    gps_positions_map_for_submaps.push_back(pos_map);
                }
                if (!gps_positions_map_for_submaps.empty()) {
                    rviz_publisher_.publishGPSMarkersWithConstraintLines(all_sm, gps_positions_map_for_submaps);
                }
                std::vector<Eigen::Vector3d> kf_gps_path = buildKeyframeGpsPathPointsForRviz(all_sm);
                if (!kf_gps_path.empty()) {
                    rviz_publisher_.publishGpsKeyframePath(kf_gps_path);
                }
            } catch (const std::exception& e) {
                RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][HBA][GPS] update exception: %s", e.what());
            } catch (...) {}
        }
        last_applied_pose_tx_version_.store(tx.version_id, std::memory_order_release);
        const auto apply_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - apply_t0).count();
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][POSE_TX][APPLY_END] tx_version=%lu source=%s mode=%s frame=%s hba_poses=%zu elapsed_ms=%ld",
            static_cast<unsigned long>(tx.version_id),
            toString(tx.source), toString(tx.mode), toString(tx.frame), pose_count, apply_ms);
        return;
    }

    const OptimizationResult& res = tx.result;
    if (ConfigManager::instance().backendVerboseTrace()) {
        RCLCPP_INFO(get_logger(), "[BACKEND_TRACE] applyPoseTransaction LOOP_OPT submap_poses=%zu kf_poses=%zu",
            res.submap_poses.size(), res.keyframe_poses.size());
    }
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][POSE_TX][APPLY_BEGIN] tx_version=%lu source=%s mode=%s frame=%s submap_count=%zu kf_count=%zu",
        static_cast<unsigned long>(tx.version_id),
        toString(tx.source), toString(tx.mode), toString(tx.frame),
        res.submap_poses.size(), res.keyframe_poses.size());

    const size_t n_sm = res.submap_poses.size();
    const size_t n_kf = res.keyframe_poses.size();
    
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][POSE_UPDATE_DEBUG] onPoseUpdated called, submap_poses=%zu, kf_poses=%zu", n_sm, n_kf);
    
    // 使用锁保护 submap/kf 的并发更新
    std::lock_guard<std::mutex> lk(submap_update_mutex_);
    
    std::unordered_set<int> sm_with_kf_updates;
    for (const auto& [node_id, _] : res.keyframe_poses) {
        sm_with_kf_updates.insert(node_id / MAX_KF_PER_SUBMAP);
    }

    // 1. 处理子图级节点 (Symbol 's')
    int first_id = -1;
    double first_x = 0, first_y = 0, first_z = 0;
    double max_sm_delta_m = 0.0;
    double max_sm_rot_deg = 0.0;
    int max_sm_id = -1;
    int sampled_sm_logs = 0;
    for (const auto& [sm_id, pose] : res.submap_poses) {
        if (first_id < 0) {
            first_id = sm_id;
            first_x = pose.translation().x();
            first_y = pose.translation().y();
            first_z = pose.translation().z();
        }
        Pose3d before = Pose3d::Identity();
        bool has_before = false;
        if (auto sm_before = submap_manager_.getSubmap(sm_id)) {
            before = sm_before->pose_w_anchor;
            has_before = true;
        }
        if (sm_with_kf_updates.find(sm_id) != sm_with_kf_updates.end()) {
            RCLCPP_WARN(get_logger(),
                "[AutoMapSystem][POSE_TX][INVARIANT] skip submap delta update sm_id=%d reason=same_tx_has_kf_absolute_set",
                sm_id);
            continue;
        }
        // 🔧 [核心修复] updateSubmapPose 内部现在是增量更新，消除了与 HBA 的冲突
        submap_manager_.updateSubmapPose(sm_id, pose);
        if (has_before) {
            const double dtrans = (pose.translation() - before.translation()).norm();
            const double drot_deg = poseRotationDeltaDeg(before, pose);
            if (dtrans > max_sm_delta_m) {
                max_sm_delta_m = dtrans;
                max_sm_rot_deg = drot_deg;
                max_sm_id = sm_id;
            }
            if (sampled_sm_logs < 5 || dtrans > 2.0 || drot_deg > 10.0) {
                RCLCPP_INFO(get_logger(),
                    "[AutoMapSystem][POSE_UPDATE][SM] sm_id=%d before=[%.3f,%.3f,%.3f] after=[%.3f,%.3f,%.3f] dtrans=%.3fm drot=%.3fdeg",
                    sm_id,
                    before.translation().x(), before.translation().y(), before.translation().z(),
                    pose.translation().x(), pose.translation().y(), pose.translation().z(),
                    dtrans, drot_deg);
                sampled_sm_logs++;
            }
        }
    }
    
    // 2. 处理关键帧级节点 (Symbol 'x')
    if (!res.keyframe_poses.empty()) {
        // [修复] 关键帧更新不再局限于 ACTIVE 子图，因为回环可能影响任何已有的子图
        // 直接更新对应关键帧的已优化位姿
        int updated_kf_count = 0;
        int sampled_kf_logs = 0;
        double max_kf_delta_m = 0.0;
        double max_kf_rot_deg = 0.0;
        int max_kf_node = -1;
        for (const auto& [node_id, kf_pose] : res.keyframe_poses) {
            int kf_sm_id = node_id / MAX_KF_PER_SUBMAP;
            int kf_idx = node_id % MAX_KF_PER_SUBMAP;
            auto sm = submap_manager_.getSubmap(kf_sm_id);
            if (sm && kf_idx < static_cast<int>(sm->keyframes.size())) {
                auto& kf = sm->keyframes[kf_idx];
                if (kf) {
                    const Pose3d before = kf->T_w_b_optimized;
                    // 🔧 [修复] 关键帧位姿是直接赋值，因为 iSAM2 返回的是全局绝对位姿
                    kf->T_w_b_optimized = kf_pose;
                    const double dtrans = (kf_pose.translation() - before.translation()).norm();
                    const double drot_deg = poseRotationDeltaDeg(before, kf_pose);
                    if (dtrans > max_kf_delta_m) {
                        max_kf_delta_m = dtrans;
                        max_kf_rot_deg = drot_deg;
                        max_kf_node = node_id;
                    }
                    if (sampled_kf_logs < 8 || dtrans > 2.0 || drot_deg > 10.0) {
                        RCLCPP_INFO(get_logger(),
                            "[AutoMapSystem][POSE_UPDATE][KF] node=%d sm_id=%d idx=%d kf_id=%lu before=[%.3f,%.3f,%.3f] after=[%.3f,%.3f,%.3f] dtrans=%.3fm drot=%.3fdeg",
                            node_id, kf_sm_id, kf_idx, static_cast<unsigned long>(kf->id),
                            before.translation().x(), before.translation().y(), before.translation().z(),
                            kf_pose.translation().x(), kf_pose.translation().y(), kf_pose.translation().z(),
                            dtrans, drot_deg);
                        sampled_kf_logs++;
                    }
                    updated_kf_count++;
                }
            }
        }
        if (updated_kf_count > 0) {
            RCLCPP_INFO(get_logger(),
                        "[AutoMapSystem][POSE][KF] updated %d keyframes from iSAM2 result | max_delta node=%d dtrans=%.3fm drot=%.3fdeg",
                        updated_kf_count, max_kf_node, max_kf_delta_m, max_kf_rot_deg);
        }
    }
    
    // 🔧 [核心修复] 处理活跃子图：如果活跃子图的关键帧被优化了，必须强制更新子图位姿和点云
    // 解决子图内回环后，由于活跃子图未冻结（iSAM2 无 's' 节点）导致其点云锁死在里程计系产生的重影
    auto active_sm = submap_manager_.getActiveSubmap();
    if (active_sm && !active_sm->keyframes.empty()) {
        const bool active_sm_already_kf_updated = (sm_with_kf_updates.find(active_sm->id) != sm_with_kf_updates.end());
        if (active_sm_already_kf_updated) {
            // 关键修复：本轮已经直接写回该子图关键帧位姿，不能再对子图执行一次增量变换，
            // 否则会把同一批关键帧“二次变换”导致轨迹/GPS严重错位。
            RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][POSE_UPDATE][ACTIVE_SM] skip forced submap update sm_id=%d reason=kf_pose_already_updated",
                active_sm->id);
        } else {
            auto anchor_kf = active_sm->keyframes.front();
            if (anchor_kf) {
                // 仅当该子图本轮没有关键帧级更新时，才用锚点关键帧同步子图位姿
                // 以触发 merged_cloud 跟随更新，避免局部显示滞后。
                submap_manager_.updateSubmapPose(active_sm->id, anchor_kf->T_w_b_optimized);
                RCLCPP_INFO(get_logger(),
                    "[AutoMapSystem][POSE_UPDATE][ACTIVE_SM] forced submap update sm_id=%d using anchor kf_id=%lu",
                    active_sm->id, static_cast<unsigned long>(anchor_kf->id));
            }
        }
    }
    
    if (n_sm > 0) {
        RCLCPP_INFO(get_logger(),
                    "[AutoMapSystem][POSE] updated count=%zu first_sm_id=%d pos=[%.2f,%.2f,%.2f] max_sm_delta sm_id=%d dtrans=%.3fm drot=%.3fdeg",
                    n_sm, first_id, first_x, first_y, first_z, max_sm_id, max_sm_delta_m, max_sm_rot_deg);
    }
    
    // GPS 显示更新
    if (gps_aligned_.load()) {
        try {
            auto all_sm = submap_manager_.getAllSubmaps();
            std::vector<Eigen::Vector3d> gps_positions_map_for_submaps;
            for (const auto& sm : all_sm) {
                if (!sm || !sm->has_valid_gps) continue;
                // 🔧 修复：若系统已对齐且全球化，GPS 坐标本身就在全局 "map" 系，无需再次变换
                Eigen::Vector3d pos_map;
                if (gps_aligned_.load()) {
                    pos_map = sm->gps_center;
                } else {
                    pos_map = gps_manager_.enu_to_map(sm->gps_center);
                }
                gps_positions_map_for_submaps.push_back(pos_map);
            }
            if (!gps_positions_map_for_submaps.empty()) {
                rviz_publisher_.publishGPSMarkersWithConstraintLines(all_sm, gps_positions_map_for_submaps);
            }
            std::vector<Eigen::Vector3d> gps_positions_map;
            for (const auto& sm : all_sm) {
                if (!sm) continue;
                for (const auto& kf : sm->keyframes) {
            if (!kf->has_valid_gps) continue;
            // 🔧 修复：若系统已对齐且全球化，GPS 坐标本身就在全局 "map" 系，无需再次变换
            Eigen::Vector3d pos_map;
            if (gps_aligned_.load()) {
                pos_map = kf->gps.position_enu;
            } else {
                pos_map = gps_manager_.enu_to_map(kf->gps.position_enu);
            }
            gps_positions_map.push_back(pos_map);
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

    if (ConfigManager::instance().backendVerboseTrace()) {
        RCLCPP_INFO(get_logger(),
            "[GHOSTING_TRACE] applyPoseTransaction LOOP_OPT done tx=%lu kf_updated=%zu sm_updated=%zu (merged_cloud 未 rebuild；若 buildGlobalMap 走 fallback 则有重影风险；grep GHOSTING_TRACE)",
            static_cast<unsigned long>(tx.version_id), n_kf, n_sm);
    }

    last_applied_pose_tx_version_.store(tx.version_id, std::memory_order_release);
    const auto apply_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - apply_t0).count();
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][POSE_TX][APPLY_END] tx_version=%lu source=%s mode=%s frame=%s applied_submaps=%zu applied_kf=%zu elapsed_ms=%ld",
        static_cast<unsigned long>(tx.version_id),
        toString(tx.source), toString(tx.mode), toString(tx.frame), n_sm, n_kf, apply_ms);
}

void AutoMapSystem::flushDeferredPoseUpdates() {
    std::deque<PoseUpdateTransaction> pending;
    {
        std::lock_guard<std::mutex> lk(deferred_pose_updates_mutex_);
        if (deferred_pose_updates_.empty()) return;
        pending.swap(deferred_pose_updates_);
    }
    const auto first_tx = pending.front().version_id;
    const auto last_tx = pending.back().version_id;
    RCLCPP_INFO(get_logger(),
        "[AutoMapSystem][POSE_TX][FLUSH] flushing deferred pose updates count=%zu tx_range=[%lu,%lu]",
        pending.size(),
        static_cast<unsigned long>(first_tx), static_cast<unsigned long>(last_tx));
    for (const auto& tx : pending) {
        applyPoseTransaction(tx);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// HBA 完成处理
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onHBADone(const HBAResult& result) {
    PoseUpdateTransaction tx;
    tx.version_id = pose_tx_version_counter_.fetch_add(1, std::memory_order_acq_rel) + 1;
    tx.source = PoseUpdateSource::HBA_WRITEBACK;
    tx.frame = gps_aligned_.load(std::memory_order_acquire) ? PoseFrameSemantics::ENU_GLOBAL
                                                             : PoseFrameSemantics::MAP_LOCAL;
    tx.mode = PoseWriteMode::KEYFRAME_ABSOLUTE_SET;
    tx.hba_result = std::make_shared<HBAResult>(result);
    if (pose_update_barrier_active_.load(std::memory_order_acquire)) {
        std::lock_guard<std::mutex> lk(deferred_pose_updates_mutex_);
        deferred_pose_updates_.push_back(tx);
        RCLCPP_WARN(get_logger(),
            "[AutoMapSystem][POSE_TX][DEFER] tx_version=%lu source=%s mode=%s frame=%s reason=barrier_active deferred_size=%zu",
            static_cast<unsigned long>(tx.version_id),
            toString(tx.source), toString(tx.mode), toString(tx.frame),
            deferred_pose_updates_.size());
        return;
    }
    applyPoseTransaction(tx);
}

}  // namespace automap_pro
