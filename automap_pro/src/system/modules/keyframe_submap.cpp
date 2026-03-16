// 模块4: 关键帧与子图管理
// 包含: tryCreateKeyFrame, onSubmapFrozen, transformWorldToBody, processKeyframeTask

#include "automap_pro/system/automap_system.h"

namespace automap_pro {

// ─────────────────────────────────────────────────────────────────────────────
// 关键帧与子图管理 - 任务投递版本
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::tryCreateKeyFrame(double ts) {
    std::lock_guard<std::mutex> lk(data_mutex_);
    tryCreateKeyFrame(ts, last_odom_pose_, last_cov_, last_cloud_, &last_livo_info_, nullptr);
}

void AutoMapSystem::tryCreateKeyFrame(double ts, const Pose3d& pose, const Mat66d& cov,
                                       const CloudXYZIPtr& cloud,
                                       const LivoKeyFrameInfo* optional_livo_info,
                                       const CloudXYZIPtr* optional_cloud_ds) {
    // 如果 cloud 已转换为 body 系，这里不再转换（cloud_for_kf 已经是 body 系）
    // transformWorldToBody 仅在 backendWorkerLoop 中根据 cloud_frame 配置调用
    
    const bool kf_created = kf_manager_.needNewKeyFrame(ts, pose, cov, cloud);
    if (!kf_created) {
        return;
    }

    KeyFrame::Ptr kf = std::make_shared<KeyFrame>();
    kf->timestamp = ts;
    kf->T_w_b = pose;
    kf->covariance = cov;
    kf->cloud_body = cloud;
    kf->cloud_ds_body = optional_cloud_ds ? *optional_cloud_ds : nullptr;
    if (optional_livo_info) {
        kf->livo_info = *optional_livo_info;
    }
    kf->session_id = current_session_id_;

    // 如果 GPS 已对齐，对新关键帧应用相同的刚体变换
    // 使用 mutex 保护确保读取到一致的值
    bool aligned = gps_aligned_.load();
    if (aligned) {
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        {
            std::lock_guard<std::mutex> lk(gps_transform_mutex_);
            R = gps_transform_R_;
            t = gps_transform_t_;
        }
        kf->T_w_b = Pose3d(R * pose.linear(), R * pose.translation() + t);
    }

    // 保存前一个关键帧信息用于里程计因子计算（在投递之前获取）
    auto prev_kf = kf_manager_.getLastKeyFrame();
    bool has_prev_kf = (prev_kf != nullptr);
    int prev_kf_id = has_prev_kf ? prev_kf->id : 0;

    // 投递子图管理任务到opt_worker（统一在opt_worker线程处理关键帧添加和子图冻结）
    // 这样确保所有GTSAM调用在单一线程执行，避免跨线程竞态
    {
        std::lock_guard<std::mutex> lk(opt_task_mutex_);
        if (opt_task_queue_.size() < kMaxOptTaskQueueSize) {
            OptTaskItem task;
            task.type = OptTaskItem::Type::KEYFRAME_CREATE;
            task.keyframe = kf;
            task.has_prev_kf = has_prev_kf;
            task.prev_kf_id = prev_kf_id;
            task.gps_aligned = gps_aligned_.load();
            {
                std::lock_guard<std::mutex> lk(gps_transform_mutex_);
                task.gps_transform_R = gps_transform_R_;
                task.gps_transform_t = gps_transform_t_;
            }
            opt_task_queue_.push_back(task);
            opt_task_cv_.notify_one();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][KEYFRAME] enqueued KEYFRAME_CREATE task kf_id=%d prev_kf_id=%d", 
                        kf->id, prev_kf_id);
        } else {
            RCLCPP_WARN(get_logger(), "[AutoMapSystem][KEYFRAME] opt_task_queue full, falling back to direct call");
            // 降级方案：直接调用（保持兼容性）
            // 使用 submap_update_mutex_ 保护，与 onPoseUpdated 一致，避免竞态条件
            {
                std::lock_guard<std::mutex> lk(submap_update_mutex_);
                submap_manager_.addKeyFrame(kf);
                kf_manager_.addKeyFrame(kf);
            }

            // 检查子图冻结（使用 submap_update_mutex_ 保护）
            {
                std::lock_guard<std::mutex> lk(submap_update_mutex_);
                if (submap_manager_.needFreezeSubmap()) {
                    auto frozen_sm = submap_manager_.freezeCurrentSubmap();
                    if (frozen_sm) {
                        RCLCPP_INFO(get_logger(), "[AutoMapSystem] SubMap %d frozen (fallback), keyframes=%zu",
                                    frozen_sm->id, frozen_sm->keyframes.size());
                    }
                }
            }

            // 里程计因子计算和投递（标记为降级路径，避免重复添加）
            if (has_prev_kf && prev_kf) {
                Pose3d rel = prev_kf->T_w_b.inverse() * kf->T_w_b;
                Mat66d info = computeOdomInfoMatrixForKeyframes(prev_kf, kf, rel);
                std::lock_guard<std::mutex> lk2(opt_task_mutex_);
                if (opt_task_queue_.size() < kMaxOptTaskQueueSize) {
                    OptTaskItem odom_task;
                    odom_task.type = OptTaskItem::Type::ODOM_FACTOR;
                    odom_task.from_id = prev_kf->id;
                    odom_task.to_id = kf->id;
                    odom_task.rel_pose = rel;
                    odom_task.info_matrix = info;
                    opt_task_queue_.push_back(odom_task);
                    opt_task_cv_.notify_one();
                }
            }

            // 触发回环检测
            if (loop_detector_.isRunning()) {
                std::lock_guard<std::mutex> lk3(loop_trigger_mutex_);
                if (loop_trigger_queue_.size() < kMaxLoopTriggerQueueSize) {
                    loop_trigger_queue_.push_back(kf);
                    loop_trigger_cv_.notify_one();
                }
            }
        }
    }
}

CloudXYZIPtr AutoMapSystem::transformWorldToBody(const CloudXYZIPtr& world_cloud, const Pose3d& T_w_b) const {
    // 将世界系点云转为 body 系：T_b_w * cloud
    // T_w_b 是 body 在 world 中的位姿
    // T_b_w = T_w_b.inverse() 是 world 在 body 中的位姿
    if (!world_cloud || world_cloud->empty()) {
        return nullptr;
    }
    Eigen::Isometry3d T_b_w = Eigen::Isometry3d(T_w_b.inverse().matrix());
    CloudXYZIPtr body_cloud = std::make_shared<CloudXYZ>();
    pcl::transformPointCloud(*world_cloud, *body_cloud, T_b_w.matrix().cast<float>());
    return body_cloud;
}

void AutoMapSystem::onSubmapFrozen(const SubMap::Ptr& submap) {
    frozen_submap_count_++;
    const int count = frozen_submap_count_.load();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] SubMap %d frozen (total frozen: %d), keyframes=%zu, points=%llu",
                submap->id, count, submap->keyframes.size(),
                static_cast<unsigned long long>(submap->cloud->size()));

    // 注意：不再在子图冻结时触发HBA，HBA只在建图结束后触发一次

    // 尝试触发 GPS 对齐（如果还未对齐且有足够的子图）
    if (!gps_aligned_.load() && count >= ConfigManager::instance().gpsAlignMinSubmaps()) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem] Enough submaps (%d) for GPS alignment attempt", count);
        gps_manager_.requestAlignment();
    }
}

}  // namespace automap_pro
