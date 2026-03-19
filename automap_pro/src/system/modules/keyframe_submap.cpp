// 模块4: 关键帧与子图管理
// 包含: tryCreateKeyFrame, onSubmapFrozen, transformWorldToBody, processKeyframeTask

#include "automap_pro/system/automap_system.h"
#include "automap_pro/core/config_manager.h"
#include <pcl/common/transforms.h>

namespace automap_pro {

// ─────────────────────────────────────────────────────────────────────────────
// 关键帧与子图管理 - 任务投递版本
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::tryCreateKeyFrame(double ts) {
    Pose3d pose;
    Mat66d cov;
    CloudXYZIPtr cloud;
    LivoKeyFrameInfo info;
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        pose = last_odom_pose_;
        cov = last_cov_;
        cloud = last_cloud_;
        info = last_livo_info_;
    }
    tryCreateKeyFrame(ts, pose, cov, cloud, &info, nullptr);
}

void AutoMapSystem::tryCreateKeyFrame(double ts, const Pose3d& pose, const Mat66d& cov,
                                       const CloudXYZIPtr& cloud,
                                       const LivoKeyFrameInfo* optional_livo_info,
                                       const CloudXYZIPtr* optional_cloud_ds) {
    // 1. 判断是否需要创建关键帧
    if (!kf_manager_.shouldCreateKeyFrame(pose, ts)) {
        return;
    }

    // 2. 查询当前可能的 GPS 观测
    GPSMeasurement gps;
    bool has_gps = false;
    auto gps_opt = gps_manager_.queryByTimestamp(ts);
    if (gps_opt) {
        gps = *gps_opt;
        has_gps = true;
    }

    // 🔧 V2 修复：在创建新关键帧前，先获取当前最新的关键帧作为“前一帧”，避免 createKeyFrame 内部更新 last_keyframe_ 后无法获取
    KeyFrame::Ptr prev_kf = kf_manager_.getLastKeyFrame();

    // 3. 调用 KeyFrameManager 创建关键帧对象
    // 注意：createKeyFrame 会自动递增 ID 并更新 kf_manager_ 内部的 last_pose_
    KeyFrame::Ptr kf = kf_manager_.createKeyFrame(
        pose, cov, ts, cloud, 
        (optional_cloud_ds ? *optional_cloud_ds : nullptr),
        gps, has_gps, current_session_id_
    );
    
    if (!kf) return;

    if (optional_livo_info) {
        kf->livo_info = *optional_livo_info;
    }

    // 4. 如果 GPS 已对齐，对新关键帧应用相同的刚体变换（地图坐标系修正）
    bool aligned = gps_aligned_.load();
    Eigen::Matrix3d R_snapshot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_snapshot = Eigen::Vector3d::Zero();
    if (aligned) {
        {
            std::lock_guard<std::mutex> lk(gps_transform_mutex_);
            R_snapshot = gps_transform_R_;
            t_snapshot = gps_transform_t_;
        }
        {
            Pose3d T = Pose3d::Identity();
            T.linear() = R_snapshot * kf->T_w_b.linear();
            T.translation() = R_snapshot * kf->T_w_b.translation() + t_snapshot;
            kf->T_w_b = T;
        }
    }

    // 5. 获取前一关键帧用于里程计因子
    // 在 V2 中，我们必须保存当前 kf 的 parent 指针，确保异步执行时位姿关系正确
    bool has_prev_kf = (prev_kf != nullptr);
    int prev_kf_id = has_prev_kf ? static_cast<int>(prev_kf->id) : -1;

    // 6. 投递任务到 opt_worker 线程
    if (task_dispatcher_) {
        if (task_dispatcher_->submitKeyFrameCreate(kf, has_prev_kf, prev_kf_id, prev_kf, aligned, R_snapshot, t_snapshot)) {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][KEYFRAME] enqueued KEYFRAME_CREATE task kf_id=%lu prev_kf_id=%d", 
                        kf->id, prev_kf_id);
        } else {
            RCLCPP_WARN(get_logger(), "[AutoMapSystem][KEYFRAME] task_dispatcher failed to submit KEYFRAME_CREATE kf_id=%lu", kf->id);
            // 降级：直接更新（不推荐，但在任务队列满且必须处理时使用）
            {
                std::lock_guard<std::mutex> lk_upd(submap_update_mutex_);
                submap_manager_.addKeyFrame(kf);
            }
        }
    }
}

CloudXYZIPtr AutoMapSystem::transformWorldToBody(const CloudXYZIPtr& world_cloud, const Pose3d& T_w_b) const {
    if (!world_cloud || world_cloud->empty()) {
        return nullptr;
    }
    Eigen::Isometry3d T_b_w = Eigen::Isometry3d(T_w_b.inverse().matrix());
    CloudXYZIPtr body_cloud = std::make_shared<CloudXYZI>();
    pcl::transformPointCloud(*world_cloud, *body_cloud, T_b_w.matrix().cast<float>());
    return body_cloud;
}

void AutoMapSystem::onSubmapFrozen(const SubMap::Ptr& submap) {
    if (!submap) return;

    frozen_submap_count_++;
    const int count = frozen_submap_count_.load();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] SubMap %d frozen (total frozen: %d), keyframes=%zu, points=%zu",
                submap->id, count, submap->keyframes.size(), submap->merged_cloud ? submap->merged_cloud->size() : 0u);

    // 🔧 V2 修复：通过 TaskDispatcher 异步添加子图节点和里程计因子到 iSAM2
    if (task_dispatcher_) {
        // 首个子图节点添加 Prior
        bool is_first = (count == 1);
        task_dispatcher_->submitSubmapNode(submap->id, submap->pose_w_anchor, is_first);
        
        // 添加与前一子图的里程计因子
        auto all_frozen = submap_manager_.getFrozenSubmaps();
        if (all_frozen.size() >= 2) {
            // 查找前一个子图（即倒数第二个，因为当前 submap 已在 getFrozenSubmaps 结果中）
            auto it = std::find_if(all_frozen.rbegin(), all_frozen.rend(),
                                   [&](const SubMap::Ptr& s) { return s->id == submap->id; });
            if (it != all_frozen.rend()) {
                auto next_it = std::next(it);
                if (next_it != all_frozen.rend()) {
                    auto prev = *next_it;
                    if (prev) {
                        Pose3d rel = prev->pose_w_anchor.inverse() * submap->pose_w_anchor;
                        Mat66d info = computeOdomInfoMatrix(prev, submap, rel);
                        task_dispatcher_->submitOdomFactor(prev->id, submap->id, rel, info);
                        RCLCPP_INFO(get_logger(), "[AutoMapSystem][SUBMAP_ODOM] added factor: sm_%d -> sm_%d",
                                    prev->id, submap->id);
                    }
                }
            }
        }
        
        // 立即触发一次优化更新
        task_dispatcher_->submitForceUpdate();
    }

    // 🔧 V2 修复：触发 HBA 周期性全局优化（若已启用且达到触发频率）
    if (ConfigManager::instance().hbaEnabled() && ConfigManager::instance().hbaOnLoop()) {
        const int trigger_mod = ConfigManager::instance().hbaTriggerSubmaps();
        if (count % trigger_mod == 0) {
            auto all_frozen = submap_manager_.getFrozenSubmaps();
            hba_optimizer_.triggerAsync(all_frozen, false, "onSubmapFrozen");
        }
    }

    // 尝试触发 GPS 对齐（ConfigManager 无 gpsAlignMinSubmaps，用最少子图数 2）
    if (!gps_aligned_.load() && count >= 2) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem] Enough submaps (%d) for GPS alignment attempt", count);
        gps_manager_.requestAlignment();
    }
}

}  // namespace automap_pro
