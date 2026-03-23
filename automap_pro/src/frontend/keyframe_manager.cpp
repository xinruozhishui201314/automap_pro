#include "automap_pro/frontend/keyframe_manager.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#define MOD "KFManager"
#include <pcl/filters/voxel_grid.h>
#include <cmath>

namespace automap_pro {

std::atomic<uint64_t> KeyFrameManager::next_id_{0};

KeyFrameManager::KeyFrameManager() {
    const auto& cfg = ConfigManager::instance();
    min_translation_  = cfg.kfMinTranslation();
    min_rotation_deg_ = cfg.kfMinRotationDeg();
    max_interval_     = cfg.kfMaxInterval();
}

void KeyFrameManager::updateLivoInfo(const LivoKeyFrameInfo& info) {
    std::lock_guard<std::mutex> lk(mutex_);
    is_degenerate_ = info.is_degenerate;

    double cov_norm = info.esikf_cov_norm;
    double max_cov  = ConfigManager::instance().kfMaxEsikfCovNorm();

    if (info.is_degenerate || cov_norm > max_cov) {
        adaptive_dist_scale_ = 0.3;
        ALOG_DEBUG(MOD, "ESIKF degenerate/high_cov (cov_norm={:.4f}) → dist_scale=0.3", cov_norm);
    } else if (cov_norm > max_cov * 0.5) {
        adaptive_dist_scale_ = 0.6;
    } else {
        adaptive_dist_scale_ = 1.0;
    }
}

bool KeyFrameManager::shouldCreateKeyFrame(const Pose3d& cur_pose, double timestamp) {
    std::lock_guard<std::mutex> lk(mutex_);

    if (!has_last_) {
        // 第一个关键帧总是需要的，但我们不在这里更新 last_pose_，由 createKeyFrame 更新
        return true;
    }

    double scale = adaptive_dist_scale_;
    double eff_min_trans = min_translation_ * scale;
    double eff_min_rot   = min_rotation_deg_ * scale;

    double dist = (cur_pose.translation() - last_pose_.translation()).norm();
    if (dist >= eff_min_trans) {
        return true;
    }

    Eigen::AngleAxisd aa(cur_pose.rotation() * last_pose_.rotation().transpose());
    double angle_deg = std::abs(aa.angle()) * 180.0 / M_PI;
    if (angle_deg >= eff_min_rot) {
        return true;
    }

    // 可以在这里增加时间间隔检查
    // double dt = timestamp - last_ts_;
    // if (dt >= max_interval_) return true;

    return false;
}

KeyFrame::Ptr KeyFrameManager::createKeyFrame(
    const Pose3d& T_odom_b,
    const Mat66d& covariance,
    double timestamp,
    const CloudXYZIPtr& cloud_body,
    const CloudXYZIPtr& cloud_ds,
    const GPSMeasurement& gps,
    bool has_gps,
    uint64_t session_id)
{
    std::lock_guard<std::mutex> lk(mutex_);
    
    auto kf = std::make_shared<KeyFrame>();
    // ID 从 0 开始，fetch_add 返回旧值
    kf->id             = next_id_.fetch_add(1);
    kf->session_id     = session_id;
    kf->timestamp      = timestamp;
    kf->T_odom_b       = T_odom_b;
    kf->T_map_b_optimized = T_odom_b;
    kf->covariance     = covariance;
    kf->cloud_body     = cloud_body;
    kf->cloud_ds_body  = cloud_ds;
    kf->gps            = gps;
    kf->has_valid_gps  = has_gps;
    
    // 更新最后状态
    last_keyframe_ = kf;
    last_pose_     = T_odom_b;
    last_ts_       = timestamp;
    has_last_      = true;
    kf_count_++;

    for (auto& cb : cbs_) cb(kf);
    return kf;
}

void KeyFrameManager::resetLastPose(const Pose3d& pose, double ts) {
    std::lock_guard<std::mutex> lk(mutex_);
    last_pose_ = pose;
    last_ts_   = ts;
    has_last_  = true;
}

void KeyFrameManager::addKeyFrame(KeyFrame::Ptr kf) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (kf) {
        last_keyframe_ = kf;
        last_pose_ = kf->T_odom_b;
        last_ts_ = kf->timestamp;
        has_last_ = true;
        kf_count_++;
    }
}

KeyFrame::Ptr KeyFrameManager::getKeyFrameById(int id) const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (last_keyframe_ && static_cast<int>(last_keyframe_->id) == id) {
        return last_keyframe_;
    }
    return nullptr;
}

} // namespace automap_pro
