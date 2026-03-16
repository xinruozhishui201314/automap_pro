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
        last_pose_ = cur_pose;
        last_ts_   = timestamp;
        has_last_  = true;
        ALOG_INFO(MOD, "KF decision: first frame ts={:.3f}", timestamp);
        return true;
    }

    double scale = adaptive_dist_scale_;
    double eff_min_trans = min_translation_ * scale;
    double eff_min_rot   = min_rotation_deg_ * scale;

    double dist = (cur_pose.translation() - last_pose_.translation()).norm();
    if (dist >= eff_min_trans) {
        last_pose_ = cur_pose;
        last_ts_   = timestamp;
        ALOG_DEBUG(MOD, "KF decision: dist={:.3f}m >= {:.3f} ts={:.3f}", dist, eff_min_trans, timestamp);
        return true;
    }

    Eigen::AngleAxisd aa(cur_pose.rotation() * last_pose_.rotation().transpose());
    double angle_deg = std::abs(aa.angle()) * 180.0 / M_PI;
    if (angle_deg >= eff_min_rot) {
        last_pose_ = cur_pose;
        last_ts_   = timestamp;
        ALOG_DEBUG(MOD, "KF decision: rot={:.2f}deg >= {:.2f} ts={:.3f}", angle_deg, eff_min_rot, timestamp);
        return true;
    }

    // double dt = timestamp - last_ts_;
    // if (dt >= max_interval_) {
    //     last_pose_ = cur_pose;
    //     last_ts_   = timestamp;
    //     ALOG_DEBUG(MOD, "KF decision: dt={:.2f}s >= {:.2f} ts={:.3f}", dt, max_interval_, timestamp);
    //     return true;
    // }

    return false;
}

KeyFrame::Ptr KeyFrameManager::createKeyFrame(
    const Pose3d& T_w_b,
    const Mat66d& covariance,
    double timestamp,
    const CloudXYZIPtr& cloud_body,
    const CloudXYZIPtr& cloud_ds,
    const GPSMeasurement& gps,
    bool has_gps,
    uint64_t session_id)
{
    auto kf = std::make_shared<KeyFrame>();
    // 修复: fetch_add返回旧值，直接使用即可，ID从1开始
    kf->id             = next_id_.fetch_add(1);
    kf->session_id     = session_id;
    kf->timestamp      = timestamp;
    kf->T_w_b          = T_w_b;
    kf->T_w_b_optimized = T_w_b;
    kf->covariance     = covariance;
    kf->cloud_body     = cloud_body;
    kf->cloud_ds_body  = cloud_ds;
    kf->gps            = gps;
    kf->has_valid_gps  = has_gps;
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

bool KeyFrameManager::needNewKeyFrame(double timestamp, const Pose3d& pose, const Mat66d& cov, const CloudXYZIPtr& cloud) {
    std::lock_guard<std::mutex> lk(mutex_);
    
    if (!has_last_) {
        // 第一个关键帧
        last_pose_ = pose;
        last_ts_   = timestamp;
        has_last_  = true;
        
        // 创建第一个关键帧
        last_keyframe_ = std::make_shared<KeyFrame>();
        last_keyframe_->id = next_id_.fetch_add(1);
        last_keyframe_->session_id = 0;
        last_keyframe_->timestamp = timestamp;
        last_keyframe_->T_w_b = pose;
        last_keyframe_->covariance = cov;
        last_keyframe_->cloud_body = cloud;
        kf_count_++;
        
        ALOG_INFO(MOD, "needNewKeyFrame: first frame ts=%.3f id=%d", timestamp, last_keyframe_->id);
        return true;
    }

    double scale = adaptive_dist_scale_;
    double eff_min_trans = min_translation_ * scale;
    double eff_min_rot   = min_rotation_deg_ * scale;

    double dist = (pose.translation() - last_pose_.translation()).norm();
    Eigen::AngleAxisd aa(pose.rotation() * last_pose_.rotation().transpose());
    double angle_deg = std::abs(aa.angle()) * 180.0 / M_PI;
    
    if (dist >= eff_min_trans || angle_deg >= eff_min_rot) {
        last_pose_ = pose;
        last_ts_   = timestamp;
        
        // 创建新关键帧
        last_keyframe_ = std::make_shared<KeyFrame>();
        last_keyframe_->id = next_id_.fetch_add(1);
        last_keyframe_->session_id = 0;
        last_keyframe_->timestamp = timestamp;
        last_keyframe_->T_w_b = pose;
        last_keyframe_->covariance = cov;
        last_keyframe_->cloud_body = cloud;
        kf_count_++;
        
        ALOG_DEBUG(MOD, "needNewKeyFrame: created id=%d dist=%.3f angle=%.2fdeg", 
                   last_keyframe_->id, dist, angle_deg);
        return true;
    }
    
    return false;
}

void KeyFrameManager::addKeyFrame(KeyFrame::Ptr kf) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (kf) {
        last_keyframe_ = kf;
        last_pose_ = kf->T_w_b;
        last_ts_ = kf->timestamp;
        has_last_ = true;
        kf_count_++;
    }
}

KeyFrame::Ptr KeyFrameManager::getKeyFrameById(int id) const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (last_keyframe_ && last_keyframe_->id == id) {
        return last_keyframe_;
    }
    return nullptr;
}

} // namespace automap_pro
