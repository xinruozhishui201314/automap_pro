#include "automap_pro/frontend/keyframe_manager.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"

namespace automap_pro {

KeyFrameManager::KeyFrameManager() {
    const auto& cfg = ConfigManager::instance();
    min_translation_  = cfg.kfMinTranslation();
    min_rotation_rad_ = cfg.kfMinRotationDeg() * M_PI / 180.0;
    max_interval_     = cfg.kfMaxInterval();
}

void KeyFrameManager::registerCallback(KeyFrameCallback cb) {
    callbacks_.push_back(std::move(cb));
}

void KeyFrameManager::setMinTranslation(double min_translation) {
    std::lock_guard<std::mutex> lk(mutex_);
    min_translation_ = min_translation;
}

void KeyFrameManager::setMinRotationDeg(double min_rotation_deg) {
    std::lock_guard<std::mutex> lk(mutex_);
    min_rotation_rad_ = min_rotation_deg * M_PI / 180.0;
}

void KeyFrameManager::setMaxInterval(double max_interval) {
    std::lock_guard<std::mutex> lk(mutex_);
    max_interval_ = max_interval;
}

bool KeyFrameManager::shouldCreateKeyFrame(const Pose3d& pose, double timestamp) const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (last_kf_time_ < 0.0) return true;  // first keyframe

    // Time condition
    if (timestamp - last_kf_time_ > max_interval_) return true;

    // Translation condition
    double trans = (pose.translation() - last_kf_pose_.translation()).norm();
    if (trans > min_translation_) return true;

    // Rotation condition
    Pose3d delta = last_kf_pose_.inverse() * pose;
    if (utils::rotationAngleDeg(delta) * M_PI / 180.0 > min_rotation_rad_) return true;

    return false;
}

KeyFrame::Ptr KeyFrameManager::createKeyFrame(
        const Pose3d& pose, const Pose3d& /*unused*/,
        const Mat66d& covariance,
        double timestamp,
        const CloudXYZIPtr& cloud,
        const CloudXYZIPtr& cloud_ds,
        const GPSMeasurement& gps,
        bool has_valid_gps,
        uint64_t session_id) {

    auto kf = std::make_shared<KeyFrame>();
    kf->id          = nextId();
    kf->session_id  = session_id;
    kf->timestamp   = timestamp;
    kf->T_w_b       = pose;
    kf->T_w_b_optimized = pose;
    kf->covariance  = covariance;
    kf->cloud_body  = cloud;
    kf->cloud_ds_body = cloud_ds;
    kf->gps         = gps;
    kf->has_valid_gps = has_valid_gps;

    {
        std::lock_guard<std::mutex> lk(mutex_);
        last_kf_pose_ = pose;
        last_kf_time_ = timestamp;
        last_kf_      = kf;
    }

    for (auto& cb : callbacks_) cb(kf);
    return kf;
}

uint64_t KeyFrameManager::nextId() {
    return id_counter_++;
}

const KeyFrame::Ptr& KeyFrameManager::lastKeyFrame() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return last_kf_;
}

void KeyFrameManager::reset() {
    std::lock_guard<std::mutex> lk(mutex_);
    last_kf_time_ = -1e9;
    id_counter_.store(0);
    last_kf_.reset();
}

}  // namespace automap_pro
