#pragma once

#include <atomic>
#include <deque>
#include <mutex>
#include <functional>
#include <memory>

#include "automap_pro/core/data_types.h"

namespace automap_pro {

// ──────────────────────────────────────────────────────────
// KeyFrameManager: decides when to create keyframes
// and manages the keyframe queue
// ──────────────────────────────────────────────────────────
class KeyFrameManager {
public:
    using KeyFrameCallback = std::function<void(const KeyFrame::Ptr&)>;

    KeyFrameManager();
    ~KeyFrameManager() = default;

    void registerCallback(KeyFrameCallback cb);

    // Keyframe policy (thread-safe)
    void setMinTranslation(double min_translation);
    void setMinRotationDeg(double min_rotation_deg);
    void setMaxInterval(double max_interval);

    // Check if a new keyframe should be created given current state
    bool shouldCreateKeyFrame(const Pose3d& current_pose, double timestamp) const;

    // Build and dispatch a keyframe
    KeyFrame::Ptr createKeyFrame(
        const Pose3d& pose,
        const Pose3d& pose_covariance_pose,  // unused - use cov below
        const Mat66d& covariance,
        double timestamp,
        const CloudXYZIPtr& cloud,
        const CloudXYZIPtr& cloud_ds,
        const GPSMeasurement& gps,
        bool has_valid_gps,
        uint64_t session_id);

    uint64_t nextId();
    const KeyFrame::Ptr& lastKeyFrame() const;

    void reset();

private:
    Pose3d last_kf_pose_;
    double last_kf_time_ = -1e9;
    std::atomic<uint64_t> id_counter_{0};
    KeyFrame::Ptr last_kf_;
    std::vector<KeyFrameCallback> callbacks_;
    mutable std::mutex mutex_;

    double min_translation_;
    double min_rotation_rad_;
    double max_interval_;
};

}  // namespace automap_pro
