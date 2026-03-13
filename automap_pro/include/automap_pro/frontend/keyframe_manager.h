#pragma once
#include "automap_pro/core/data_types.h"
#include <atomic>
#include <functional>
#include <mutex>

namespace automap_pro {

/**
 * 关键帧管理器（自适应策略）
 *
 * 基础触发条件（三选一，满足其一即添加关键帧）：
 *   - 与上一关键帧平移 ≥ min_translation_（默认 0.5 m）
 *   - 与上一关键帧旋转 ≥ min_rotation_deg_（默认 10°）
 *   - 时间间隔 ≥ max_interval_
 *
 * 自适应触发（基于 ESIKF 质量）：
 *   - 当 ESIKF 退化（is_degenerate=true）时 → 降低阈值，强制插入 KF
 *   - 当 esikf_cov_norm 超高（定位漂移）时 → 更频繁插入 KF
 */
class KeyFrameManager {
public:
    explicit KeyFrameManager();

    void setMinTranslation(double v)  { min_translation_ = v; }
    void setMinRotationDeg(double v)  { min_rotation_deg_ = v; }
    void setMaxInterval(double v)     { max_interval_ = v; }

    /** 更新 ESIKF 质量信息，影响 KF 触发策略 */
    void updateLivoInfo(const LivoKeyFrameInfo& info);

    /** 判断是否应创建关键帧 */
    bool shouldCreateKeyFrame(const Pose3d& cur_pose, double timestamp);

    /** 创建关键帧（不会检查条件，直接创建） */
    KeyFrame::Ptr createKeyFrame(
        const Pose3d& T_w_b,
        const Mat66d& covariance,
        double timestamp,
        const CloudXYZIPtr& cloud_body,
        const CloudXYZIPtr& cloud_ds,
        const GPSMeasurement& gps,
        bool has_gps,
        uint64_t session_id);

    void registerCallback(KeyFrameCallback cb) { cbs_.push_back(std::move(cb)); }
    void resetLastPose(const Pose3d& pose, double ts);
    int  keyframeCount() const { return kf_count_; }

private:
    double min_translation_  = 0.5;    // 米，与上一关键帧位置变化≥此值即添加
    double min_rotation_deg_ = 10.0;   // 度，与上一关键帧角度变化≥此值即添加
    double max_interval_     = 2.0;    // 秒

    Pose3d last_pose_           = Pose3d::Identity();
    double last_ts_             = -1.0;
    bool   has_last_            = false;
    int    kf_count_            = 0;

    // 自适应参数（基于 ESIKF 质量）
    double adaptive_dist_scale_   = 1.0;  // 1.0=正常，0.3=退化时降低阈值
    bool   is_degenerate_         = false;

    mutable std::mutex mutex_;
    std::vector<KeyFrameCallback> cbs_;

    static std::atomic<uint64_t> next_id_;
};

} // namespace automap_pro
