#pragma once
/**
 * @file backend/gps_constraint_policy.h
 * @brief 后端优化：iSAM2、HBA、GPS/回环因子、任务队列与坐标管理。
 */


#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/data_types.h"
#include <cmath>
#include <limits>

namespace automap_pro {

enum class GPSConstraintRejectReason : int {
    NONE = 0,
    NO_GPS_ON_KEYFRAME = 1,
    QUALITY_BELOW_POLICY = 2,
    ENU_NOT_FINITE = 3,
    COV_NOT_FINITE = 4,
    OUTSIDE_TIME_WINDOW = 5,
};

struct GPSConstraintDecision {
    bool accepted = false;
    GPSConstraintRejectReason reason = GPSConstraintRejectReason::NONE;
};

inline int gpsQualityRank(GPSQuality q) {
    return static_cast<int>(q);
}

inline bool gpsQualityAcceptedByPolicy(GPSQuality q, const ConfigManager& cfg) {
    return gpsQualityRank(q) >= cfg.gpsMinAcceptedQualityLevel();
}

/** 使用缓存的 min_level，避免 worker 线程访问 ConfigManager 单例（shutdown 时 SIGSEGV） */
inline bool gpsQualityAcceptedByPolicy(GPSQuality q, int min_accepted_level) {
    return gpsQualityRank(q) >= min_accepted_level;
}

inline GPSConstraintDecision evaluateKeyframeGpsConstraint(
    const GPSMeasurement& gps,
    bool has_valid_gps,
    int min_accepted_level,
    bool require_finite_covariance,
    double measurement_time_abs_diff_s = -1.0,
    double max_allowed_time_abs_diff_s = std::numeric_limits<double>::infinity())
{
    if (!has_valid_gps) {
        return {false, GPSConstraintRejectReason::NO_GPS_ON_KEYFRAME};
    }
    if (std::isfinite(max_allowed_time_abs_diff_s) &&
        measurement_time_abs_diff_s >= 0.0 &&
        measurement_time_abs_diff_s > max_allowed_time_abs_diff_s) {
        return {false, GPSConstraintRejectReason::OUTSIDE_TIME_WINDOW};
    }
    if (!gpsQualityAcceptedByPolicy(gps.quality, min_accepted_level)) {
        return {false, GPSConstraintRejectReason::QUALITY_BELOW_POLICY};
    }
    if (!gps.position_enu.allFinite()) {
        return {false, GPSConstraintRejectReason::ENU_NOT_FINITE};
    }
    if (require_finite_covariance && !gps.covariance.allFinite()) {
        return {false, GPSConstraintRejectReason::COV_NOT_FINITE};
    }
    return {true, GPSConstraintRejectReason::NONE};
}

}  // namespace automap_pro
