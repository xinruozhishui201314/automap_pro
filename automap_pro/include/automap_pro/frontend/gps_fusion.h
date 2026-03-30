#pragma once
/**
 * @file frontend/gps_fusion.h
 * @brief 前端：关键帧、LIVO 适配、GPS 融合与桥接。
 */


#include <deque>
#include <mutex>
#include <functional>

#include "automap_pro/core/data_types.h"

namespace automap_pro {

// ──────────────────────────────────────────────────────────
// GPSFusion: adapts GPS observations for use in ESIKF / HBA
// ──────────────────────────────────────────────────────────
class GPSFusion {
public:
    struct Observation {
        Eigen::Vector3d position_enu;
        Eigen::Matrix3d covariance;
        GPSQuality quality;
        bool should_use = false;
    };

    GPSFusion();
    ~GPSFusion() = default;

    // Process raw GPS measurement; returns adaptive observation
    Observation processForFrontend(const GPSMeasurement& meas,
                                    const Pose3d& current_pose,
                                    const Eigen::Matrix3d& odom_cov) const;

    // Returns covariance for HBA backend based on quality
    Eigen::Matrix3d covarianceForBackend(const GPSMeasurement& meas) const;

    // Check if GPS is usable in frontend ESIKF
    bool isFrontendUsable(GPSQuality quality) const;

    // Check if GPS should be added as HBA factor
    bool isBackendUsable(GPSQuality quality) const;

    GPSState currentState() const;
    void     setState(GPSState state);

private:
    GPSState state_ = GPSState::INIT;
    mutable std::mutex mutex_;

    double chi2_threshold_ = 11.345;
};

}  // namespace automap_pro
