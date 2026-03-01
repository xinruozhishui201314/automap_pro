#include "automap_pro/frontend/gps_fusion.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"

namespace automap_pro {

GPSFusion::GPSFusion() {
    chi2_threshold_ = ConfigManager::instance().gpsChi2Threshold();
}

GPSFusion::Observation GPSFusion::processForFrontend(
        const GPSMeasurement& meas,
        const Pose3d& current_pose,
        const Eigen::Matrix3d& odom_cov) const {
    Observation obs;
    obs.quality     = meas.quality;
    obs.should_use  = false;

    if (!meas.is_valid || meas.is_outlier) return obs;
    if (!isFrontendUsable(meas.quality))   return obs;

    // Mahalanobis consistency check
    Eigen::Vector3d delta = meas.position_enu - current_pose.translation();
    Eigen::Matrix3d total_cov = meas.covariance + odom_cov;
    double d_m = utils::mahalanobisDistance3d(delta, total_cov);
    if (d_m > std::sqrt(chi2_threshold_)) return obs;

    obs.position_enu = meas.position_enu;
    obs.covariance   = meas.covariance;
    obs.should_use   = true;

    // Inflate covariance for MEDIUM quality
    if (meas.quality == GPSQuality::MEDIUM) {
        obs.covariance *= 4.0;
    }

    return obs;
}

Eigen::Matrix3d GPSFusion::covarianceForBackend(const GPSMeasurement& meas) const {
    const auto& cfg = ConfigManager::instance();
    Eigen::Vector3d sigmas;
    switch (meas.quality) {
        case GPSQuality::EXCELLENT: sigmas = cfg.gpsCovExcellent(); break;
        case GPSQuality::HIGH:      sigmas = cfg.gpsCovHigh();      break;
        case GPSQuality::MEDIUM:    sigmas = cfg.gpsCovMedium();    break;
        case GPSQuality::LOW:       sigmas = cfg.gpsCovLow();       break;
        default:
            return Eigen::Matrix3d::Identity() * 1e6;
    }
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    cov(0,0) = sigmas(0) * sigmas(0);
    cov(1,1) = sigmas(1) * sigmas(1);
    cov(2,2) = sigmas(2) * sigmas(2);
    return cov;
}

bool GPSFusion::isFrontendUsable(GPSQuality quality) const {
    return quality == GPSQuality::EXCELLENT ||
           quality == GPSQuality::HIGH      ||
           quality == GPSQuality::MEDIUM;
}

bool GPSFusion::isBackendUsable(GPSQuality quality) const {
    return quality != GPSQuality::INVALID;
}

GPSState GPSFusion::currentState() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return state_;
}

void GPSFusion::setState(GPSState state) {
    std::lock_guard<std::mutex> lk(mutex_);
    state_ = state;
}

}  // namespace automap_pro
