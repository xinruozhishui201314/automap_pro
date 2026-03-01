#include "automap_pro/sensor/gps_processor.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"

namespace automap_pro {

GPSProcessor::GPSProcessor() {
    const auto& cfg = ConfigManager::instance();
    excellent_hdop_              = cfg.gpsExcellentHDOP();
    high_hdop_                   = cfg.gpsHighHDOP();
    medium_hdop_                 = cfg.gpsMediumHDOP();
    max_jump_                    = cfg.gpsMaxJump();
    max_velocity_                = cfg.gpsMaxVelocity();
    chi2_threshold_              = cfg.gpsChi2Threshold();
    consecutive_valid_required_  = cfg.gpsConsecutiveValid();
    jump_detection_enabled_      = cfg.gpsJumpDetection();
    consistency_check_enabled_   = cfg.gpsConsistencyCheck();
}

void GPSProcessor::init(rclcpp::Node::SharedPtr node) {
    logger_ = node->get_logger();
    std::string topic = ConfigManager::instance().gpsTopic();
    sub_ = node->create_subscription<sensor_msgs::msg::NavSatFix>(
        topic, 100, std::bind(&GPSProcessor::gpsCallback, this, std::placeholders::_1));
    RCLCPP_INFO(logger_, "[GPSProcessor] Subscribing to %s", topic.c_str());
}

void GPSProcessor::setENUOrigin(double lat, double lon, double alt) {
    std::lock_guard<std::mutex> lk(mutex_);
    enu_origin_.latitude  = lat;
    enu_origin_.longitude = lon;
    enu_origin_.altitude  = alt;
    enu_origin_.initialized = true;
    RCLCPP_INFO(logger_, "[GPSProcessor] ENU origin set: lat=%.7f lon=%.7f alt=%.2f", lat, lon, alt);
}

void GPSProcessor::registerCallback(MeasurementCallback cb) {
    callbacks_.push_back(std::move(cb));
}

void GPSProcessor::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    process(msg);
}

void GPSProcessor::process(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);

    if (!enu_origin_.initialized) {
        if (msg->status.status < 0) return;
        enu_origin_.latitude  = msg->latitude;
        enu_origin_.longitude = msg->longitude;
        enu_origin_.altitude  = msg->altitude;
        enu_origin_.initialized = true;
        RCLCPP_INFO(logger_, "[GPSProcessor] Auto-init ENU origin: lat=%.7f lon=%.7f",
                 msg->latitude, msg->longitude);
    }

    GPSMeasurement meas;
    meas.timestamp  = rclcpp::Time(msg->header.stamp).seconds();
    meas.latitude   = msg->latitude;
    meas.longitude  = msg->longitude;
    meas.altitude   = msg->altitude;

    double e, n, u;
    utils::wgs84ToENU(msg->latitude, msg->longitude, msg->altitude,
                      enu_origin_.latitude, enu_origin_.longitude, enu_origin_.altitude,
                      e, n, u);
    meas.position_enu = Eigen::Vector3d(e, n, u);
    meas.quality = assessQuality(*msg);
    meas.hdop = 99.0;

    if (msg->position_covariance_type > 0) {
        meas.covariance = Eigen::Matrix3d::Identity();
        meas.covariance(0,0) = std::max(msg->position_covariance[0], 0.01);
        meas.covariance(1,1) = std::max(msg->position_covariance[4], 0.01);
        meas.covariance(2,2) = std::max(msg->position_covariance[8], 0.01);
    } else {
        meas.covariance = qualityToCovariance(meas.quality, meas.hdop);
    }

    meas.is_valid = (meas.quality != GPSQuality::INVALID);

    if (jump_detection_enabled_ && !gps_history_.empty()) {
        double dt = meas.timestamp - gps_history_.back().first;
        if (!jumpDetection(meas.position_enu, dt)) {
            meas.is_outlier = true;
            meas.is_valid   = false;
            consecutive_outliers_++;
            consecutive_valid_ = 0;
        } else {
            consecutive_outliers_ = 0;
            consecutive_valid_++;
        }
    }

    if (consistency_check_enabled_ && !meas.is_outlier && !odom_history_.empty()) {
        if (!consistencyCheck(meas)) {
            meas.is_outlier = true;
            meas.is_valid   = false;
            consecutive_outliers_++;
        }
    }

    gps_history_.push_back({meas.timestamp, meas.position_enu});
    if (gps_history_.size() > 200) gps_history_.pop_front();
    updateState(meas.is_valid && !meas.is_outlier);
    meas.is_valid = (state_ == GPSState::TRACKING || state_ == GPSState::DEGRADED) && meas.is_valid;

    for (auto& cb : callbacks_) cb(meas);
}

GPSQuality GPSProcessor::assessQuality(const sensor_msgs::msg::NavSatFix& msg) const {
    if (msg.status.status < 0) return GPSQuality::INVALID;
    double cov_xy = 1.0;
    if (msg.position_covariance_type > 0) {
        cov_xy = std::sqrt(std::max(msg.position_covariance[0], 0.0));
    }
    if (msg.status.status >= 2) {
        if (cov_xy <= excellent_hdop_) return GPSQuality::EXCELLENT;
        if (cov_xy <= high_hdop_)     return GPSQuality::HIGH;
    }
    if (msg.status.status >= 1) {
        if (cov_xy <= medium_hdop_) return GPSQuality::MEDIUM;
        return GPSQuality::LOW;
    }
    if (msg.status.status == 0) return GPSQuality::LOW;
    return GPSQuality::INVALID;
}

Eigen::Matrix3d GPSProcessor::qualityToCovariance(GPSQuality q, double /*hdop*/) const {
    const auto& cfg = ConfigManager::instance();
    Eigen::Vector3d sigmas;
    switch (q) {
        case GPSQuality::EXCELLENT: sigmas = cfg.gpsCovExcellent(); break;
        case GPSQuality::HIGH:      sigmas = cfg.gpsCovHigh();      break;
        case GPSQuality::MEDIUM:    sigmas = cfg.gpsCovMedium();     break;
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

bool GPSProcessor::jumpDetection(const Eigen::Vector3d& pos, double dt) {
    if (gps_history_.empty()) return true;
    const auto& prev = gps_history_.back().second;
    double dist = (pos - prev).norm();
    double max_allowed = max_velocity_ * std::max(dt, 0.1) + max_jump_;
    return dist <= max_allowed;
}

bool GPSProcessor::consistencyCheck(const GPSMeasurement& meas) {
    if (odom_history_.empty()) return true;
    Eigen::Vector3d odom_pos;
    double best_dt = 1e9;
    for (const auto& [ts, pos] : odom_history_) {
        double dt = std::abs(ts - meas.timestamp);
        if (dt < best_dt) { best_dt = dt; odom_pos = pos; }
    }
    if (best_dt > 1.0) return true;
    Eigen::Vector3d delta = meas.position_enu - odom_pos;
    Eigen::Matrix3d cov = meas.covariance + Eigen::Matrix3d::Identity() * 0.25;
    double d_m = utils::mahalanobisDistance3d(delta, cov);
    return d_m <= std::sqrt(chi2_threshold_);
}

void GPSProcessor::updateState(bool valid) {
    if (!valid) {
        consecutive_outliers_++;
        consecutive_valid_ = 0;
        if (consecutive_outliers_ >= 3) {
            state_ = (state_ == GPSState::TRACKING) ? GPSState::DEGRADED : GPSState::LOST;
        }
    } else {
        consecutive_valid_++;
        if (consecutive_valid_ >= consecutive_valid_required_) {
            state_ = GPSState::TRACKING;
            consecutive_outliers_ = 0;
        }
    }
    if (state_ == GPSState::INIT && valid) state_ = GPSState::TRACKING;
}

void GPSProcessor::updateOdometryPose(double timestamp, const Eigen::Vector3d& pos) {
    std::lock_guard<std::mutex> lk(mutex_);
    odom_history_.push_back({timestamp, pos});
    if (odom_history_.size() > 500) odom_history_.pop_front();
}

GPSState  GPSProcessor::currentState() const { return state_; }
ENUOrigin GPSProcessor::enuOrigin()    const { return enu_origin_; }
bool      GPSProcessor::hasOrigin()   const { return enu_origin_.initialized; }

}  // namespace automap_pro
