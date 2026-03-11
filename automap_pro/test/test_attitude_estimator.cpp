#include <gtest/gtest.h>
#include <cmath>
#include "automap_pro/sensor/attitude_estimator.h"
#include "automap_pro/core/data_types.h"

using namespace automap_pro;

static constexpr double kPi = 3.14159265358979323846;

TEST(AttitudeEstimatorTest, PitchRollFromGravity) {
    AttitudeEstimator::Config cfg;
    cfg.gravity_norm = 9.81;
    AttitudeEstimator est(cfg);
    // 静止时加速度计测量重力：body 系下 (0, 0, -9.81) 表示水平
    Eigen::Vector3d acc(0.0, 0.0, -9.81);
    Eigen::Vector3d gyr(0.0, 0.0, 0.0);
    for (int i = 0; i < 10; ++i) {
        est.addIMU(static_cast<double>(i) * 0.01, acc, gyr);
    }
    AttitudeEstimate out = est.estimateAt(0.05);
    EXPECT_NEAR(out.pitch, 0.0, 0.05);
    EXPECT_NEAR(out.roll, 0.0, 0.05);
    EXPECT_TRUE(out.variance(0) < 0.5 && out.variance(1) < 0.5);
}

TEST(AttitudeEstimatorTest, YawFromGPSTrajectory) {
    AttitudeEstimator::Config cfg;
    cfg.min_velocity_for_yaw = 0.5;
    AttitudeEstimator est(cfg);
    est.addGPSPos(0.0, Eigen::Vector3d(0, 0, 0));
    est.addGPSPos(1.0, Eigen::Vector3d(10, 0, 0));  // 向东 10m/s
    est.addIMU(0.5, Eigen::Vector3d(0, 0, -9.81), Eigen::Vector3d::Zero());
    AttitudeEstimate out = est.estimateAt(0.5);
    // 航迹角 atan2(0, 10) = 0 (东)
    EXPECT_NEAR(out.yaw, 0.0, 0.2);
    EXPECT_GT(out.velocity_horizontal, 0.5);
}

TEST(AttitudeEstimatorTest, OdomYawFallback) {
    AttitudeEstimator::Config cfg;
    cfg.min_velocity_for_yaw = 10.0;  // 很高，GPS 航迹角不会触发
    AttitudeEstimator est(cfg);
    est.addGPSPos(0.0, Eigen::Vector3d(0, 0, 0));
    est.addGPSPos(1.0, Eigen::Vector3d(0.1, 0, 0));  // 几乎静止
    est.addOdometryYaw(0.5, kPi / 4.0);  // 45°
    est.addIMU(0.5, Eigen::Vector3d(0, 0, -9.81), Eigen::Vector3d::Zero());
    AttitudeEstimate out = est.estimateAt(0.5);
    EXPECT_NEAR(std::abs(out.yaw), kPi / 4.0, 0.15);
}

TEST(AttitudeEstimatorTest, GPSAttitudePreferred) {
    AttitudeEstimator::Config cfg;
    cfg.use_gps_attitude_when_available = true;  // 启用时才会使用 addGPSAttitude 注入的姿态
    AttitudeEstimator est(cfg);
    est.addIMU(0.0, Eigen::Vector3d(0, 0, -9.81), Eigen::Vector3d::Zero());
    est.addGPSAttitude(0.0, 0.1, 0.05, 1.0, 0.9);  // roll, pitch, yaw, confidence
    AttitudeEstimate out = est.estimateAt(0.0);
    EXPECT_EQ(out.source, AttitudeSource::GPS_DUAL_ANTENNA);
    EXPECT_NEAR(out.roll, 0.1, 0.01);
    EXPECT_NEAR(out.pitch, 0.05, 0.01);
    EXPECT_NEAR(out.yaw, 1.0, 0.01);
    EXPECT_TRUE(out.is_valid);
}

TEST(AttitudeEstimatorTest, PitchRollTilted) {
    AttitudeEstimator::Config cfg;
    cfg.gravity_norm = 9.81;
    AttitudeEstimator est(cfg);
    // 绕 Y 轴倾斜 10°：acc 在 XZ 平面，pitch ≈ 10° ≈ 0.174 rad
    double p = 10.0 * kPi / 180.0;
    Eigen::Vector3d acc(-9.81 * std::sin(p), 0.0, -9.81 * std::cos(p));
    for (int i = 0; i < 20; ++i) {
        est.addIMU(static_cast<double>(i) * 0.01, acc, Eigen::Vector3d::Zero());
    }
    AttitudeEstimate out = est.estimateAt(0.1);
    EXPECT_NEAR(out.pitch, p, 0.08);
    EXPECT_NEAR(out.roll, 0.0, 0.08);
}
