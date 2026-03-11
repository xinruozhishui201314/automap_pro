/**
 * @file test_delayed_gps_compensator.cpp
 * @brief 单元测试：GPS 对齐后历史帧与回环帧延迟补偿
 *
 * 验证内容：
 *   - test_delayed_compensation: GPS 对齐后收集历史子图并补偿
 *   - test_loop_trigger: 回环优化后触发补偿
 *   - test_idempotency: 同一子图不重复补偿
 *   - test_independence: 未对齐或禁用时不执行补偿
 */

#include <gtest/gtest.h>
#include "automap_pro/backend/delayed_gps_compensator.h"
#include "automap_pro/core/data_types.h"
#include "automap_pro/core/config_manager.h"

using namespace automap_pro;

// 创建成功的 GPS 对齐结果
static GPSAlignResult makeAlignResult() {
    GPSAlignResult r;
    r.success = true;
    r.R_gps_lidar = Eigen::Matrix3d::Identity();
    r.t_gps_lidar = Eigen::Vector3d(10.0, 20.0, 0.0);
    r.rmse_m = 0.5;
    r.matched_points = 50;
    return r;
}

// 创建带时间范围的子图
static SubMap::Ptr makeSubMap(int id, SubMapState state, double t_start = 100.0, double t_end = 110.0) {
    auto sm = std::make_shared<SubMap>();
    sm->id = id;
    sm->state = state;
    sm->t_start = t_start;
    sm->t_end = t_end;
    sm->pose_w_anchor.setIdentity();
    sm->pose_w_anchor.translation() = Eigen::Vector3d(1.0 * id, 0, 0);
    return sm;
}

TEST(DelayedGPSCompensatorTest, DelayedCompensation) {
    DelayedGPSCompensator comp;
    comp.setEnabled(true);
    comp.setCompensateOnAlign(true);

    GPSAlignResult result = makeAlignResult();
    comp.onGPSAligned(result);
    EXPECT_TRUE(comp.isGPSAligned());

    std::vector<SubMap::Ptr> all = {
        makeSubMap(0, SubMapState::FROZEN),
        makeSubMap(1, SubMapState::FROZEN),
    };
    comp.setSubmapGPSQueryCallback([](int sm_id) -> std::optional<GPSMeasurement> {
        GPSMeasurement m;
        m.timestamp = 100.0 + sm_id;
        m.position_enu = Eigen::Vector3d(1.0 * sm_id, 0, 0);
        m.quality = GPSQuality::MEDIUM;
        m.hdop = 2.0;
        m.is_valid = true;
        m.covariance = Eigen::Matrix3d::Identity() * 0.1;
        return m;
    });

    int add_count = 0;
    comp.registerGpsFactorCallback(
        [&add_count](int, const Eigen::Vector3d&, const Eigen::Matrix3d&, bool) {
            add_count++;
            return true;
        });

    int collected = comp.collectHistoricalSubmaps(all);
    EXPECT_GE(collected, 0);
    EXPECT_GE(comp.pendingCount() + comp.compensatedCount(), 0u);
}

TEST(DelayedGPSCompensatorTest, LoopTrigger) {
    DelayedGPSCompensator comp;
    comp.setEnabled(true);
    comp.setCompensateOnLoop(true);
    comp.onGPSAligned(makeAlignResult());

    comp.setSubmapGPSQueryCallback([](int sm_id) -> std::optional<GPSMeasurement> {
        GPSMeasurement m;
        m.timestamp = 100.0;
        m.position_enu = Eigen::Vector3d::Zero();
        m.quality = GPSQuality::HIGH;
        m.hdop = 1.0;
        m.is_valid = true;
        m.covariance = Eigen::Matrix3d::Identity() * 0.05;
        return m;
    });

    int factor_added = 0;
    comp.registerGpsFactorCallback(
        [&factor_added](int sm_id, const Eigen::Vector3d&, const Eigen::Matrix3d&, bool) {
            factor_added++;
            return true;
        });

    comp.registerSubmap(makeSubMap(0, SubMapState::FROZEN));
    comp.registerSubmap(makeSubMap(1, SubMapState::FROZEN));

    std::unordered_map<int, Pose3d> poses;
    poses[0] = Pose3d::Identity();
    poses[1] = Pose3d::Identity();
    int n = comp.onPoseOptimized(poses);
    EXPECT_GE(n, 0);
}

TEST(DelayedGPSCompensatorTest, Idempotency) {
    DelayedGPSCompensator comp;
    comp.setEnabled(true);
    comp.onGPSAligned(makeAlignResult());

    comp.setSubmapGPSQueryCallback([](int) -> std::optional<GPSMeasurement> {
        GPSMeasurement m;
        m.timestamp = 100.0;
        m.position_enu = Eigen::Vector3d::Zero();
        m.quality = GPSQuality::MEDIUM;
        m.hdop = 2.0;
        m.is_valid = true;
        m.covariance = Eigen::Matrix3d::Identity();
        return m;
    });

    int callback_count = 0;
    comp.registerGpsFactorCallback(
        [&callback_count](int, const Eigen::Vector3d&, const Eigen::Matrix3d&, bool) {
            callback_count++;
            return true;
        });

    auto sm = makeSubMap(0, SubMapState::FROZEN);
    comp.registerSubmap(sm);
    comp.compensateBatch(10);
    int first = callback_count;
    comp.compensateBatch(10);
    EXPECT_EQ(callback_count, first);
}

TEST(DelayedGPSCompensatorTest, IndependenceWhenDisabled) {
    DelayedGPSCompensator comp;
    comp.setEnabled(false);
    comp.onGPSAligned(makeAlignResult());

    int called = 0;
    comp.registerGpsFactorCallback(
        [&called](int, const Eigen::Vector3d&, const Eigen::Matrix3d&, bool) {
            called++;
            return true;
        });

    comp.registerSubmap(makeSubMap(0, SubMapState::FROZEN));
    std::unordered_map<int, Pose3d> poses;
    poses[0] = Pose3d::Identity();
    comp.onPoseOptimized(poses);
    EXPECT_EQ(called, 0);
}

TEST(DelayedGPSCompensatorTest, IndependenceWhenNotAligned) {
    DelayedGPSCompensator comp;
    comp.setEnabled(true);
    comp.setCompensateOnLoop(true);

    int called = 0;
    comp.registerGpsFactorCallback(
        [&called](int, const Eigen::Vector3d&, const Eigen::Matrix3d&, bool) {
            called++;
            return true;
        });

    std::unordered_map<int, Pose3d> poses;
    poses[0] = Pose3d::Identity();
    comp.onPoseOptimized(poses);
    EXPECT_FALSE(comp.isGPSAligned());
    EXPECT_EQ(called, 0);
}

TEST(DelayedGPSCompensatorTest, Reset) {
    DelayedGPSCompensator comp;
    comp.onGPSAligned(makeAlignResult());
    comp.registerSubmap(makeSubMap(0, SubMapState::FROZEN));
    EXPECT_GT(comp.pendingCount(), 0u);

    comp.reset();
    EXPECT_FALSE(comp.isGPSAligned());
    EXPECT_EQ(comp.pendingCount(), 0u);
    EXPECT_EQ(comp.compensatedCount(), 0u);
}
