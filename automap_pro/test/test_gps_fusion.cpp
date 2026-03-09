#include <gtest/gtest.h>
#include <ros/ros.h>

#include "automap_pro/core/data_types.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/frontend/gps_fusion.h"
#include "automap_pro/core/utils.h"

using namespace automap_pro;

TEST(GPSFusionTest, QualityAssessmentValid) {
    GPSFusion fusion;

    GPSMeasurement meas;
    meas.quality    = GPSQuality::EXCELLENT;
    meas.is_valid   = true;
    meas.is_outlier = false;
    meas.position_enu = Eigen::Vector3d(0, 0, 0);
    meas.covariance   = Eigen::Matrix3d::Identity() * 0.01;

    Pose3d pose;
    pose.setIdentity();
    Eigen::Matrix3d odom_cov = Eigen::Matrix3d::Identity() * 0.01;

    auto obs = fusion.processForFrontend(meas, pose, odom_cov);
    EXPECT_TRUE(obs.should_use);
    EXPECT_EQ(obs.quality, GPSQuality::EXCELLENT);
}

TEST(GPSFusionTest, InvalidGPSNotUsed) {
    GPSFusion fusion;

    GPSMeasurement meas;
    meas.quality  = GPSQuality::INVALID;
    meas.is_valid = false;

    Pose3d pose; pose.setIdentity();
    auto obs = fusion.processForFrontend(meas, pose, Eigen::Matrix3d::Identity());
    EXPECT_FALSE(obs.should_use);
}

TEST(GPSFusionTest, OutlierNotUsed) {
    GPSFusion fusion;

    GPSMeasurement meas;
    meas.quality    = GPSQuality::HIGH;
    meas.is_valid   = true;
    meas.is_outlier = true;

    Pose3d pose; pose.setIdentity();
    auto obs = fusion.processForFrontend(meas, pose, Eigen::Matrix3d::Identity());
    EXPECT_FALSE(obs.should_use);
}

TEST(GPSFusionTest, CovarianceBackend) {
    GPSFusion fusion;

    GPSMeasurement meas_ex;
    meas_ex.quality = GPSQuality::EXCELLENT;
    auto cov_ex = fusion.covarianceForBackend(meas_ex);
    EXPECT_NEAR(cov_ex(0,0), 0.05*0.05, 1e-6);

    GPSMeasurement meas_lo;
    meas_lo.quality = GPSQuality::LOW;
    auto cov_lo = fusion.covarianceForBackend(meas_lo);
    EXPECT_NEAR(cov_lo(0,0), 10.0*10.0, 1e-4);
}

TEST(GPSFusionTest, StateTransition) {
    GPSFusion fusion;
    EXPECT_EQ(fusion.currentState(), GPSState::INIT);
    fusion.setState(GPSState::TRACKING);
    EXPECT_EQ(fusion.currentState(), GPSState::TRACKING);
}

TEST(UtilsTest, WGS84ToENU) {
    double e, n, u;
    utils::wgs84ToENU(37.7749, -122.4194, 10.0,
                      37.7749, -122.4194, 10.0,
                      e, n, u);
    EXPECT_NEAR(e, 0.0, 0.1);
    EXPECT_NEAR(n, 0.0, 0.1);
    EXPECT_NEAR(u, 0.0, 0.1);
}

TEST(UtilsTest, VoxelDownsampleReducesPoints) {
    CloudXYZIPtr cloud(new CloudXYZI);
    for (int i = 0; i < 1000; ++i) {
        PointXYZI pt;
        pt.x = static_cast<float>(i) * 0.01f;
        pt.y = 0; pt.z = 0; pt.intensity = 1.0;
        cloud->push_back(pt);
    }
    auto ds = utils::voxelDownsample(cloud, 0.5);
    EXPECT_LT(ds->size(), cloud->size());
    EXPECT_GT(ds->size(), 0u);
}

TEST(PoseGraphTest, AddAndRetrieveNodes) {
    PoseGraph graph;
    Pose3d T; T.setIdentity();
    graph.addNode(0, NodeType::SUBMAP, T, true);
    graph.addNode(1, NodeType::SUBMAP, T, false);
    EXPECT_EQ(graph.numNodes(), 2);
    EXPECT_TRUE(graph.hasNode(0));
    EXPECT_FALSE(graph.hasNode(99));
}

TEST(PoseGraphTest, AddEdgesAndCount) {
    PoseGraph graph;
    Pose3d T; T.setIdentity();
    graph.addNode(0, NodeType::KEYFRAME, T);
    graph.addNode(1, NodeType::KEYFRAME, T);
    graph.addOdomEdge(0, 1, T, Mat66d::Identity());
    graph.addLoopEdge(0, 1, T, Mat66d::Identity());
    EXPECT_EQ(graph.numEdges(),    2);
    EXPECT_EQ(graph.numLoopEdges(), 1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_gps_fusion");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
