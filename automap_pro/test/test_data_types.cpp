#include <gtest/gtest.h>
#include "automap_pro/core/data_types.h"
#include "automap_pro/core/utils.h"

using namespace automap_pro;

TEST(DataTypesTest, KeyFrameCreation) {
    KeyFrame kf;
    kf.id         = 42;
    kf.session_id = 1;
    kf.timestamp  = 1234.5;
    kf.T_w_b.setIdentity();
    EXPECT_EQ(kf.id, 42u);
    EXPECT_DOUBLE_EQ(kf.timestamp, 1234.5);
}

TEST(DataTypesTest, SubMapAddKeyFrame) {
    SubMap sm;
    sm.id = 0;
    sm.session_id = 0;
    sm.merged_cloud = std::make_shared<CloudXYZI>();
    sm.downsampled_cloud = std::make_shared<CloudXYZI>();

    auto kf1 = std::make_shared<KeyFrame>();
    kf1->id = 0; kf1->timestamp = 0.1;
    kf1->T_w_b.setIdentity();
    kf1->cloud_ds_body = std::make_shared<CloudXYZI>();

    auto kf2 = std::make_shared<KeyFrame>();
    kf2->id = 1; kf2->timestamp = 1.1;
    kf2->T_w_b = utils::vec6dToPose(Vec6d(2,0,0,0,0,0));
    kf2->cloud_ds_body = std::make_shared<CloudXYZI>();

    sm.addKeyFrame(kf1);
    sm.addKeyFrame(kf2);

    EXPECT_EQ(sm.keyframes.size(), 2u);
    EXPECT_DOUBLE_EQ(sm.t_start, 0.1);
    EXPECT_DOUBLE_EQ(sm.t_end,   1.1);
    EXPECT_NEAR(sm.spatial_extent, 2.0, 0.01);
}

TEST(DataTypesTest, SubMapIsFull) {
    SubMap sm;
    sm.id = 0; sm.merged_cloud = std::make_shared<CloudXYZI>();
    sm.downsampled_cloud = std::make_shared<CloudXYZI>();

    for (int i = 0; i < 5; ++i) {
        auto kf = std::make_shared<KeyFrame>();
        kf->id = i; kf->timestamp = i;
        kf->T_w_b.setIdentity();
        kf->cloud_ds_body = std::make_shared<CloudXYZI>();
        sm.addKeyFrame(kf);
    }

    EXPECT_FALSE(sm.isFull(10, 200.0, 600.0));
    EXPECT_TRUE (sm.isFull(5,  200.0, 600.0));   // max keyframes exceeded
}

TEST(DataTypesTest, GPSQualityStrings) {
    EXPECT_EQ(gpsQualityToString(GPSQuality::EXCELLENT), "EXCELLENT");
    EXPECT_EQ(gpsQualityToString(GPSQuality::INVALID),   "INVALID");
    EXPECT_EQ(gpsStateToString(GPSState::TRACKING),      "TRACKING");
    EXPECT_EQ(subMapStateToString(SubMapState::FROZEN),   "FROZEN");
}

TEST(DataTypesTest, PoseConversions) {
    Pose3d T;
    T.setIdentity();
    T.translation() = Eigen::Vector3d(1, 2, 3);

    Vec6d v = utils::poseToVec6d(T);
    EXPECT_NEAR(v(0), 1.0, 1e-9);
    EXPECT_NEAR(v(1), 2.0, 1e-9);
    EXPECT_NEAR(v(2), 3.0, 1e-9);

    Pose3d T2 = utils::vec6dToPose(v);
    EXPECT_NEAR(T2.translation().x(), 1.0, 1e-9);
    EXPECT_NEAR(T2.translation().y(), 2.0, 1e-9);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
