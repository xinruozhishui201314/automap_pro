#include <gtest/gtest.h>

#include "automap_pro/core/data_types.h"
#include "automap_pro/v3/pose_chain.hpp"

namespace {

using automap_pro::Pose3d;
using automap_pro::PoseFrame;
using automap_pro::PoseSnapshot;
namespace pc = automap_pro::v3::pose_chain;

TEST(V3PoseChain, MapBodyFromOdomBodyMatchesCompose) {
    Pose3d T_map_odom;
    T_map_odom.linear() = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T_map_odom.translation() << 1.0, 2.0, 3.0;
    Pose3d T_odom_b = Pose3d::Identity();
    T_odom_b.translation() << 10.0, 0.0, 0.0;
    const Pose3d out = pc::mapBodyFromOdomBody(T_map_odom, T_odom_b);
    const Pose3d expected = T_map_odom * T_odom_b;
    EXPECT_TRUE(out.matrix().isApprox(expected.matrix(), 1e-9));
}

TEST(V3PoseChain, WorldToMapFromAnchorChain) {
    Pose3d T_k_map = Pose3d::Identity();
    T_k_map.translation() << 100.0, 0.0, 0.0;
    Pose3d T_k_odom = Pose3d::Identity();
    T_k_odom.translation() << 50.0, 0.0, 0.0;
    const Pose3d T_wm = pc::worldToMapFromAnchorChain(T_k_map, T_k_odom);
    const Eigen::Vector3d p_odom(0.0, 0.0, 0.0);
    const Eigen::Vector3d p_map = T_wm * p_odom;
    EXPECT_NEAR(p_map.x(), 50.0, 1e-6);
}

TEST(V3PoseChain, ResolveTMapOdomPrefersSessionThenGlobal) {
    PoseSnapshot snap;
    snap.gps_aligned = true;
    snap.R_enu_to_map = Eigen::Matrix3d::Identity();
    snap.t_enu_to_map = Eigen::Vector3d(1.0, 0.0, 0.0);
    PoseSnapshot::SessionAlignment sa;
    sa.aligned = true;
    sa.T_map_odom.translation() << 5.0, 0.0, 0.0;
    sa.T_map_odom.linear() = Eigen::Matrix3d::Identity();
    snap.session_alignments[42] = sa;

    const Pose3d Ts = pc::resolveTMapOdomFromSnapshot(snap, 42);
    EXPECT_NEAR(Ts.translation().x(), 5.0, 1e-9);

    const Pose3d Tg = pc::resolveTMapOdomFromSnapshot(snap, 0);
    EXPECT_NEAR(Tg.translation().x(), 1.0, 1e-9);
}

TEST(V3PoseChain, KeyframeOptimizedInMapFrameElevatesOdomKfWhenGps) {
    Pose3d T_map_odom;
    T_map_odom.translation() << 2.0, 0.0, 0.0;
    T_map_odom.linear() = Eigen::Matrix3d::Identity();
    Pose3d T_stored = Pose3d::Identity();
    T_stored.translation() << 1.0, 0.0, 0.0;
    const Pose3d out =
        pc::keyframeOptimizedInMapFrame(T_map_odom, true, PoseFrame::ODOM, T_stored);
    const Pose3d expected = T_map_odom * T_stored;
    EXPECT_TRUE(out.matrix().isApprox(expected.matrix(), 1e-9));
}

TEST(V3PoseChain, ChainWeightFromDriftTranslation) {
    EXPECT_DOUBLE_EQ(pc::chainWeightFromDriftTranslation(0.0), 1.0);
    EXPECT_DOUBLE_EQ(pc::chainWeightFromDriftTranslation(0.25), 1.0);
    EXPECT_DOUBLE_EQ(pc::chainWeightFromDriftTranslation(2.0), 0.0);
    EXPECT_GT(pc::chainWeightFromDriftTranslation(1.0), 0.0);
    EXPECT_LT(pc::chainWeightFromDriftTranslation(1.0), 1.0);
}

TEST(V3PoseChain, BlendWorldToMapSE3TranslationMidpoint) {
    Pose3d A = Pose3d::Identity();
    Pose3d B = Pose3d::Identity();
    B.translation() << 10.0, 0.0, 0.0;
    const Pose3d bl = pc::blendWorldToMapSE3(A, B, 0.5);
    EXPECT_NEAR(bl.translation().x(), 5.0, 1e-6);
}

}  // namespace
