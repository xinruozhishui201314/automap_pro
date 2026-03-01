#include <gtest/gtest.h>
#include <cmath>

#include "automap_pro/core/data_types.h"
#include "automap_pro/loop_closure/fpfh_extractor.h"
#include "automap_pro/loop_closure/overlap_transformer.h"
#include "automap_pro/loop_closure/icp_refiner.h"

using namespace automap_pro;

static CloudXYZIPtr makeSphereCloud(int n = 500, float radius = 5.0f) {
    CloudXYZIPtr cloud(new CloudXYZI);
    for (int i = 0; i < n; ++i) {
        float theta = static_cast<float>(i) / n * 2.0f * M_PI;
        float phi   = std::acos(1.0f - 2.0f * i / n);
        PointXYZI pt;
        pt.x = radius * std::sin(phi) * std::cos(theta);
        pt.y = radius * std::sin(phi) * std::sin(theta);
        pt.z = radius * std::cos(phi);
        pt.intensity = 1.0f;
        cloud->push_back(pt);
    }
    return cloud;
}

TEST(FPFHExtractorTest, ComputeDescriptor) {
    FPFHExtractor fpfh;
    auto cloud = makeSphereCloud();
    auto feat  = fpfh.compute(cloud);
    ASSERT_NE(feat, nullptr);
    EXPECT_EQ(feat->size(), cloud->size());
}

TEST(FPFHExtractorTest, FindCorrespondences) {
    FPFHExtractor fpfh;
    auto cloud1 = makeSphereCloud(200);
    auto cloud2 = makeSphereCloud(200);
    auto f1 = fpfh.compute(cloud1);
    auto f2 = fpfh.compute(cloud2);
    auto corr = fpfh.findCorrespondences(f1, f2, true);
    EXPECT_GT(corr.size(), 0u);
}

TEST(OverlapTransformerTest, DescriptorNormalized) {
    OverlapTransformer ot;
    auto cloud = makeSphereCloud(1000);
    auto desc  = ot.computeDescriptor(cloud);
    EXPECT_EQ(desc.size(), ot.isModelLoaded() ? 256 : 256);
    EXPECT_NEAR(desc.norm(), 1.0f, 0.01f);
}

TEST(OverlapTransformerTest, RangeImageShape) {
    OverlapTransformer ot;
    auto cloud = makeSphereCloud(2000, 20.0f);
    auto img   = ot.generateRangeImage(cloud);
    EXPECT_FALSE(img.empty());
}

TEST(OverlapTransformerTest, RetrieveTopK) {
    OverlapTransformer ot;
    std::vector<SubMap::Ptr> db;
    for (int i = 0; i < 10; ++i) {
        auto sm = std::make_shared<SubMap>();
        sm->id   = i;
        sm->downsampled_cloud = makeSphereCloud(500, 5.0f + i);
        sm->overlap_descriptor = ot.computeDescriptor(sm->downsampled_cloud);
        sm->has_descriptor = true;
        db.push_back(sm);
    }

    auto query = ot.computeDescriptor(db[0]->downsampled_cloud);
    auto cands = ot.retrieve(query, db, 3, 0.0);
    EXPECT_LE(cands.size(), 3u);
    if (!cands.empty()) {
        EXPECT_EQ(cands[0].submap_id, 0);  // exact match first
    }
}

TEST(ICPRefinerTest, ConvergesOnIdentity) {
    ICPRefiner icp;
    auto cloud = makeSphereCloud(300);
    // Source = target with tiny perturbation
    auto tgt = cloud;
    Pose3d init; init.setIdentity();
    init.translation() = Eigen::Vector3d(0.05, 0.02, 0.0);
    auto result = icp.refine(cloud, tgt, init);
    EXPECT_TRUE(result.converged);
    EXPECT_LT(result.rmse, 0.1);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
