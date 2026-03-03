#include "automap_pro/loop_closure/teaser_matcher.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#define MOD "TeaserMatcher"
#include "automap_pro/loop_closure/fpfh_extractor.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace automap_pro {

TeaserMatcher::TeaserMatcher() {
    const auto& cfg = ConfigManager::instance();
    noise_bound_      = cfg.teaserNoiseBound();
    cbar2_            = cfg.teaserCbar2();
    voxel_size_       = cfg.teaserVoxelSize();
    min_inlier_ratio_ = cfg.teaserMinInlierRatio();
    max_rmse_         = cfg.teaserMaxRMSE();
}

CloudXYZIPtr TeaserMatcher::preprocess(const CloudXYZIPtr& cloud) const {
    CloudXYZIPtr ds(new CloudXYZI);
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    vg.filter(*ds);

    if ((int)ds->size() > max_points_) {
        CloudXYZIPtr capped(new CloudXYZI);
        capped->reserve(max_points_);
        int step = ds->size() / max_points_;
        for (size_t i = 0; i < ds->size() && (int)capped->size() < max_points_; i += step)
            capped->push_back(ds->points[i]);
        return capped;
    }
    return ds;
}

TeaserMatcher::Result TeaserMatcher::match(
    const CloudXYZIPtr& src_cloud,
    const CloudXYZIPtr& tgt_cloud,
    const Pose3d& /*initial_guess*/) const
{
    Result result;
    auto src = preprocess(src_cloud);
    auto tgt = preprocess(tgt_cloud);

    if (src->size() < 30 || tgt->size() < 30) return result;

    // FPFH 特征提取
    FpfhExtractor extractor;
    auto src_feat = extractor.compute(src);
    auto tgt_feat = extractor.compute(tgt);

    // 互近邻对应
    auto corrs = extractor.findCorrespondences(src_feat, tgt_feat, true);
    result.num_correspondences = (int)corrs.size();
    if ((int)corrs.size() < 10) return result;

#ifdef USE_TEASER
    teaser::PointCloud src_pts, tgt_pts;
    std::vector<std::pair<int, int>> teaser_corrs;
    for (size_t i = 0; i < corrs.size(); ++i) {
        const auto& [si, ti] = corrs[i];
        const auto& sp = src->points[si];
        const auto& tp = tgt->points[ti];
        src_pts.push_back({sp.x, sp.y, sp.z});
        tgt_pts.push_back({tp.x, tp.y, tp.z});
        teaser_corrs.push_back({static_cast<int>(i), static_cast<int>(i)});
    }

    teaser::RobustRegistrationSolver::Params params;
    params.noise_bound               = noise_bound_;
    params.cbar2                     = cbar2_;
    params.estimate_scaling          = false;   // 同一传感器，不估计尺度（提速）
    params.rotation_estimation_algorithm =
        teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.rotation_gnc_factor       = 1.4;
    params.rotation_max_iterations   = 100;
    params.rotation_cost_threshold   = 1e-6;
    params.inlier_selection_mode     =
        teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_HEU;  // 更快
    params.max_clique_time_limit     = 10.0;  // 限制时间

    {
        AUTOMAP_TIMED_SCOPE(MOD, fmt::format("TEASER solve corr={}", corrs.size()), 3000.0);
        teaser::RobustRegistrationSolver solver(params);
        solver.solve(src_pts, tgt_pts, teaser_corrs);

        auto solution = solver.getSolution();
        if (!solution.valid) {
            ALOG_WARN(MOD, "TEASER solution invalid (corr={})", corrs.size());
            return result;
        }

        // 计算内点率
        const auto& inlier_mask = solver.getInlierMaxClique();
        int inliers = 0;
        for (bool b : inlier_mask) if (b) inliers++;
        result.inlier_ratio = (float)inliers / (float)corrs.size();
        ALOG_DEBUG(MOD, "TEASER: inlier_ratio={:.2f} ({}/{}) thresh={}",
                   result.inlier_ratio, inliers, corrs.size(), min_inlier_ratio_);

        if (result.inlier_ratio < min_inlier_ratio_) return result;

        result.T_tgt_src = Pose3d::Identity();
        result.T_tgt_src.linear()      = solution.rotation;
        result.T_tgt_src.translation() = solution.translation;
    }

    // 计算 RMSE（内点，在 block 外使用 result 中已存的数据）
    {
        double sq_err = 0.0;
        int cnt = 0;
        for (size_t idx = 0; idx < corrs.size(); ++idx) {
            const auto& sp = src->points[corrs[idx].first];
            const auto& tp = tgt->points[corrs[idx].second];
            Eigen::Vector3d pred = result.T_tgt_src.linear() *
                Eigen::Vector3d(sp.x, sp.y, sp.z) + result.T_tgt_src.translation();
            Eigen::Vector3d actual(tp.x, tp.y, tp.z);
            sq_err += (pred - actual).squaredNorm();
            cnt++;
        }
        result.rmse    = cnt > 0 ? static_cast<float>(std::sqrt(sq_err / cnt)) : 1e6f;
        result.success = (result.rmse < max_rmse_);
    }
#else
    // 无 TEASER++ 时回退到 SVD（仅参考用，鲁棒性低）
    (void)src_pts; (void)tgt_pts;
    result.success = false;
#endif

    if (result.success) {
        ALOG_INFO(MOD, "TEASER match OK: inlier={:.2f} rmse={:.3f}m t=[{:.2f},{:.2f},{:.2f}]",
                  result.inlier_ratio, result.rmse,
                  result.T_tgt_src.translation().x(),
                  result.T_tgt_src.translation().y(),
                  result.T_tgt_src.translation().z());
    } else {
        ALOG_DEBUG(MOD, "TEASER match rejected: inlier={:.2f} rmse={:.3f}m",
                   result.inlier_ratio, result.rmse);
    }
    return result;
}

} // namespace automap_pro
