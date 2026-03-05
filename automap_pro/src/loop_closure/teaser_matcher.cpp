#include "automap_pro/loop_closure/teaser_matcher.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#define MOD "TeaserMatcher"
#include "automap_pro/loop_closure/fpfh_extractor.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/SVD>

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

        // ✅ 修复：在 solver 作用域内计算 RMSE（避免悬空引用）
        {
            double sq_err = 0.0;
            int cnt = 0;
            auto inlier_map = solver.getTranslationInliersMap();
            for (Eigen::Index c = 0; c < inlier_map.cols(); ++c) {
                int idx = inlier_map(0, c);
                if (idx < 0 || idx >= static_cast<int>(corrs.size())) continue;
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
    }
#else
    // 无 TEASER++ 时回退到 Umeyama/SVD 相似变换（鲁棒性低于 TEASER，但可运行）
    Eigen::Matrix3Xd src_pts(3, corrs.size()), tgt_pts(3, corrs.size());
    for (size_t i = 0; i < corrs.size(); ++i) {
        const auto& sp = src->points[corrs[i].first];
        const auto& tp = tgt->points[corrs[i].second];
        src_pts.col(i) << sp.x, sp.y, sp.z;
        tgt_pts.col(i) << tp.x, tp.y, tp.z;
    }
    Eigen::Vector3d src_centroid = src_pts.rowwise().mean();
    Eigen::Vector3d tgt_centroid = tgt_pts.rowwise().mean();
    Eigen::Matrix3Xd src_c = src_pts.colwise() - src_centroid;
    Eigen::Matrix3Xd tgt_c = tgt_pts.colwise() - tgt_centroid;
    Eigen::Matrix3d H = src_c * tgt_c.transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
    if (R.determinant() < 0) {
        Eigen::Matrix3d V = svd.matrixV();
        V.col(2) *= -1;
        R = V * svd.matrixU().transpose();
    }
    Eigen::Vector3d t = tgt_centroid - R * src_centroid;
    result.T_tgt_src = Pose3d::Identity();
    result.T_tgt_src.linear() = R;
    result.T_tgt_src.translation() = t;
    double sq_err = 0;
    int cnt = 0;
    for (size_t i = 0; i < corrs.size(); ++i) {
        Eigen::Vector3d pred = R * src_pts.col(i) + t;
        sq_err += (pred - tgt_pts.col(i)).squaredNorm();
        cnt++;
    }
    result.rmse = cnt > 0 ? static_cast<float>(std::sqrt(sq_err / cnt)) : 1e6f;
    result.inlier_ratio = 1.0f;  // SVD 无内点筛选，保守视为全部内点
    result.success = (result.rmse < max_rmse_);
    ALOG_DEBUG(MOD, "SVD fallback: rmse={:.3f}m success={}", result.rmse, result.success);
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
