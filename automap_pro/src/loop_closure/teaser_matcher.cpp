#include "automap_pro/loop_closure/teaser_matcher.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"

#include <rclcpp/rclcpp.hpp>
#include <pcl/common/transforms.h>
#include <random>

#ifdef USE_TEASER
#include <teaser/registration.h>
#include <teaser/fpfh.h>
#endif

namespace automap_pro {

TeaserMatcher::TeaserMatcher() {
    const auto& cfg = ConfigManager::instance();
    voxel_size_       = cfg.teaserVoxelSize();
    noise_bound_      = cfg.teaserNoiseBound();
    min_inlier_ratio_ = cfg.teaserMinInlierRatio();
    max_rmse_         = cfg.teaserMaxRMSE();
    max_points_       = 5000;
}

CloudXYZIPtr TeaserMatcher::preprocess(const CloudXYZIPtr& cloud) const {
    auto ds = utils::voxelDownsample(cloud, voxel_size_);
    // Cap points
    if (static_cast<int>(ds->size()) > max_points_) {
        CloudXYZIPtr capped(new CloudXYZI);
        capped->reserve(max_points_);
        int step = ds->size() / max_points_;
        for (size_t i = 0; i < ds->size() && (int)capped->size() < max_points_; i += step) {
            capped->push_back(ds->points[i]);
        }
        return capped;
    }
    return ds;
}

TeaserMatcher::Result TeaserMatcher::match(const CloudXYZIPtr& src_cloud,
                                             const CloudXYZIPtr& tgt_cloud,
                                             const Pose3d& /*initial_guess*/) const {
    Result result;

    CloudXYZIPtr src = preprocess(src_cloud);
    CloudXYZIPtr tgt = preprocess(tgt_cloud);

    if (src->size() < 50 || tgt->size() < 50) {
        RCLCPP_WARN(rclcpp::get_logger("automap_pro"), "[TeaserMatcher] Too few points: src=%zu tgt=%zu",
                 src->size(), tgt->size());
        return result;
    }

    // Compute FPFH features
    auto src_feat = fpfh_extractor_.compute(src);
    auto tgt_feat = fpfh_extractor_.compute(tgt);

    // Find correspondences
    auto correspondences = fpfh_extractor_.findCorrespondences(src_feat, tgt_feat, true);

    result.num_correspondences = static_cast<int>(correspondences.size());
    if (correspondences.size() < 10) {
        RCLCPP_WARN(rclcpp::get_logger("automap_pro"), "[TeaserMatcher] Too few correspondences: %zu", correspondences.size());
        return result;
    }

#ifdef USE_TEASER
    // TEASER++ registration
    teaser::PointCloud src_pts, tgt_pts;
    for (const auto& [si, ti] : correspondences) {
        const auto& sp = src->points[si];
        const auto& tp = tgt->points[ti];
        src_pts.push_back({sp.x, sp.y, sp.z});
        tgt_pts.push_back({tp.x, tp.y, tp.z});
    }

    teaser::RobustRegistrationSolver::Params params;
    params.noise_bound               = noise_bound_;
    params.cbar2                     = ConfigManager::instance().teaserCbar2();
    params.rotation_gnc_factor       = ConfigManager::instance().teaserRotGNCFactor();
    params.rotation_max_iterations   = ConfigManager::instance().teaserRotMaxIter();
    params.rotation_cost_threshold   = ConfigManager::instance().teaserRotCostThresh();

    // TEASER++ solve(PointCloud, PointCloud, correspondences): indices into the two point clouds
    std::vector<std::pair<int, int>> corr;
    corr.reserve(src_pts.size());
    for (size_t i = 0; i < src_pts.size(); ++i)
        corr.emplace_back(static_cast<int>(i), static_cast<int>(i));
    teaser::RobustRegistrationSolver solver(params);
    solver.solve(src_pts, tgt_pts, corr);
    auto solution = solver.getSolution();

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = solution.rotation;
    T.block<3,1>(0,3) = solution.translation;
    result.T_tgt_src = utils::matrix4dToIsometry3d(T);

    const auto& inliers = solver.getTranslationInliersMask();
    result.num_inliers   = std::count(inliers.begin(), inliers.end(), true);
    result.inlier_ratio  = static_cast<double>(result.num_inliers) / correspondences.size();

#else
    // Fallback: RANSAC-based SVD
    result = matchRANSAC(src, tgt, correspondences);
#endif

    // Compute RMSE
    result.rmse = computeRMSE(src, tgt, result.T_tgt_src);

    result.success = (result.inlier_ratio >= min_inlier_ratio_) &&
                     (result.rmse <= max_rmse_);

    return result;
}

TeaserMatcher::Result TeaserMatcher::matchRANSAC(
        const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
        const std::vector<std::pair<int,int>>& correspondences) const {
    Result result;
    int n = static_cast<int>(correspondences.size());
    if (n < 4) return result;

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(0, n - 1);

    Pose3d best_T;
    int    best_inliers = 0;

    auto computeTransform = [&](const std::vector<int>& sample_idx) -> Pose3d {
        // SVD-based point-to-point rigid transform from 3 random correspondences
        Eigen::MatrixXd src_pts(3, sample_idx.size());
        Eigen::MatrixXd tgt_pts(3, sample_idx.size());
        for (size_t k = 0; k < sample_idx.size(); ++k) {
            int ci = sample_idx[k];
            const auto& sp = src->points[correspondences[ci].first];
            const auto& tp = tgt->points[correspondences[ci].second];
            src_pts.col(k) = Eigen::Vector3d(sp.x, sp.y, sp.z);
            tgt_pts.col(k) = Eigen::Vector3d(tp.x, tp.y, tp.z);
        }
        Eigen::Vector3d sc = src_pts.rowwise().mean();
        Eigen::Vector3d tc = tgt_pts.rowwise().mean();
        Eigen::MatrixXd A = (src_pts.colwise() - sc);
        Eigen::MatrixXd B = (tgt_pts.colwise() - tc);
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(B * A.transpose(),
                                               Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
        if (R.determinant() < 0) {
            Eigen::Matrix3d V = svd.matrixV();
            V.col(2) *= -1.0;
            R = svd.matrixU() * V.transpose();
        }
        Pose3d T;
        T.linear()      = R;
        T.translation() = tc - R * sc;
        return T;
    };

    for (int iter = 0; iter < ransac_max_iter_; ++iter) {
        // Sample 3 correspondences
        std::vector<int> sample;
        for (int k = 0; k < 3; ++k) {
            int idx;
            do { idx = dist(rng); } while (std::find(sample.begin(), sample.end(), idx) != sample.end());
            sample.push_back(idx);
        }

        Pose3d T = computeTransform(sample);

        // Count inliers
        int inliers = 0;
        for (int ci = 0; ci < n; ++ci) {
            const auto& sp = src->points[correspondences[ci].first];
            const auto& tp = tgt->points[correspondences[ci].second];
            Eigen::Vector3d sp_w = T * Eigen::Vector3d(sp.x, sp.y, sp.z);
            Eigen::Vector3d tp_v(tp.x, tp.y, tp.z);
            if ((sp_w - tp_v).norm() <= ransac_inlier_dist_) inliers++;
        }

        if (inliers > best_inliers) {
            best_inliers = inliers;
            best_T = T;
        }

        // Early exit if enough inliers found
        if (static_cast<double>(best_inliers) / n >= 0.7) break;
    }

    result.T_tgt_src     = best_T;
    result.num_inliers   = best_inliers;
    result.inlier_ratio  = static_cast<double>(best_inliers) / n;
    result.success       = false;  // set by caller after RMSE check

    return result;
}

double TeaserMatcher::computeRMSE(const CloudXYZIPtr& src,
                                    const CloudXYZIPtr& tgt,
                                    const Pose3d& T) const {
    auto src_t = utils::transformCloud(src, T);
    if (src_t->empty() || tgt->empty()) return 1e9;

    // Build KD-tree on target
    pcl::search::KdTree<PointXYZI> kdtree;
    kdtree.setInputCloud(tgt);

    double sum_sq = 0.0;
    int cnt = 0;
    for (const auto& pt : src_t->points) {
        std::vector<int> idx(1);
        std::vector<float> dist(1);
        if (kdtree.nearestKSearch(pt, 1, idx, dist) > 0) {
            sum_sq += dist[0];
            cnt++;
        }
    }
    return cnt > 0 ? std::sqrt(sum_sq / cnt) : 1e9;
}

}  // namespace automap_pro
