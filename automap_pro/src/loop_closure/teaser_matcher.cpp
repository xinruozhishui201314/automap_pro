#include "automap_pro/loop_closure/teaser_matcher.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/utils.h"
#define MOD "TeaserMatcher"
#include "automap_pro/loop_closure/fpfh_extractor.h"

#include <pcl/common/transforms.h>
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
    const unsigned tid = automap_pro::logThreadId();
    // 精准日志：入参指针与引用计数，便于定位 double-free / use-after-free
    ALOG_INFO(MOD, "[tid={}] step=preprocess_enter cloud_ptr={} use_count={} in_pts={}",
              tid, static_cast<const void*>(cloud.get()), cloud ? cloud.use_count() : 0, cloud ? cloud->size() : 0u);
    
    if (!cloud || cloud->empty()) {
        ALOG_DEBUG(MOD, "[tid={}] step=preprocess_skip empty or null cloud", tid);
        return std::make_shared<CloudXYZI>();
    }

    try {
        // 使用 utils::voxelDownsample 替代 PCL VoxelGrid，避免 PCL 在部分 leaf/范围下写越界导致析构时 SIGSEGV
        const float leaf = static_cast<float>(std::max(voxel_size_, static_cast<double>(utils::kMinVoxelLeafSize)));
        ALOG_DEBUG(MOD, "[tid={}] step=preprocess_voxel_enter voxel_size={:.3f} leaf={:.3f} in_pts={}",
                  tid, voxel_size_, leaf, cloud->size());
        
        CloudXYZIPtr ds = utils::voxelDownsample(cloud, leaf);
        if (!ds) {
            ALOG_WARN(MOD, "[tid={}] step=preprocess_voxel_fail voxelDownsample returned null, using empty", tid);
            ds = std::make_shared<CloudXYZI>();
        }
        
        ALOG_INFO(MOD, "[tid={}] step=preprocess_voxel_done ds_ptr={} ds_pts={} ds_use_count={} reduction_ratio={:.1f}",
                  tid, static_cast<const void*>(ds.get()), ds->size(), ds.use_count(),
                  cloud->size() > 0 ? 100.0f * ds->size() / cloud->size() : 0.0f);

        // === 裁剪至最大点数 ===
        if (static_cast<int>(ds->size()) > max_points_) {
            ALOG_DEBUG(MOD, "[tid={}] step=preprocess_capping ds_pts={} > max_points={}", tid, ds->size(), max_points_);
            CloudXYZIPtr capped = std::make_shared<CloudXYZI>();
            capped->reserve(static_cast<size_t>(max_points_));
            int step = static_cast<int>(ds->size()) / max_points_;
            
            for (size_t i = 0; i < ds->size() && static_cast<int>(capped->size()) < max_points_; i += static_cast<size_t>(step)) {
                capped->push_back(ds->points[i]);
            }
            
            ALOG_INFO(MOD, "[tid={}] step=preprocess_capped capped_ptr={} out_pts={} use_count={} step={}",
                      tid, static_cast<const void*>(capped.get()), capped->size(), capped.use_count(), step);
            return capped;
        }
        
        ALOG_INFO(MOD, "[tid={}] step=preprocess_done ds_ptr={} out_pts={} use_count={}",
                  tid, static_cast<const void*>(ds.get()), ds->size(), ds.use_count());
        return ds;
        
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "[tid={}] step=preprocess_exception in_pts={} msg={}", tid, cloud->size(), e.what());
        return std::make_shared<CloudXYZI>();
    } catch (...) {
        ALOG_ERROR(MOD, "[tid={}] step=preprocess_unknown_exception in_pts={}", tid, cloud->size());
        return std::make_shared<CloudXYZI>();
    }
}

TeaserMatcher::Result TeaserMatcher::match(
    const CloudXYZIPtr& src_cloud,
    const CloudXYZIPtr& tgt_cloud,
    const Pose3d& /*initial_guess*/) const
{
    const unsigned tid = automap_pro::logThreadId();
    ALOG_INFO(MOD, "[tid={}] step=match_enter src_ptr={} src_use_count={} src_pts={} tgt_ptr={} tgt_use_count={} tgt_pts={}",
              tid, static_cast<const void*>(src_cloud.get()), src_cloud ? src_cloud.use_count() : 0, src_cloud ? src_cloud->size() : 0u,
              static_cast<const void*>(tgt_cloud.get()), tgt_cloud ? tgt_cloud.use_count() : 0, tgt_cloud ? tgt_cloud->size() : 0u);
    Result result;
    
    try {
        // === 阶段1：预处理源点云 ===
        ALOG_DEBUG(MOD, "[tid={}] step=preprocess_src_enter src_ptr={} use_count={}", tid, static_cast<const void*>(src_cloud.get()), src_cloud ? src_cloud.use_count() : 0);
        auto src = preprocess(src_cloud);
        if (!src) {
            ALOG_WARN(MOD, "[tid={}] step=preprocess_src_fail preprocess returned null", tid);
            src = std::make_shared<CloudXYZI>();
        }
        ALOG_INFO(MOD, "[tid={}] step=preprocess_src_done src_out_ptr={} src_pts={} use_count={}",
                 tid, static_cast<const void*>(src.get()), src->size(), src.use_count());

        // === 阶段2：预处理目标点云 ===
        ALOG_DEBUG(MOD, "[tid={}] step=preprocess_tgt_enter tgt_ptr={} use_count={}", tid, static_cast<const void*>(tgt_cloud.get()), tgt_cloud ? tgt_cloud.use_count() : 0);
        auto tgt = preprocess(tgt_cloud);
        if (!tgt) {
            ALOG_WARN(MOD, "[tid={}] step=preprocess_tgt_fail preprocess returned null", tid);
            tgt = std::make_shared<CloudXYZI>();
        }
        ALOG_INFO(MOD, "[tid={}] step=preprocess_tgt_done tgt_out_ptr={} tgt_pts={} use_count={}",
                 tid, static_cast<const void*>(tgt.get()), tgt->size(), tgt.use_count());

        // === 检查最少点数要求 ===
        if (src->size() < 30 || tgt->size() < 30) {
            ALOG_WARN(MOD, "[tid={}] step=match_skip insufficient_pts src={} tgt={} (min=30)", tid, src->size(), tgt->size());
            return result;
        }

        // === 阶段3：FPFH 特征提取 ===
        ALOG_DEBUG(MOD, "[tid={}] step=fpfh_compute_start src_pts={} tgt_pts={}", tid, src->size(), tgt->size());
        FpfhExtractor extractor;
        
        ALOG_DEBUG(MOD, "[tid={}] step=fpfh_src_compute_start src_ptr={} src_pts={}", 
                  tid, static_cast<const void*>(src.get()), src->size());
        auto src_feat = extractor.compute(src, 0.5f, 1.0f);
        ALOG_INFO(MOD, "[tid={}] step=fpfh_src_done src_feat_ptr={} src_feat_pts={} src_feat_use_count={} src_pts={}",
                 tid, static_cast<const void*>(src_feat.get()), src_feat ? src_feat->size() : 0u, 
                 src_feat ? src_feat.use_count() : 0, src->size());
        
        if (!src_feat || src_feat->empty()) {
            ALOG_WARN(MOD, "[tid={}] step=fpfh_src_empty_skip src_pts={} src_feat_pts={}", 
                     tid, src->size(), src_feat ? src_feat->size() : 0u);
            return result;
        }
        
        ALOG_DEBUG(MOD, "[tid={}] step=fpfh_tgt_compute_start tgt_ptr={} tgt_pts={}", 
                  tid, static_cast<const void*>(tgt.get()), tgt->size());
        auto tgt_feat = extractor.compute(tgt, 0.5f, 1.0f);
        ALOG_INFO(MOD, "[tid={}] step=fpfh_tgt_done tgt_feat_ptr={} tgt_feat_pts={} tgt_feat_use_count={} tgt_pts={}",
                 tid, static_cast<const void*>(tgt_feat.get()), tgt_feat ? tgt_feat->size() : 0u,
                 tgt_feat ? tgt_feat.use_count() : 0, tgt->size());
        
        if (!tgt_feat || tgt_feat->empty()) {
            ALOG_WARN(MOD, "[tid={}] step=fpfh_tgt_empty_skip tgt_pts={} tgt_feat_pts={}", 
                     tid, tgt->size(), tgt_feat ? tgt_feat->size() : 0u);
            return result;
        }

        // === 阶段4：特征对应匹配 ===
        ALOG_DEBUG(MOD, "[tid={}] step=fpfh_match_start src_feat={} tgt_feat={}",
                  tid, src_feat ? src_feat->size() : 0u, tgt_feat ? tgt_feat->size() : 0u);
        auto corrs = extractor.findCorrespondences(src_feat, tgt_feat, true);
        result.num_correspondences = (int)corrs.size();
        ALOG_INFO(MOD, "[tid={}] step=fpfh_match_done correspondences={}", tid, result.num_correspondences);
        
        if ((int)corrs.size() < 10) {
            ALOG_WARN(MOD, "[tid={}] step=match_skip insufficient_corrs={} (min=10)", tid, corrs.size());
            return result;
        }

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
    ALOG_DEBUG(MOD, "[tid={}] step=teaser_prep corrs={} teaser_corrs_size={}", tid, corrs.size(), teaser_corrs.size());

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

    try {
        ALOG_DEBUG(MOD, "[tid={}] step=teaser_solve_enter corrs={} noise_bound={}", tid, corrs.size(), params.noise_bound);
        AUTOMAP_TIMED_SCOPE(MOD, fmt::format("TEASER solve corr={}", corrs.size()), 3000.0);
        teaser::RobustRegistrationSolver solver(params);
        solver.solve(src_pts, tgt_pts, teaser_corrs);
        ALOG_DEBUG(MOD, "[tid={}] step=teaser_solve_done", tid);

        auto solution = solver.getSolution();
        if (!solution.valid) {
            ALOG_WARN(MOD, "[tid={}] step=teaser_solve_invalid corr={}", tid, corrs.size());
            return result;
        }
        ALOG_DEBUG(MOD, "[tid={}] step=teaser_solution_valid", tid);

        // 计算内点率
        const auto& inlier_mask = solver.getInlierMaxClique();
        int inliers = 0;
        for (bool b : inlier_mask) if (b) inliers++;
        result.inlier_ratio = (float)inliers / (float)corrs.size();
        ALOG_INFO(MOD, "[tid={}] step=teaser_inlier_computed inliers={}/{} ratio={:.2f} thresh={}",
                 tid, inliers, corrs.size(), result.inlier_ratio, min_inlier_ratio_);

        if (result.inlier_ratio < min_inlier_ratio_) {
            ALOG_WARN(MOD, "[tid={}] step=teaser_inlier_rejected inlier_ratio={:.2f} < thresh={}", 
                     tid, result.inlier_ratio, min_inlier_ratio_);
            return result;
        }

        result.T_tgt_src = Pose3d::Identity();
        result.T_tgt_src.linear()      = solution.rotation;
        result.T_tgt_src.translation() = solution.translation;

        // ✅ 修复：在 solver 作用域内计算 RMSE（避免悬空引用）
        {
            double sq_err = 0.0;
            int cnt = 0;
            auto inlier_map = solver.getTranslationInliersMap();
            ALOG_DEBUG(MOD, "[tid={}] step=teaser_rmse_compute inlier_map_cols={}", tid, inlier_map.cols());
            
            for (Eigen::Index c = 0; c < inlier_map.cols(); ++c) {
                int idx = inlier_map(0, c);
                if (idx < 0 || idx >= static_cast<int>(corrs.size())) {
                    ALOG_DEBUG(MOD, "[tid={}] step=teaser_rmse_skip_invalid_idx idx={}", tid, idx);
                    continue;
                }
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
            ALOG_INFO(MOD, "[tid={}] step=teaser_rmse_final cnt={} rmse={:.3f}m success={}", 
                     tid, cnt, result.rmse, result.success);
        }
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "[tid={}] step=teaser_solve_exception msg={}", tid, e.what());
        return result;
    } catch (...) {
        ALOG_ERROR(MOD, "[tid={}] step=teaser_solve_unknown_exception", tid);
        return result;
    }
#else
    // 无 TEASER++ 时回退到 Umeyama/SVD 相似变换（鲁棒性低于 TEASER，但可运行）
    ALOG_DEBUG(MOD, "[tid={}] step=svd_fallback TEASER disabled, using SVD corrs={}", tid, corrs.size());
    try {
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
        ALOG_INFO(MOD, "[tid={}] step=svd_done rmse={:.3f}m success={} inlier_ratio={}", 
                 tid, result.rmse, result.success, result.inlier_ratio);
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "[tid={}] step=svd_exception msg={}", tid, e.what());
        return result;
    } catch (...) {
        ALOG_ERROR(MOD, "[tid={}] step=svd_unknown_exception", tid);
        return result;
    }
#endif

    if (result.success) {
        ALOG_INFO(MOD, "[tid={}] step=match_success inlier={:.2f} corrs={} rmse={:.3f}m translation=[{:.2f},{:.2f},{:.2f}]",
                 tid, result.inlier_ratio, result.num_correspondences, result.rmse,
                 result.T_tgt_src.translation().x(),
                 result.T_tgt_src.translation().y(),
                 result.T_tgt_src.translation().z());
    } else {
        ALOG_WARN(MOD, "[tid={}] step=match_failed inlier={:.2f} corrs={} rmse={:.3f}m",
                 tid, result.inlier_ratio, result.num_correspondences, result.rmse);
    }
    ALOG_DEBUG(MOD, "[tid={}] step=match_exit", tid);
    return result;
} catch (const std::exception& e) {
    ALOG_ERROR(MOD, "[tid={}] step=match_exception msg={}", tid, e.what());
    return result;
} catch (...) {
    ALOG_ERROR(MOD, "[tid={}] step=match_unknown_exception", tid);
    return result;
}
}

} // namespace automap_pro
