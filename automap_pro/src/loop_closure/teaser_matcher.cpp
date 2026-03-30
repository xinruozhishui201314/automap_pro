/**
 * @file teaser_matcher.cpp
 * @brief TEASER 匹配、FPFH 预处理、SVD/Umeyama 兜底与配置热更新（applyConfig）。
 */
#include "automap_pro/loop_closure/teaser_matcher.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/utils.h"
#define MOD "TeaserMatcher"
#include "automap_pro/loop_closure/fpfh_extractor.h"

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <memory>
#include <new>
#include <pcl/common/transforms.h>
#include <Eigen/SVD>
#if defined(__linux__)
#include <unistd.h>
#include <sys/syscall.h>
#endif

namespace automap_pro {

namespace {
// 用于 CRASH_TRACE 的 LWP（与 GDB "info threads" 对应），便于精确定位崩溃线程
inline long getLwpForLog() {
#if defined(__linux__)
    return static_cast<long>(syscall(SYS_gettid));
#else
    return -1;
#endif
}

/// SVD/Umeyama 配准：无 TEASER PMC 依赖，极少对应点时仍稳定不崩溃
/// 当 corrs 少时 TEASER 析构会 SIGSEGV，用此替代可从根本上避免
void runSvdRegistration(const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
                        const std::vector<std::pair<int, int>>& corrs,
                        TeaserMatcher::Result& result, float max_rmse) {
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
    result.inlier_ratio = 1.0f;  // SVD 无内点筛选
    result.success = (result.rmse < max_rmse);
}
}  // namespace

TeaserMatcher::TeaserMatcher() {
    applyConfig();
}

void TeaserMatcher::applyConfig() {
    const auto& cfg = ConfigManager::instance();
    noise_bound_       = cfg.teaserNoiseBound();
    cbar2_             = cfg.teaserCbar2();
    voxel_size_        = cfg.teaserVoxelSize();
    min_inlier_ratio_  = cfg.teaserMinInlierRatio();
    max_rmse_          = cfg.teaserMaxRMSE();
    max_points_        = cfg.teaserMaxPoints();
    min_safe_inliers_  = cfg.teaserMinSafeInliers();
    fpfh_corr_max_dist_ = cfg.teaserFpfhCorrMaxDistanceM();
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
    const Pose3d& initial_guess) const
{
    // 首次 match 时打印实际使用的参数到 ROS 日志，便于 full.log 验证 YAML 是否生效
    static std::once_flag once_verify;
    std::call_once(once_verify, [this]() {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[TeaserMatcher][VERIFY] first match params in use: min_safe_inliers=%d min_inlier_ratio=%.2f max_rmse=%.2f (grep VERIFY 验证与 CONFIG 一致)",
            min_safe_inliers_, min_inlier_ratio_, max_rmse_);
    });

    const unsigned tid = automap_pro::logThreadId();
    const size_t src_pts = src_cloud ? src_cloud->size() : 0u;
    const size_t tgt_pts = tgt_cloud ? tgt_cloud->size() : 0u;
    ALOG_INFO(MOD, "[LOOP_STEP][TEASER] match_enter src_pts={} tgt_pts={} params: voxel_size={:.3f} max_points={} min_safe_inliers={} min_inlier_ratio={:.3f} max_rmse={:.3f}m (拒绝条件: inliers<min_safe_inliers 或 ratio<min_inlier_ratio 或 rmse>max_rmse)",
              src_pts, tgt_pts, voxel_size_, max_points_, min_safe_inliers_, min_inlier_ratio_, max_rmse_);
    ALOG_INFO(MOD, "[LOOP_COMPUTE][TEASER] match_enter src_pts={} tgt_pts={} (精准优化: 后续见 fpfh_done/teaser_solve_enter/teaser_done)",
              src_pts, tgt_pts);
    ALOG_INFO(MOD, "[tid={}] step=match_enter src_ptr={} src_use_count={} src_pts={} tgt_ptr={} tgt_use_count={} tgt_pts={}",
              tid, static_cast<const void*>(src_cloud.get()), src_cloud ? src_cloud.use_count() : 0, src_pts,
              static_cast<const void*>(tgt_cloud.get()), tgt_cloud ? tgt_cloud.use_count() : 0, tgt_pts);
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

        // 粗匹配初值 T_tgt_src：将源点云预变换到目标系，使 FPFH/TEASER/SVD 在近似重叠坐标系下工作；输出时 T_tgt_src = T_est * T_init
        const bool init_finite = initial_guess.matrix().allFinite();
        const bool use_pose_init =
            init_finite &&
            !initial_guess.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-12);
        if (use_pose_init) {
            CloudXYZIPtr src_warped = std::make_shared<CloudXYZI>();
            src_warped->reserve(src->size());
            Eigen::Affine3f Ttf;
            Ttf.matrix() = initial_guess.matrix().cast<float>();
            pcl::transformPointCloud(*src, *src_warped, Ttf);
            src = src_warped;
            ALOG_INFO(MOD,
                      "[tid={}] step=pose_init_warp src prealigned to tgt (T_tgt_src init): trans_norm={:.3f}m",
                      tid, initial_guess.translation().norm());
        }

        // === 检查最少点数要求 ===
        if (src->size() < 30 || tgt->size() < 30) {
            ALOG_INFO(MOD, "[LOOP_COMPUTE][TEASER] teaser_fail reason=insufficient_pts src_pts={} tgt_pts={} min=30", src->size(), tgt->size());
            ALOG_WARN(MOD, "[tid={}] step=match_skip insufficient_pts src={} tgt={} (min=30)", tid, src->size(), tgt->size());
            ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=insufficient_pts tid={} src_pts={} tgt_pts={} min=30 (精准定位: 预处理后点数不足，可增大 teaser.max_points 或减小 voxel_size)", tid, src->size(), tgt->size());
            return result;
        }

        // === 阶段3：FPFH 特征提取（每步独立 try-catch，便于定位异常阶段）===
        ALOG_DEBUG(MOD, "[tid={}] step=fpfh_compute_start src_pts={} tgt_pts={}", tid, src->size(), tgt->size());
        FpfhExtractor extractor;

        FPFHCloudPtr src_feat;
        try {
            ALOG_DEBUG(MOD, "[tid={}] step=fpfh_src_compute_start src_ptr={} src_pts={}", tid, static_cast<const void*>(src.get()), src->size());
            // 🏛️ [V3 修复] 自适应半径：确保法线搜索半径至少为体素大小的 2 倍，FPFH 半径为 4 倍。
            // 否则在 sparse 下采样（如 0.5m）后，0.5m 半径无法找到邻域导致 FPFH 特征全是 NaN/垃圾。
            const float normal_r = std::max(0.5f, static_cast<float>(voxel_size_ * 2.0));
            const float fpfh_r   = std::max(1.0f, normal_r * 2.0f);
            src_feat = extractor.compute(src, normal_r, fpfh_r);
        } catch (const std::bad_alloc& e) {
            ALOG_ERROR(MOD, "[tid={}] step=fpfh_src_exception reason=bad_alloc msg={}", tid, e.what());
            ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=fpfh_src_bad_alloc tid={} (精准定位: 源点云 FPFH 内存不足)", tid);
            return result;
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "[tid={}] step=fpfh_src_exception msg={}", tid, e.what());
            ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=fpfh_src_exception tid={} what={}", tid, e.what());
            return result;
        } catch (...) {
            ALOG_ERROR(MOD, "[tid={}] step=fpfh_src_unknown_exception", tid);
            ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=fpfh_src_unknown_exception tid={}", tid);
            return result;
        }
        ALOG_INFO(MOD, "[tid={}] step=fpfh_src_done src_feat_ptr={} src_feat_pts={} src_feat_use_count={} src_pts={}",
                 tid, static_cast<const void*>(src_feat.get()), src_feat ? src_feat->size() : 0u, src_feat ? src_feat.use_count() : 0, src->size());

        if (!src_feat || src_feat->empty()) {
            ALOG_WARN(MOD, "[tid={}] step=fpfh_src_empty_skip src_pts={} src_feat_pts={}", tid, src->size(), src_feat ? src_feat->size() : 0u);
            ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=fpfh_src_empty tid={} src_pts={} (精准定位: 源点云 FPFH 特征为空)", tid, src->size());
            return result;
        }

        FPFHCloudPtr tgt_feat;
        try {
            ALOG_DEBUG(MOD, "[tid={}] step=fpfh_tgt_compute_start tgt_ptr={} tgt_pts={}", tid, static_cast<const void*>(tgt.get()), tgt->size());
            const float normal_r = std::max(0.5f, static_cast<float>(voxel_size_ * 2.0));
            const float fpfh_r   = std::max(1.0f, normal_r * 2.0f);
            tgt_feat = extractor.compute(tgt, normal_r, fpfh_r);
        } catch (const std::bad_alloc& e) {
            ALOG_ERROR(MOD, "[tid={}] step=fpfh_tgt_exception reason=bad_alloc msg={}", tid, e.what());
            ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=fpfh_tgt_bad_alloc tid={} (精准定位: 目标点云 FPFH 内存不足)", tid);
            return result;
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "[tid={}] step=fpfh_tgt_exception msg={}", tid, e.what());
            ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=fpfh_tgt_exception tid={} what={}", tid, e.what());
            return result;
        } catch (...) {
            ALOG_ERROR(MOD, "[tid={}] step=fpfh_tgt_unknown_exception", tid);
            ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=fpfh_tgt_unknown_exception tid={}", tid);
            return result;
        }
        ALOG_INFO(MOD, "[tid={}] step=fpfh_tgt_done tgt_feat_ptr={} tgt_feat_pts={} tgt_feat_use_count={} tgt_pts={}",
                 tid, static_cast<const void*>(tgt_feat.get()), tgt_feat ? tgt_feat->size() : 0u, tgt_feat ? tgt_feat.use_count() : 0, tgt->size());

        if (!tgt_feat || tgt_feat->empty()) {
            ALOG_WARN(MOD, "[tid={}] step=fpfh_tgt_empty_skip tgt_pts={} tgt_feat_pts={}", tid, tgt->size(), tgt_feat ? tgt_feat->size() : 0u);
            ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=fpfh_tgt_empty tid={} tgt_pts={} (精准定位: 目标点云 FPFH 特征为空)", tid, tgt->size());
            return result;
        }

        // === 阶段4：特征对应匹配 ===
        std::vector<std::pair<int, int>> corrs;
        try {
            ALOG_DEBUG(MOD, "[tid={}] step=fpfh_match_start src_feat={} tgt_feat={}", tid, src_feat ? src_feat->size() : 0u, tgt_feat ? tgt_feat->size() : 0u);
            corrs = extractor.findCorrespondences(src_feat, tgt_feat, true);
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "[tid={}] step=fpfh_match_exception msg={}", tid, e.what());
            ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=fpfh_match_exception tid={} what={}", tid, e.what());
            return result;
        } catch (...) {
            ALOG_ERROR(MOD, "[tid={}] step=fpfh_match_unknown_exception", tid);
            ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=fpfh_match_unknown_exception tid={}", tid);
            return result;
        }
        result.num_correspondences = static_cast<int>(corrs.size());
        ALOG_INFO(MOD, "[LOOP_COMPUTE][TEASER] fpfh_done corrs={} (精准优化: corrs<20 会跳过 TEASER，可调 FPFH 或 voxel 增加对应点)",
                  result.num_correspondences);
        ALOG_INFO(MOD, "[tid={}] step=fpfh_match_done correspondences={}", tid, result.num_correspondences);

        // 【FPFH_DIAG】统计对应点距离分布，诊断误匹配（阈值仅来自 applyConfig 缓存，避免 match 线程访问 ConfigManager）
        const double fpfh_corr_max_dist = fpfh_corr_max_dist_;
        if (!corrs.empty()) {
            std::vector<double> distances;
            distances.reserve(corrs.size());
            for (const auto& [si, ti] : corrs) {
                const auto& sp = src->points[si];
                const auto& tp = tgt->points[ti];
                double d = std::sqrt((sp.x - tp.x) * (sp.x - tp.x) +
                                    (sp.y - tp.y) * (sp.y - tp.y) +
                                    (sp.z - tp.z) * (sp.z - tp.z));
                distances.push_back(d);
            }
            std::sort(distances.begin(), distances.end());
            double p10 = distances[std::min(size_t(distances.size() * 0.1), distances.size() - 1)];
            double p50 = distances[distances.size() / 2];
            double p90 = distances[std::min(size_t(distances.size() * 0.9), distances.size() - 1)];
            result.fpfh_p90 = static_cast<float>(p90);
            ALOG_WARN(MOD, "[FPFH_DIAG] tid={} corrs={} dist_p10={:.2f}m p50={:.2f}m p90={:.2f}m "
                      "(p90>>5m 表示存在大量误匹配；若 p10 也很大则 FPFH 特征可能在不同区域发生了严重哈希碰撞)",
                      tid, corrs.size(), p10, p50, p90);
            
            // 【架构防护】拦截极端垃圾数据：若 90% 的对应点距离过大，说明特征完全匹配错误
            // 此时调用 TEASER++ 会触发内部 PMC 内存损坏风险，必须直接拒绝。
            // 🏛️ [V3 修复] 动态阈值：硬截断应略大于 FPFH 过滤阈值，避免合法的大位移候选被误杀（尤其在 coordinate offset 场景下）。
            const double hard_reject_thresh = std::max(20.0, fpfh_corr_max_dist * 2.0);
            if (p90 > hard_reject_thresh) {
                ALOG_ERROR(MOD, "[FPFH_DIAG][CRITICAL] High p90 distance ({:.2f}m) detected (thresh={:.1f}m)! REJECTING garbage input to prevent TEASER++ heap corruption.", p90, hard_reject_thresh);
                ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=fpfh_garbage_input tid={} p90={:.2f}", tid, p90);
                result.success = false;
                result.fpfh_garbage_rejected = true;
                return result;
            }

            // FPFH 对应点几何过滤：仅保留距离 <= fpfh_corr_max_dist 的对应点再送 TEASER（树木场景建议 3～5m）
            if (fpfh_corr_max_dist > 0 && !corrs.empty()) {
                std::vector<std::pair<int, int>> filtered_corrs;
                filtered_corrs.reserve(corrs.size());
                for (const auto& [si, ti] : corrs) {
                    const auto& sp = src->points[si];
                    const auto& tp = tgt->points[ti];
                    double d = std::sqrt((sp.x - tp.x) * (sp.x - tp.x) +
                                        (sp.y - tp.y) * (sp.y - tp.y) +
                                        (sp.z - tp.z) * (sp.z - tp.z));
                    if (d <= fpfh_corr_max_dist) {
                        filtered_corrs.emplace_back(si, ti);
                    }
                }
                if (filtered_corrs.size() >= 10) {
                    ALOG_INFO(MOD, "[FPFH_GEO_FILTER] tid={} filtered_corrs={}/{} ({:.1f}%) max_dist={:.2f}m (loop_closure.teaser.fpfh_corr_max_distance_m)",
                              tid, filtered_corrs.size(), corrs.size(),
                              100.0 * filtered_corrs.size() / corrs.size(), fpfh_corr_max_dist);
                    corrs = std::move(filtered_corrs);
                    result.num_correspondences = static_cast<int>(corrs.size());
                } else if (p90 > 10.0) {
                    // 兼容：过滤后不足 10 对且 p90 很大时，用中位数+倍数做宽松过滤
                    const double median_dist = p50;
                    const double abs_threshold = std::max(median_dist * 3.0, 10.0);
                    std::vector<std::pair<int, int>> fallback_corrs;
                    fallback_corrs.reserve(corrs.size());
                    for (const auto& [si, ti] : corrs) {
                        const auto& sp = src->points[si];
                        const auto& tp = tgt->points[ti];
                        double d = std::sqrt((sp.x - tp.x) * (sp.x - tp.x) +
                                            (sp.y - tp.y) * (sp.y - tp.y) +
                                            (sp.z - tp.z) * (sp.z - tp.z));
                        if (d <= abs_threshold) fallback_corrs.emplace_back(si, ti);
                    }
                    if (fallback_corrs.size() >= 10) {
                        corrs = std::move(fallback_corrs);
                        result.num_correspondences = static_cast<int>(corrs.size());
                        ALOG_WARN(MOD, "[FPFH_GEO_FILTER] tid={} fallback filtered_corrs={}/{} threshold={:.2f}m",
                                  tid, corrs.size(), result.num_correspondences, abs_threshold);
                    }
                }
            }
        }

        if ((int)corrs.size() < 10) {
            ALOG_INFO(MOD, "[LOOP_COMPUTE][TEASER] teaser_fail reason=insufficient_corrs corrs={} min=10", static_cast<int>(corrs.size()));
            ALOG_WARN(MOD, "[tid={}] step=match_skip insufficient_corrs={} (min=10)", tid, corrs.size());
            ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=insufficient_corrs tid={} corrs={} min=10 (精准定位: FPFH 对应点过少)", tid, corrs.size());
            return result;
        }

#ifdef USE_TEASER
    const int corr_count = static_cast<int>(corrs.size());
    // 【根本修复】corrs < 20 时用 SVD 替代 TEASER：不创建 TEASER 对象，从根源避免 PMC 析构崩溃
    // 内点可能确实很少，SVD 无 PMC 依赖，鲁棒性略低但绝不崩溃
    if (corr_count < 20) {
        ALOG_INFO(MOD, "[LOOP_COMPUTE][TEASER] corrs={} < 20, using SVD fallback (avoids TEASER PMC destructor crash)",
                  corr_count);
        ALOG_INFO(MOD, "[tid={}] step=svd_fallback_low_corrs corrs={} (fundamental fix: no TEASER, no crash)", tid, corr_count);
        try {
            runSvdRegistration(src, tgt, corrs, result, max_rmse_);
            result.used_teaser = false;
            result.geom_path = TeaserMatcher::GeomPath::SVD_FALLBACK;
            if (use_pose_init) {
                result.T_tgt_src = result.T_tgt_src * initial_guess;
            }
            ALOG_INFO(MOD, "[tid={}] step=svd_fallback_done rmse={:.3f}m success={}", tid, result.rmse, result.success);
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "[tid={}] step=svd_fallback_exception msg={}", tid, e.what());
            return result;
        } catch (...) {
            ALOG_ERROR(MOD, "[tid={}] step=svd_fallback_unknown_exception", tid);
            return result;
        }
    } else {
    result.used_teaser = true;
    result.geom_path = TeaserMatcher::GeomPath::TEASER;

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
    // 强制 PMC 单线程，避免极少 inlier 路径下析构时多线程 free() 导致 SIGSEGV（见 TEASER_CRASH_ANALYSIS.md）
    params.max_clique_num_threads   = 1;

    try {
        ALOG_INFO(MOD, "[LOOP_COMPUTE][TEASER] teaser_solve_enter corrs={} noise_bound={} (精准优化: 求解后见 teaser_done inliers/ratio)",
                  static_cast<int>(corrs.size()), params.noise_bound);
        ALOG_DEBUG(MOD, "[tid={}] step=teaser_solve_enter corrs={} noise_bound={}", tid, corrs.size(), params.noise_bound);
        AUTOMAP_TIMED_SCOPE(MOD, fmt::format("TEASER solve corr={}", corrs.size()), 3000.0);
        // 使用 unique_ptr 便于在低 inlier 提前 return 前显式 reset，避免栈展开时 TEASER 析构触发已知的 SIGSEGV（极少 inlier 时）
        auto solver = std::make_unique<teaser::RobustRegistrationSolver>(params);
        ALOG_INFO(MOD, "[CRASH_TRACE][tid={} lwp={}] step=teaser_solver_created solver_ptr={}", tid, getLwpForLog(), static_cast<const void*>(solver.get()));
        solver->solve(src_pts, tgt_pts, teaser_corrs);
        ALOG_DEBUG(MOD, "[tid={}] step=teaser_solve_done", tid);

        auto solution = solver->getSolution();
        if (!solution.valid) {
            ALOG_INFO(MOD, "[LOOP_COMPUTE][TEASER] teaser_fail reason=teaser_solution_invalid corrs={}", static_cast<int>(corrs.size()));
            ALOG_WARN(MOD, "[tid={}] step=teaser_solve_invalid corr={}", tid, corrs.size());
            ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=teaser_solution_invalid tid={} corrs={}", tid, corrs.size());
            // 【V4 修复】不再 release 导致泄漏；已通过 pre-filter 拦截垃圾输入，此处 reset 是安全的
            solver.reset();
            return result;
        }
        ALOG_DEBUG(MOD, "[tid={}] step=teaser_solution_valid", tid);

        // 计算内点率（getInlierMaxClique 返回 vector<int>，为索引个数即 inlier 数）
        const auto max_clique = solver->getInlierMaxClique();
        const int inliers = static_cast<int>(max_clique.size());
        const float ratio = corrs.empty() ? 0.f : static_cast<float>(inliers) / static_cast<float>(corrs.size());
        const int valid = (inliers >= min_safe_inliers_ && ratio >= min_inlier_ratio_) ? 1 : 0;
        ALOG_INFO(MOD, "[LOOP_COMPUTE][TEASER] teaser_done inliers={} corrs={} ratio={:.3f} valid={} (精准优化: inliers<min_safe_inliers 会拒绝，可调 loop_closure.teaser.min_safe_inliers)",
                  inliers, static_cast<int>(corrs.size()), ratio, valid);
        ALOG_INFO(MOD, "[tid={}] step=teaser_inlier_computed inliers={}/{} ratio={:.2f} thresh={}",
                 tid, inliers, corrs.size(), ratio, min_inlier_ratio_);
        ALOG_INFO(MOD, "[TEASER_DIAG] inliers={} corrs={} ratio={:.4f} min_safe_inliers={} min_ratio={:.3f} (valid={})",
                  inliers, static_cast<int>(corrs.size()), ratio, min_safe_inliers_, min_inlier_ratio_, valid);
        // 子图间几何诊断：TEASER 估计的 T_tgt_src（target 系下 source 位姿），便于与 odom 相对位姿对比
        {
            const double teaser_trans_m = solution.translation.norm();
            const Eigen::Matrix3d R = solution.rotation;
            const double teaser_rot_deg = Eigen::AngleAxisd(R).angle() * 180.0 / M_PI;
            
            // 提取 RPY
            Eigen::Vector3d rpy = R.eulerAngles(2, 1, 0).reverse() * 180.0 / M_PI;
            
            ALOG_INFO(MOD, "[TEASER_DIAG] estimated_pose T_tgt_src: trans_norm_m={:.3f} rot_deg={:.2f} rpy=[{:.1f},{:.1f},{:.1f}] (与 INTER_KF GEOM_DIAG 中 odom rel_trans/rel_rot 对比；差异大则误匹配或 FPFH 对应点差)",
                      teaser_trans_m, teaser_rot_deg, rpy.x(), rpy.y(), rpy.z());
            
            // ⚠️ [GHOSTING_DIAG] 特别关注 Pitch 和 Roll
            // 正常路面建图 Pitch/Roll 不应超过 20 度（除非是无人机或极端坡度）
            // 拦截翻转匹配（180度翻转），防止重影
            if (std::abs(rpy.x()) > 20.0 || std::abs(rpy.y()) > 20.0) {
                ALOG_ERROR(MOD, "[TEASER_DIAG][UNUSUAL_ORIENTATION] FATAL: Large Pitch/Roll detected: P={:.1f} R={:.1f}. REJECTING FLIPPED MATCH to prevent ghosting!", rpy.y(), rpy.x());
                result.success = false;
                return result;
            }
        }

        // 【TEASER_DIAG】使用 solution 的位姿计算内点 RMSE 和距离分布（result.T_tgt_src 尚未赋值）
        if (inliers > 0) {
            const Eigen::Matrix3d& R_sol = solution.rotation;
            const Eigen::Vector3d& t_sol = solution.translation;
            std::vector<double> inlier_dists;
            inlier_dists.reserve(inliers);
            auto inlier_map = solver->getTranslationInliersMap();
            double sq_err_sum = 0.0;
            for (Eigen::Index c = 0; c < inlier_map.cols(); ++c) {
                int idx = inlier_map(0, c);
                if (idx < 0 || idx >= static_cast<int>(corrs.size())) continue;
                const auto& sp = src->points[corrs[idx].first];
                const auto& tp = tgt->points[corrs[idx].second];
                Eigen::Vector3d pred = R_sol * Eigen::Vector3d(sp.x, sp.y, sp.z) + t_sol;
                Eigen::Vector3d actual(tp.x, tp.y, tp.z);
                double e = (pred - actual).norm();
                inlier_dists.push_back(e);
                sq_err_sum += e * e;
            }
            if (!inlier_dists.empty()) {
                std::sort(inlier_dists.begin(), inlier_dists.end());
                double rmse = std::sqrt(sq_err_sum / inlier_dists.size());
                double p50 = inlier_dists[inlier_dists.size() / 2];
                double p90 = inlier_dists[std::min(size_t(inlier_dists.size() * 0.9), inlier_dists.size() - 1)];
                ALOG_WARN(MOD, "[TEASER_DIAG] tid={} inliers={} rmse={:.3f}m dist_p50={:.3f}m p90={:.3f}m "
                          "(rmse>0.5m 或 p90>1m 表示几何一致性差)",
                          tid, inliers, rmse, p50, p90);
            }
        }

        // ===【V3 激进修复】双重安全检查：提前 abort + 延迟析构===
        if (inliers < min_safe_inliers_) {
            ALOG_INFO(MOD, "[LOOP_COMPUTE][TEASER] teaser_fail reason=teaser_extremely_few_inliers inliers={} safe_min={} (精准优化: 可放宽 loop_closure.teaser.min_safe_inliers 或改进 FPFH/重叠)",
                      inliers, min_safe_inliers_);
            ALOG_WARN(MOD, "[tid={}] [CRITICAL_V3] step=teaser_extremely_few_inliers inliers={} < safe_threshold={}, HIGH CRASH RISK",
                     tid, inliers, min_safe_inliers_);
            ALOG_INFO(MOD, "[CRASH_TRACE][tid={} lwp={}] step=teaser_solver_abort_low_inliers solver_ptr={} inliers={} safe_min={}",
                      tid, getLwpForLog(), static_cast<const void*>(solver.get()), inliers, min_safe_inliers_);
            std::cerr << "[CRASH_TRACE_CRITICAL] lwp=" << getLwpForLog()
                      << " step=teaser_extremely_few_inliers inliers=" << inliers
                      << " safe_min=" << min_safe_inliers_ << std::endl;
            // 【V4 修复】不再 release 导致泄漏；已通过 pre-filter 拦截垃圾输入，此处 reset 是安全的
            solver.reset();
            ALOG_INFO(MOD, "[CRASH_TRACE][tid={} lwp={}] step=teaser_solver_reset_done solver_ptr_reset=true",
                      tid, getLwpForLog());
            ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=teaser_extremely_few_inliers tid={} inliers={} safe_min={} (V4: reset solver)",
                     tid, inliers, min_safe_inliers_);
            return result;
        }

        result.inlier_ratio = (float)inliers / (float)corrs.size();

        // 将 TEASER 解写入 result，否则 result.T_tgt_src 仍为 Identity，子图间回环会全部 trans_norm=0 被 trivial 过滤
        result.T_tgt_src = Pose3d::Identity();
        result.T_tgt_src.linear() = solution.rotation;
        result.T_tgt_src.translation() = solution.translation;

        if (result.inlier_ratio < min_inlier_ratio_) {
            if (use_pose_init) {
                result.T_tgt_src = result.T_tgt_src * initial_guess;
            }
            ALOG_INFO(MOD, "[LOOP_COMPUTE][TEASER] teaser_fail reason=inlier_ratio_low inliers={} ratio={:.3f} min_ratio={}", inliers, result.inlier_ratio, min_inlier_ratio_);
            ALOG_WARN(MOD, "[tid={}] step=teaser_inlier_rejected inlier_ratio={:.2f} < thresh={}", tid, result.inlier_ratio, min_inlier_ratio_);
            ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=inlier_ratio_low tid={} inlier_ratio={:.3f} min={}", tid, result.inlier_ratio, min_inlier_ratio_);
            ALOG_INFO(MOD, "[CRASH_TRACE][tid={} lwp={}] step=teaser_inlier_rejected_natural_destruct solver_ptr={} inliers={}",
                      tid, getLwpForLog(), static_cast<const void*>(solver.get()), inliers);
            std::cerr << "[CRASH_TRACE] lwp=" << getLwpForLog() << " step=teaser_inlier_rejected_natural_destruct inliers=" << inliers 
                      << " (solver will be released and destruct at function exit)" << std::endl;
            // 【V4 修复】不再 release 导致泄漏；已通过 pre-filter 拦截垃圾输入，此处 reset 是安全的
            solver.reset();
            ALOG_INFO(MOD, "[CRASH_TRACE][tid={} lwp={}] step=teaser_solver_reset_done solver_ptr_reset=true (delayed destruct)",
                      tid, getLwpForLog());
            return result;
        }

        // ✅ 修复：在 solver 作用域内计算 RMSE（避免悬空引用）
        {
            double sq_err = 0.0;
            int cnt = 0;
            auto inlier_map = solver->getTranslationInliersMap();
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
        if (use_pose_init) {
            result.T_tgt_src = result.T_tgt_src * initial_guess;
        }
        // 【V4 优化】不再 release；已通过 pre-filter 拦截垃圾输入
        if (inliers < 10) {
            ALOG_WARN(MOD, "[tid={}] [SAFEGUARD] success path inliers={} < 10, resetting solver",
                      tid, inliers);
            solver.reset();
        }
        ALOG_INFO(MOD, "[CRASH_TRACE][tid={} lwp={}] step=teaser_solver_scope_exit success_path solver_ptr={} (unique_ptr leaving scope, natural destruct)",
                  tid, getLwpForLog(), static_cast<const void*>(solver.get()));
        std::cerr << "[CRASH_TRACE] lwp=" << getLwpForLog() << " step=teaser_success_path_solver_natural_destruct solver_ptr=" 
                  << static_cast<const void*>(solver.get()) << std::endl;
    } catch (const std::bad_alloc& e) {
        ALOG_ERROR(MOD, "[tid={} lwp={}] [ROBUST_FIX] step=teaser_solve_exception reason=bad_alloc msg={}", tid, getLwpForLog(), e.what());
        ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=teaser_out_of_memory tid={} (TEASER/PMC OOM)", tid);
        std::cerr << "[ROBUST_FIX_L5_BADALLOC] lwp=" << getLwpForLog() << " TEASER OOM" << std::endl;
        return result;
    } catch (const std::runtime_error& e) {
        ALOG_ERROR(MOD, "[tid={} lwp={}] [ROBUST_FIX] step=teaser_solve_exception reason=runtime_error msg={}", tid, getLwpForLog(), e.what());
        ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=teaser_runtime_error tid={} what={}", tid, e.what());
        std::cerr << "[ROBUST_FIX_L5_RUNTIME] lwp=" << getLwpForLog() << " runtime_error: " << e.what() << std::endl;
        return result;
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "[tid={} lwp={}] [ROBUST_FIX] step=teaser_solve_exception type={} msg={}", tid, getLwpForLog(), typeid(e).name(), e.what());
        ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=teaser_std_exception tid={} type={}", tid, typeid(e).name());
        std::cerr << "[ROBUST_FIX_L5_EXCEPTION] lwp=" << getLwpForLog() << " type=" << typeid(e).name() << std::endl;
        return result;
    } catch (...) {
        ALOG_ERROR(MOD, "[tid={} lwp={}] [ROBUST_FIX] step=teaser_solve_unknown_exception (CRITICAL: unknown type)", tid, getLwpForLog());
        ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=teaser_unknown_exception tid={} (CRITICAL, may indicate solver crash)", tid);
        std::cerr << "[ROBUST_FIX_L5_CRITICAL] lwp=" << getLwpForLog() << " UNKNOWN EXCEPTION - solver may have crashed" << std::endl;
        return result;
    }
    }  // else (corrs >= 20, TEASER path)
#else
    // 无 TEASER++ 时用 SVD
    ALOG_DEBUG(MOD, "[tid={}] step=svd_fallback TEASER disabled, using SVD corrs={}", tid, corrs.size());
    try {
        runSvdRegistration(src, tgt, corrs, result, max_rmse_);
        result.used_teaser = false;
        result.geom_path = TeaserMatcher::GeomPath::SVD_FALLBACK;
        if (use_pose_init) {
            result.T_tgt_src = result.T_tgt_src * initial_guess;
        }
        ALOG_INFO(MOD, "[tid={}] step=svd_done rmse={:.3f}m success={}", tid, result.rmse, result.success);
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
        ALOG_INFO(MOD, "[TEASER_RESULT] PASS inliers≈{} corrs={} ratio={:.4f} rmse={:.4f}m (min_safe={} min_ratio={:.2f} max_rmse={:.2f}m)",
                  static_cast<int>(result.inlier_ratio * std::max(0, result.num_correspondences)),
                  result.num_correspondences, result.inlier_ratio, result.rmse,
                  min_safe_inliers_, min_inlier_ratio_, max_rmse_);
    } else {
        ALOG_WARN(MOD, "[tid={}] step=match_failed inlier={:.2f} corrs={} rmse={:.3f}m",
                 tid, result.inlier_ratio, result.num_correspondences, result.rmse);
        ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=rmse_too_high tid={} rmse={:.4f} max_rmse={} (精准定位: 匹配 RMSE 超阈值)", tid, result.rmse, max_rmse_);
        ALOG_INFO(MOD, "[TEASER_RESULT] FAIL reason=rmse_or_final_check inlier_ratio={:.4f} corrs={} rmse={:.4f}m max_rmse={:.2f}m",
                  result.inlier_ratio, result.num_correspondences, result.rmse, max_rmse_);
    }
    ALOG_INFO(MOD, "[TEASER_EXIT] result={} success={} inlier_ratio={:.4f} rmse={:.4f}m corrs={} (若 FAIL 见上方 [LOOP_COMPUTE][TEASER] reason= 或 [TEASER_RESULT] FAIL)",
              result.success ? "PASS" : "FAIL", result.success ? 1 : 0,
              result.inlier_ratio, result.rmse, result.num_correspondences);
    ALOG_INFO(MOD, "[TEASER_PATH] used_teaser={} geom_path={} corrs={}",
              result.used_teaser ? 1 : 0,
              result.geom_path == TeaserMatcher::GeomPath::TEASER ? "TEASER" :
              (result.geom_path == TeaserMatcher::GeomPath::SVD_FALLBACK ? "SVD_FALLBACK" : "UNKNOWN"),
              result.num_correspondences);
    ALOG_INFO(MOD, "[LOOP_STEP][TEASER] match_exit success={} inliers≈{} corrs={} rmse={:.4f}m (几何验证完成，success=1 表示回环候选通过 TEASER)",
              result.success, static_cast<int>(result.inlier_ratio * std::max(0, result.num_correspondences)),
              result.num_correspondences, result.rmse);
    ALOG_DEBUG(MOD, "[tid={}] step=match_exit", tid);
    return result;
} catch (const std::bad_alloc& e) {
    ALOG_ERROR(MOD, "[tid={}] [ROBUST_FIX] step=match_exception reason=bad_alloc msg={}", tid, e.what());
    ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=match_bad_alloc tid={} (OOM during match flow)", tid);
    std::cerr << "[ROBUST_FIX_OUTER] lwp=" << getLwpForLog() << " bad_alloc: " << e.what() << std::endl;
    return result;
} catch (const std::exception& e) {
    ALOG_ERROR(MOD, "[tid={}] [ROBUST_FIX] step=match_exception type={} msg={}", tid, typeid(e).name(), e.what());
    ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=match_exception tid={} what={}", tid, e.what());
    std::cerr << "[ROBUST_FIX_OUTER] lwp=" << getLwpForLog() << " exception: " << typeid(e).name() << " - " << e.what() << std::endl;
    return result;
} catch (...) {
    ALOG_ERROR(MOD, "[tid={}] [ROBUST_FIX] step=match_unknown_exception (critical, unknown type)", tid);
    ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=match_unknown_exception tid={} (CRITICAL)", tid);
    std::cerr << "[ROBUST_FIX_OUTER_CRITICAL] lwp=" << getLwpForLog() << " unknown exception caught" << std::endl;
    return result;
}
}

} // namespace automap_pro
