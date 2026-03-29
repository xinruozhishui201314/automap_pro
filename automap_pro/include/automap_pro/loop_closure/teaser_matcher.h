#pragma once
#include "automap_pro/core/data_types.h"

#ifdef USE_TEASER
#include <teaser/registration.h>
#include <teaser/fpfh.h>
#endif

namespace automap_pro {

class TeaserMatcher {
public:
    enum class GeomPath : int {
        UNKNOWN = 0,
        TEASER = 1,
        SVD_FALLBACK = 2
    };

    struct Result {
        bool    success       = false;
        Pose3d  T_tgt_src     = Pose3d::Identity();
        float   inlier_ratio  = 0.0f;
        float   rmse          = 1e6f;
        int     num_correspondences = 0;
        bool    used_teaser   = false;
        GeomPath geom_path    = GeomPath::UNKNOWN;
        bool    fpfh_garbage_rejected = false;
        float   fpfh_p90              = 0.0f; // 🏛️ [新增] 用于健康度量监控
    };

    explicit TeaserMatcher();
    /** 从 ConfigManager 重新加载 TEASER 参数（在 config load 之后调用，解决构造早于 load 导致默认值生效的问题） */
    void applyConfig();

    /** initial_guess: T_tgt_src，将 src 预变换到 tgt 系后再做 FPFH/TEASER/SVD，返回 T_tgt_src = T_est * initial_guess */
    Result match(const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
                 const Pose3d& initial_guess = Pose3d::Identity()) const;

private:
    double noise_bound_       = 0.1;
    double cbar2_             = 1.0;
    double voxel_size_        = 0.4;
    double min_inlier_ratio_  = 0.30;
    double max_rmse_          = 0.3;
    int    max_points_        = 4000;
    int    min_safe_inliers_  = 10;  // TEASER 内点数低于此拒绝；弱重叠可配 min_safe_inliers 放宽
    /** FPFH 几何过滤距离上限 (m)，在 applyConfig() 中从 YAML 缓存；match() 内禁止调 ConfigManager（异步线程 SIGSEGV） */
    double fpfh_corr_max_dist_ = 10.0;

    CloudXYZIPtr preprocess(const CloudXYZIPtr& cloud) const;
};

} // namespace automap_pro
