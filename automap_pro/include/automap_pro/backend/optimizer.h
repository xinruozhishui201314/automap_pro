#pragma once

#include <memory>
#include <functional>

#include "automap_pro/core/data_types.h"
#include "automap_pro/backend/pose_graph.h"

namespace automap_pro {

// ──────────────────────────────────────────────────────────
// Optimizer: wraps GTSAM (or Ceres) factor graph optimizer
// ──────────────────────────────────────────────────────────
class Optimizer {
public:
    struct Options {
        int    max_iterations        = 100;
        double convergence_threshold = 1e-4;
        bool   use_robust_kernel     = true;
        double robust_kernel_delta   = 1.0;
        bool   verbose               = false;

        // 求解失败检测参数
        double max_cost_threshold    = 1e10;   // 最大代价阈值（超过视为发散）
        double min_update_norm       = 1e-10;  // 最小更新范数（太小视为退化）
        bool   check_numerical_issues = true;   // 检查数值问题
    };

    struct Result {
        bool   success    = false;
        double final_cost = 0.0;
        int    iterations = 0;
        double time_ms    = 0.0;

        // 失败诊断信息
        enum class FailReason {
            NONE = 0,
            LINEAR_SYSTEM_SINGULAR,    // 线性系统奇异
            NUMERICAL_ISSUES,          // 数值问题（NaN/Inf）
            MAX_ITERATIONS_REACHED,    // 达到最大迭代次数
            COST_DIVERGED,             // 代价发散
            DEGENERATE_UPDATE          // 退化更新
        };
        FailReason fail_reason = FailReason::NONE;
        std::string fail_message;      // 详细失败信息
    };

    Optimizer() = default;
    ~Optimizer() = default;

    void setOptions(const Options& opts);

    // Optimize pose graph in-place (updates node poses in graph)
    Result optimize(PoseGraph& graph) const;

private:
    Options options_;

    Result optimizeGTSAM(PoseGraph& graph) const;
    Result optimizeCeres(PoseGraph& graph) const;
    Result optimizeGaussNewton(PoseGraph& graph) const;  // Fallback
};

}  // namespace automap_pro
