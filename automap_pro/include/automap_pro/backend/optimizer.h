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
    };

    struct Result {
        bool   success    = false;
        double final_cost = 0.0;
        int    iterations = 0;
        double time_ms    = 0.0;
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
