#pragma once

#include "automap_pro/core/data_types.h"

namespace automap_pro {

// ──────────────────────────────────────────────────────────
// ICPRefiner: Point-to-plane ICP for fine alignment
// ──────────────────────────────────────────────────────────
class ICPRefiner {
public:
    struct Result {
        Pose3d T_refined;
        double fitness_score = 1e9;
        double rmse          = 1e9;
        bool   converged     = false;
    };

    ICPRefiner();
    ~ICPRefiner() = default;

    Result refine(const CloudXYZIPtr& src_cloud,
                  const CloudXYZIPtr& tgt_cloud,
                  const Pose3d& initial_T) const;

private:
    int    max_iterations_;
    double max_correspondence_distance_;
    double convergence_threshold_;
};

}  // namespace automap_pro
