#pragma once
#include "automap_pro/core/data_types.h"
#include <pcl/registration/icp.h>

namespace automap_pro {

class IcpRefiner {
public:
    struct Result {
        bool   converged = false;
        Pose3d T_refined = Pose3d::Identity();
        double rmse = 1e6;
        int    iterations = 0;
    };
    Result refine(const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
                  const Pose3d& initial) const;
private:
    int    max_iter_     = 50;
    double max_corr_dist_ = 0.5;
    double tf_eps_       = 1e-8;
    double rmse_eps_     = 1e-8;
};

} // namespace automap_pro
