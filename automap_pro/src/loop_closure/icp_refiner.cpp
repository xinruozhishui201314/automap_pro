#include "automap_pro/loop_closure/icp_refiner.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

namespace automap_pro {

ICPRefiner::ICPRefiner() {
    const auto& cfg = ConfigManager::instance();
    max_iterations_             = cfg.icpMaxIterations();
    max_correspondence_distance_ = cfg.icpMaxCorrDist();
    convergence_threshold_      = cfg.icpConvThresh();
}

ICPRefiner::Result ICPRefiner::refine(const CloudXYZIPtr& src_cloud,
                                       const CloudXYZIPtr& tgt_cloud,
                                       const Pose3d& initial_T) const {
    Result result;
    result.T_refined = initial_T;

    if (!src_cloud || !tgt_cloud ||
        src_cloud->empty() || tgt_cloud->empty()) {
        return result;
    }

    // Point-to-plane ICP (nonlinear)
    pcl::IterativeClosestPointNonLinear<PointXYZI, PointXYZI> icp;
    icp.setMaximumIterations(max_iterations_);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
    icp.setTransformationEpsilon(convergence_threshold_);
    icp.setEuclideanFitnessEpsilon(convergence_threshold_ * 10.0);
    icp.setInputSource(src_cloud);
    icp.setInputTarget(tgt_cloud);

    // Initial transform
    Eigen::Matrix4f init_mat = initial_T.matrix().cast<float>();
    CloudXYZI aligned;
    icp.align(aligned, init_mat);

    result.converged = icp.hasConverged();
    if (result.converged) {
        result.T_refined = utils::matrix4dToIsometry3d(
            icp.getFinalTransformation().cast<double>());
        result.fitness_score = icp.getFitnessScore();
        result.rmse          = std::sqrt(result.fitness_score);
    }

    return result;
}

}  // namespace automap_pro
