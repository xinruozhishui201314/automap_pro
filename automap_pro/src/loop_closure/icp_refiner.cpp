#include "automap_pro/loop_closure/icp_refiner.h"
#include "automap_pro/core/logger.h"
#define MOD "ICPRefiner"
#include <pcl/common/transforms.h>

namespace automap_pro {

IcpRefiner::Result IcpRefiner::refine(
    const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
    const Pose3d& initial) const
{
    Result res;
    if (!src || !tgt || src->empty() || tgt->empty()) return res;

    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    icp.setMaximumIterations(max_iter_);
    icp.setMaxCorrespondenceDistance(max_corr_dist_);
    icp.setTransformationEpsilon(tf_eps_);
    icp.setEuclideanFitnessEpsilon(rmse_eps_);

    CloudXYZI aligned;
    Eigen::Matrix4f init = initial.cast<float>().matrix();
    icp.align(aligned, init);

    res.converged  = icp.hasConverged();
    res.rmse       = icp.getFitnessScore();
    res.iterations = max_iter_;

    if (res.converged) {
        Eigen::Matrix4d final_tf = icp.getFinalTransformation().cast<double>();
        res.T_refined.matrix() = final_tf;
        ALOG_DEBUG(MOD, "ICP converged: rmse={:.4f} src={} tgt={} pts",
                   res.rmse, src->size(), tgt->size());
    } else {
        ALOG_WARN(MOD, "ICP NOT converged: rmse={:.4f} src={} tgt={} pts",
                  res.rmse, src->size(), tgt->size());
    }
    return res;
}

} // namespace automap_pro
