// 模块9: 轨迹记录功能
// 包含: ensureTrajectoryLogDir, writeTrajectoryOdom, writeTrajectoryOdomAfterMapping, onGPSMeasurementForLog

#ifndef AUTOMAP_PRO_SYSTEM_MODULES_TRAJECTORY_LOG_H_
#define AUTOMAP_PRO_SYSTEM_MODULES_TRAJECTORY_LOG_H_

#include "automap_pro/system/automap_system.h"

namespace automap_pro {

void AutoMapSystem::ensureTrajectoryLogDir();
void AutoMapSystem::writeTrajectoryOdom(double ts, const Pose3d& pose, const Mat66d& cov);
void AutoMapSystem::writeTrajectoryOdomAfterMapping(const std::string& output_dir);
void AutoMapSystem::onGPSMeasurementForLog(double ts, const Eigen::Vector3d& pos_enu);

}  // namespace automap_pro

#endif  // AUTOMAP_PRO_SYSTEM_MODULES_TRAJECTORY_LOG_H_
