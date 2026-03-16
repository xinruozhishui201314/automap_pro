// 模块8: 定时任务与工具函数
// 包含: publishStatus, publishGlobalMap, publishDataFlowSummary, collectKeyframesFromSubmaps
//       getOutputDir, saveMapToFiles, stateToString, checkThreadHeartbeats
//       computeOdomInfoMatrix, computeOdomInfoMatrixForKeyframes

#ifndef AUTOMAP_PRO_SYSTEM_MODULES_TASKS_AND_UTILS_H_
#define AUTOMAP_PRO_SYSTEM_MODULES_TASKS_AND_UTILS_H_

#include "automap_pro/system/automap_system.h"

namespace automap_pro {

void AutoMapSystem::publishStatus();
void AutoMapSystem::publishGlobalMap();
void AutoMapSystem::publishDataFlowSummary();
std::vector<KeyFrame::Ptr> AutoMapSystem::collectKeyframesFromSubmaps(const std::vector<SubMap::Ptr>& submaps);
std::string AutoMapSystem::getOutputDir() const;
void AutoMapSystem::saveMapToFiles(const std::string& output_dir);
std::string AutoMapSystem::stateToString(SystemState s) const;
void AutoMapSystem::checkThreadHeartbeats();
Mat66d AutoMapSystem::computeOdomInfoMatrix(
    const SubMap::Ptr& prev,
    const SubMap::Ptr& curr,
    const Pose3d& rel) const;
Mat66d AutoMapSystem::computeOdomInfoMatrixForKeyframes(
    const KeyFrame::Ptr& prev_kf,
    const KeyFrame::Ptr& curr_kf,
    const Pose3d& rel) const;

}  // namespace automap_pro

#endif  // AUTOMAP_PRO_SYSTEM_MODULES_TASKS_AND_UTILS_H_
