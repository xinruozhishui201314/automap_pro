// 模块4: 关键帧与子图管理
// 包含: tryCreateKeyFrame, onSubmapFrozen, transformWorldToBody

#ifndef AUTOMAP_PRO_SYSTEM_MODULES_KEYFRAME_SUBMAP_H_
#define AUTOMAP_PRO_SYSTEM_MODULES_KEYFRAME_SUBMAP_H_

#include "automap_pro/system/automap_system.h"

namespace automap_pro {

// 关键帧与子图管理相关函数声明
void AutoMapSystem::tryCreateKeyFrame(double ts);
void AutoMapSystem::tryCreateKeyFrame(double ts, const Pose3d& pose, const Mat66d& cov,
                                       const CloudXYZIPtr& cloud,
                                       const LivoKeyFrameInfo* optional_livo_info,
                                       const CloudXYZIPtr* optional_cloud_ds);
CloudXYZIPtr AutoMapSystem::transformWorldToBody(const CloudXYZIPtr& world_cloud, const Pose3d& T_w_b) const;
void AutoMapSystem::onSubmapFrozen(const SubMap::Ptr& submap);

}  // namespace automap_pro

#endif  // AUTOMAP_PRO_SYSTEM_MODULES_KEYFRAME_SUBMAP_H_
