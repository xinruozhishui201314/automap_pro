// 模块2: 数据流回调
// 包含: onOdometry, onCloud, onKFInfo, onGPS, odomCacheAdd, odomCacheGet, kfinfoCacheAdd, kfinfoCacheGet

#ifndef AUTOMAP_PRO_SYSTEM_MODULES_DATA_CALLBACKS_H_
#define AUTOMAP_PRO_SYSTEM_MODULES_DATA_CALLBACKS_H_

#include "automap_pro/system/automap_system.h"

namespace automap_pro {

// 数据流回调相关函数声明
void AutoMapSystem::onOdometry(double ts, const Pose3d& pose, const Mat66d& cov);
void AutoMapSystem::onCloud(double ts, const CloudXYZIPtr& cloud);
void AutoMapSystem::onKFInfo(const LivoKeyFrameInfo& info);
void AutoMapSystem::onGPS(double ts, double lat, double lon, double alt, double hdop, int sats);
void AutoMapSystem::odomCacheAdd(double ts, const Pose3d& pose, const Mat66d& cov);
bool AutoMapSystem::odomCacheGet(double ts, Pose3d& out_pose, Mat66d& out_cov);
void AutoMapSystem::kfinfoCacheAdd(double ts, const LivoKeyFrameInfo& info);
bool AutoMapSystem::kfinfoCacheGet(double ts, LivoKeyFrameInfo& out_info);

}  // namespace automap_pro

#endif  // AUTOMAP_PRO_SYSTEM_MODULES_DATA_CALLBACKS_H_
