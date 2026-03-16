// 模块6: GPS处理
// 包含: onGPSAligned, transformAllPosesAfterGPSAlign, addBatchGPSFactors, ensureBackendCompletedAndFlushBeforeHBA

#ifndef AUTOMAP_PRO_SYSTEM_MODULES_GPS_PROCESSING_H_
#define AUTOMAP_PRO_SYSTEM_MODULES_GPS_PROCESSING_H_

#include "automap_pro/system/automap_system.h"

namespace automap_pro {

void AutoMapSystem::onGPSAligned(const GPSAlignResult& result);
void AutoMapSystem::transformAllPosesAfterGPSAlign(const GPSAlignResult& result);
void AutoMapSystem::addBatchGPSFactors();
void AutoMapSystem::ensureBackendCompletedAndFlushBeforeHBA();

}  // namespace automap_pro

#endif  // AUTOMAP_PRO_SYSTEM_MODULES_GPS_PROCESSING_H_
