// 模块5: 回环与优化
// 包含: onLoopDetected, onPoseUpdated, onHBADone

#ifndef AUTOMAP_PRO_SYSTEM_MODULES_LOOP_OPTIMIZATION_H_
#define AUTOMAP_PRO_SYSTEM_MODULES_LOOP_OPTIMIZATION_H_

#include "automap_pro/system/automap_system.h"

namespace automap_pro {

// 回环与优化相关函数声明
void AutoMapSystem::onLoopDetected(const LoopConstraint::Ptr& lc);
void AutoMapSystem::onPoseUpdated(const OptimizationResult& res);
void AutoMapSystem::onHBADone(const HBAResult& result);

}  // namespace automap_pro

#endif  // AUTOMAP_PRO_SYSTEM_MODULES_LOOP_OPTIMIZATION_H_
