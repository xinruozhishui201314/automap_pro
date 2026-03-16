// 模块3: 工作线程
// 包含: feederLoop, backendWorkerLoop, mapPublishLoop, statusPublisherLoop
// 注意：loopOptThreadLoop 和 vizThreadLoop 已删除

#ifndef AUTOMAP_PRO_SYSTEM_MODULES_WORKER_THREADS_H_
#define AUTOMAP_PRO_SYSTEM_MODULES_WORKER_THREADS_H_

#include "automap_pro/system/automap_system.h"

namespace automap_pro {

// 工作线程相关函数声明
void AutoMapSystem::feederLoop();
void AutoMapSystem::backendWorkerLoop();
void AutoMapSystem::mapPublishLoop();
// void AutoMapSystem::loopOptThreadLoop();  // 已删除
// void AutoMapSystem::vizThreadLoop();      // 已删除
void AutoMapSystem::statusPublisherLoop();

}  // namespace automap_pro

#endif  // AUTOMAP_PRO_SYSTEM_MODULES_WORKER_THREADS_H_
