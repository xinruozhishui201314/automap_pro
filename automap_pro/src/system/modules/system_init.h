// 模块1: 构造与初始化
// 包含: AutoMapSystem 构造函数、析构函数、loadConfigAndInit、setupModules、deferredSetupModules、setupPublishers、setupServices、setupTimers

#ifndef AUTOMAP_PRO_SYSTEM_MODULES_SYSTEM_INIT_H_
#define AUTOMAP_PRO_SYSTEM_MODULES_SYSTEM_INIT_H_

#include "automap_pro/system/automap_system.h"

namespace automap_pro {

// 构造与初始化相关函数声明
void AutoMapSystem::loadConfigAndInit();
void AutoMapSystem::setupModules();
void AutoMapSystem::deferredSetupModules();
void AutoMapSystem::setupPublishers();
void AutoMapSystem::setupServices();
void AutoMapSystem::setupTimers();

}  // namespace automap_pro

#endif  // AUTOMAP_PRO_SYSTEM_MODULES_SYSTEM_INIT_H_
