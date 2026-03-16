// 模块7: 服务处理器
// 包含: handleSaveMap, handleGetStatus, handleTriggerHBA, handleTriggerOptimize, handleTriggerGpsAlign, handleLoadSession, handleFinishMapping

#ifndef AUTOMAP_PRO_SYSTEM_MODULES_SERVICE_HANDLERS_H_
#define AUTOMAP_PRO_SYSTEM_MODULES_SERVICE_HANDLERS_H_

#include "automap_pro/system/automap_system.h"

namespace automap_pro {

void AutoMapSystem::handleSaveMap(
    const std::shared_ptr<automap_pro::srv::SaveMap::Request> req,
    std::shared_ptr<automap_pro::srv::SaveMap::Response> res);
void AutoMapSystem::handleGetStatus(
    const std::shared_ptr<automap_pro::srv::GetStatus::Request> req,
    std::shared_ptr<automap_pro::srv::GetStatus::Response> res);
void AutoMapSystem::handleTriggerHBA(
    const std::shared_ptr<automap_pro::srv::TriggerHBA::Request> req,
    std::shared_ptr<automap_pro::srv::TriggerHBA::Response> res);
void AutoMapSystem::handleTriggerOptimize(
    const std::shared_ptr<automap_pro::srv::TriggerOptimize::Request> req,
    std::shared_ptr<automap_pro::srv::TriggerOptimize::Response> res);
void AutoMapSystem::handleTriggerGpsAlign(
    const std::shared_ptr<automap_pro::srv::TriggerGpsAlign::Request> req,
    std::shared_ptr<automap_pro::srv::TriggerGpsAlign::Response> res);
void AutoMapSystem::handleLoadSession(
    const std::shared_ptr<automap_pro::srv::LoadSession::Request> req,
    std::shared_ptr<automap_pro::srv::LoadSession::Response> res);
void AutoMapSystem::handleFinishMapping(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

}  // namespace automap_pro

#endif  // AUTOMAP_PRO_SYSTEM_MODULES_SERVICE_HANDLERS_H_
