/**
 * @file system/automap_system_component_plugin.cpp
 * @brief 系统节点与 ROS 服务实现。
 */
// 仅此编译单元注册 composable node，避免在 automap_system.h 中放 RCLCPP_COMPONENTS_REGISTER_NODE
// （否则每个 include 该头的 .cpp 都会静态注册一次，与链接了 libautomap_system_component 的可执行文件叠加后触发 class_loader 冲突警告）。
#include <rclcpp_components/register_node_macro.hpp>
#include "automap_pro/system/automap_system.h"

RCLCPP_COMPONENTS_REGISTER_NODE(automap_pro::AutoMapSystem)
