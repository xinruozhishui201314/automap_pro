#pragma once
/**
 * @file submap/ms_mapping_wrapper.h
 * @brief 子图/会话：生命周期、合并、持久化与 MS-Mapping 桥接。
 */

// MS-Mapping wrapper: thin facade over SubMapManager + SessionManager
// For direct integration with the Fast-LIVO2 keyframe stream.
#include "automap_pro/submap/submap_manager.h"
#include "automap_pro/submap/session_manager.h"
