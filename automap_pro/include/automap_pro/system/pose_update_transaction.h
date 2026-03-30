#pragma once
/**
 * @file system/pose_update_transaction.h
 * @brief 系统节点：Composable 主控、帧处理、位姿事务。
 */


#include "automap_pro/core/data_types.h"
#include <cstdint>
#include <memory>
#include <string>

namespace automap_pro {

enum class PoseWriteMode {
    SUBMAP_DELTA_PROPAGATE = 0,
    KEYFRAME_ABSOLUTE_SET = 1,
    GLOBAL_RIGID_TRANSFORM = 2
};

enum class PoseFrameSemantics {
    MAP_LOCAL = 0,
    ENU_GLOBAL = 1
};

enum class PoseUpdateSource {
    ISAM2_FORCE_UPDATE = 0,
    GPS_ALIGN = 1,
    HBA_WRITEBACK = 2,
    LOOP_OPTIMIZATION = 3
};

struct PoseUpdateTransaction {
    uint64_t version_id = 0;
    PoseWriteMode mode = PoseWriteMode::KEYFRAME_ABSOLUTE_SET;
    PoseFrameSemantics frame = PoseFrameSemantics::MAP_LOCAL;
    PoseUpdateSource source = PoseUpdateSource::ISAM2_FORCE_UPDATE;
    OptimizationResult result;
    std::shared_ptr<HBAResult> hba_result;
};

inline const char* toString(PoseWriteMode mode) {
    switch (mode) {
        case PoseWriteMode::SUBMAP_DELTA_PROPAGATE: return "SUBMAP_DELTA_PROPAGATE";
        case PoseWriteMode::KEYFRAME_ABSOLUTE_SET: return "KEYFRAME_ABSOLUTE_SET";
        case PoseWriteMode::GLOBAL_RIGID_TRANSFORM: return "GLOBAL_RIGID_TRANSFORM";
    }
    return "UNKNOWN";
}

inline const char* toString(PoseFrameSemantics frame) {
    switch (frame) {
        case PoseFrameSemantics::MAP_LOCAL: return "MAP_LOCAL";
        case PoseFrameSemantics::ENU_GLOBAL: return "ENU_GLOBAL";
    }
    return "UNKNOWN";
}

inline const char* toString(PoseUpdateSource source) {
    switch (source) {
        case PoseUpdateSource::ISAM2_FORCE_UPDATE: return "ISAM2_FORCE_UPDATE";
        case PoseUpdateSource::GPS_ALIGN: return "GPS_ALIGN";
        case PoseUpdateSource::HBA_WRITEBACK: return "HBA_WRITEBACK";
        case PoseUpdateSource::LOOP_OPTIMIZATION: return "LOOP_OPTIMIZATION";
    }
    return "UNKNOWN";
}

}  // namespace automap_pro
