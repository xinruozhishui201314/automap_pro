#pragma once

#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/data_types.h"

namespace automap_pro {

inline int gpsQualityRank(GPSQuality q) {
    return static_cast<int>(q);
}

inline bool gpsQualityAcceptedByPolicy(GPSQuality q, const ConfigManager& cfg) {
    return gpsQualityRank(q) >= cfg.gpsMinAcceptedQualityLevel();
}

}  // namespace automap_pro
