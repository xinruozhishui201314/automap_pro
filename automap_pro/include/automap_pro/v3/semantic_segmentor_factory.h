#pragma once

#include "automap_pro/v3/semantic_segmentor.h"
#include <memory>

namespace automap_pro::v3 {

class SemanticSegmentorFactory {
public:
    static std::unique_ptr<ISemanticSegmentor> Create(const SegmentorConfig& cfg);
};

}  // namespace automap_pro::v3
