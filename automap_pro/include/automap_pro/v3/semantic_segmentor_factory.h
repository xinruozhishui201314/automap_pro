#pragma once
/**
 * @file v3/semantic_segmentor_factory.h
 * @brief V3 微内核：模块编排、事件总线、Registry、前端/语义/优化流水线。
 */


#include "automap_pro/v3/semantic_segmentor.h"
#include <memory>

namespace automap_pro::v3 {

class SemanticSegmentorFactory {
public:
    static std::unique_ptr<ISemanticSegmentor> Create(const SegmentorConfig& cfg);
};

}  // namespace automap_pro::v3
