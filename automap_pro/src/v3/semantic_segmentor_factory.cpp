#include "automap_pro/v3/semantic_segmentor_factory.h"

#include <stdexcept>

namespace automap_pro::v3 {

std::unique_ptr<ISemanticSegmentor> CreateSloamSemanticSegmentor(const SegmentorConfig& cfg);
std::unique_ptr<ISemanticSegmentor> CreateLsk3dnetSemanticSegmentor(const SegmentorConfig& cfg);
std::unique_ptr<ISemanticSegmentor> CreateLsk3dnetHybridSemanticSegmentor(const SegmentorConfig& cfg);
std::unique_ptr<ISemanticSegmentor> CreateNoopGeometricSemanticSegmentor(const SegmentorConfig& cfg);

std::unique_ptr<ISemanticSegmentor> SemanticSegmentorFactory::Create(const SegmentorConfig& cfg) {
    if (cfg.model_type == "noop_geometric") {
        return CreateNoopGeometricSemanticSegmentor(cfg);
    }
    if (cfg.model_type == "sloam") {
        return CreateSloamSemanticSegmentor(cfg);
    }
    if (cfg.model_type == "lsk3dnet") {
        return CreateLsk3dnetSemanticSegmentor(cfg);
    }
    if (cfg.model_type == "lsk3dnet_hybrid") {
        return CreateLsk3dnetHybridSemanticSegmentor(cfg);
    }
    throw std::runtime_error("Unsupported semantic.model_type: " + cfg.model_type);
}

}  // namespace automap_pro::v3
