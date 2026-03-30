#pragma once
/**
 * @file v3/semantic_segmentor.h
 * @brief V3 微内核：模块编排、事件总线、Registry、前端/语义/优化流水线。
 */


#include "automap_pro/core/data_types.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace automap_pro::v3 {

struct SegmentorConfig {
    std::string model_type = "lsk3dnet";
    std::string model_path;
    std::string lsk3dnet_model_path;
    std::string lsk3dnet_device = "cpu";
    /** lsk3dnet_hybrid: Python(spconv 骨干) + LibTorch(classifier TorchScript)，见 thrid_party/LSK3DNet-main/scripts/ */
    std::string lsk3dnet_repo_root;
    std::string lsk3dnet_config_yaml;
    std::string lsk3dnet_checkpoint;
    std::string lsk3dnet_classifier_torchscript;
    std::string lsk3dnet_python_exe = "python3";
    std::string lsk3dnet_worker_script;
    /** 与 LSK 训练时 dataset2.compute_normals_range 默认一致；树掩膜 fov 可与此不同 */
    std::string lsk3dnet_hybrid_normal_mode = "range";  // range | zeros
    float lsk3dnet_normal_fov_up_deg = 3.0f;
    float lsk3dnet_normal_fov_down_deg = -25.0f;
    int lsk3dnet_normal_proj_h = 64;
    int lsk3dnet_normal_proj_w = 900;
    float fov_up = 22.5f;
    float fov_down = -22.5f;
    int img_w = 2048;
    int img_h = 64;
    /** test_skitti 风格：对固定增强视角的 classifier logits 做 index_add 式累加后再 argmax；1=关闭 TTA */
    int lsk_num_vote = 1;
    /** 与训练 dataloader 一致：仅对 dataset_params 体积盒内点跑骨干/分类，盒外点语义标为 0 */
    bool lsk_training_volume_crop = false;
    bool lsk_volume_bounds_valid = false;
    float lsk_vol_min_x = 0.f;
    float lsk_vol_min_y = 0.f;
    float lsk_vol_min_z = 0.f;
    float lsk_vol_max_x = 0.f;
    float lsk_vol_max_y = 0.f;
    float lsk_vol_max_z = 0.f;
    int input_channels = 0;
    int num_classes = 0;
    int tree_class_id = -1;
    std::vector<float> input_mean;
    std::vector<float> input_std;
    bool do_destagger = true;
    int batch_size = 1;
};

struct SemanticSegResult {
    bool success = false;
    std::string backend_name;
    std::string message;
    double inference_ms = 0.0;
    /**
     * 与 run(cloud,...) 的 cloud->points 顺序一一对应（通常即 SemanticProcessor 的 flipped 点云）。
     * 非空时语义标签来自骨干+分类头逐点 argmax，不经过 range mask 的「每像素最后一次写入」坍缩。
     * 下游应优先用此构造稠密 labeled / 树干子集；mask 仍保留供 RViz 与 Trellis organized 路径。
     */
    std::vector<uint8_t> per_point_labels;
};

class ISemanticSegmentor {
public:
    virtual ~ISemanticSegmentor() = default;

    virtual const char* name() const = 0;
    virtual bool isReady() const = 0;
    virtual void run(const CloudXYZIConstPtr& cloud, cv::Mat& mask, SemanticSegResult* result) = 0;
    virtual void maskCloud(const CloudXYZIConstPtr& cloud, const cv::Mat& mask, CloudXYZIPtr& out_cloud,
                           int tree_label, bool dense_for_clustering) = 0;
};

// Process-wide shutdown guard for hybrid semantic worker recovery path.
// SemanticModule sets this flag before fatal shutdown to block worker restart
// during teardown and avoid exit-phase races.
void SetSemanticHybridShutdownGuard(bool enabled);
bool IsSemanticHybridShutdownGuard();

}  // namespace automap_pro::v3
