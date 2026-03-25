#pragma once

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

}  // namespace automap_pro::v3
