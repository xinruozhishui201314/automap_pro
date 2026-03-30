#pragma once
/**
 * @file overlap_transformer_infer.h
 * @brief OverlapTransformer 描述子：Range Image → TorchScript @f$\Rightarrow@f$ 256-D L2 归一化向量；检索用余弦相似度。
 *
 * @details
 * 相似度：对单位范数向量 @f$\mathbf{q},\mathbf{d}@f$，@f$s=\mathbf{q}^\top\mathbf{d}@f$（等价于余弦，因 @f$\|\mathbf{q}\|=\|\mathbf{d}\|=1@f$）。
 * 无 LibTorch 时走 FPFH 直方图等 fallback（见实现）。
 */
#include "automap_pro/core/data_types.h"
#include "automap_pro/core/ort_wrapper.h"
#include <string>
#include <mutex>

// LibTorch 前向声明（避免非 USE_TORCH 环境编译错误）
#ifdef USE_TORCH
#include <torch/script.h>
#include <torch/torch.h>
#endif

namespace automap_pro {

/**
 * OverlapTransformer C++ LibTorch 推理器
 *
 * 推理流程：
 *   PCL PointCloud → Range Image (H×W float32) → Tensor [1,1,H,W]
 *   → TorchScript module.forward() → Tensor [1,256] → L2 normalize → VectorXf
 *
 * 三级降级策略：
 *   Level 1: LibTorch 进程内 GPU/CPU 推理（<5ms）
 *   Level 2: 外部 Python Service（由 LoopDetector 负责，此类不处理）
 *   Level 3: FPFH 全局直方图 fallback（<2ms）
 *
 * 模型来源：由 OverlapTransformer-master/OT_libtorch/gen_libtorch_model.py 生成
 *   python gen_libtorch_model.py  →  overlapTransformer.pt
 */
class OverlapTransformerInfer {
public:
    OverlapTransformerInfer();
    ~OverlapTransformerInfer() = default;

    /**
     * 加载 TorchScript 模型（.pt 文件）
     * @param model_path  overlapTransformer.pt 路径
     * @param use_cuda    是否使用 CUDA 推理
     * @return  true=加载成功，false=失败（将使用 fallback）
     */
    bool loadModel(const std::string& model_path, bool use_cuda = true);

    bool isModelLoaded() const { return model_loaded_; }

    /** 编译时是否启用 LibTorch（便于运行日志中确认编译状态：1=可加载 .pt，0=仅 fallback） */
    static constexpr bool builtWithTorch() {
#ifdef USE_TORCH
        return true;
#else
        return false;
#endif
    }

    /**
     * 计算点云的 256 维全局描述子
     * 自动选择 LibTorch 推理或 fallback
     */
    Eigen::VectorXf computeDescriptor(const CloudXYZIPtr& cloud) const;

    /**
     * 检索候选子图（基于余弦相似度）
     * @param query_desc     查询描述子
     * @param db_submaps     数据库子图
     * @param top_k          返回 Top-K 候选
     * @param threshold      相似度阈值
     * @param min_submap_gap 最小子图间隔（同 session）
     * @param min_time_gap   最小时间间隔（秒，同 session）
     * @param gps_radius_m   GPS 半径过滤（米，-1 表示不过滤）
     * @param query_gps_pos  查询子图 GPS 中心
     * @param query_has_gps  是否有有效 GPS
     */
    struct Candidate {
        int      submap_id;
        uint64_t session_id;
        float    score;
    };
    std::vector<Candidate> retrieve(
        const Eigen::VectorXf& query_desc,
        const std::vector<std::shared_ptr<SubMap>>& db_submaps,
        int    top_k,
        float  threshold,
        int    min_submap_gap,
        double min_time_gap,
        double gps_radius_m,
        const Eigen::Vector3d& query_gps_pos,
        bool   query_has_gps) const;

    // 统计信息
    double lastInferenceMs() const { return last_infer_ms_; }
    int    totalInferences() const { return total_inferences_; }
    bool   isUsingGPU()      const { return use_cuda_ && model_loaded_; }

private:
    // ── LibTorch 模型 ─────────────────────────────────────────────────────
#ifdef USE_TORCH
    mutable torch::jit::script::Module model_;
#endif
    OnnxSession::Ptr onnx_model_ = nullptr;
    bool model_loaded_ = false;
    mutable bool use_cuda_ = false;

    // ── Range Image 参数 ──────────────────────────────────────────────────
    int   proj_H_    = 64;
    int   proj_W_    = 900;
    float fov_up_    = 3.0f;      // 度
    float fov_down_  = -25.0f;    // 度
    float max_range_ = 50.0f;     // 米

    // ── 统计 ──────────────────────────────────────────────────────────────
    mutable double last_infer_ms_ = 0.0;
    mutable int    total_inferences_ = 0;
    mutable bool   first_inference_logged_ = false;  // 仅首次推理时打一次状态日志
    mutable std::mutex mutex_;

    // ── 私有方法 ──────────────────────────────────────────────────────────

    /**
     * 生成范围图（参考 OT_libtorch/ws/fast_ot.cpp gen_range_image）
     * 输出：H×W float32 数组，值为归一化深度，-1=无效
     */
    std::vector<float> generateRangeImage(const CloudXYZIPtr& cloud) const;

#ifdef USE_TORCH
    /** LibTorch 推理（Level 1） */
    Eigen::VectorXf inferWithTorch(const std::vector<float>& range_img) const;
#endif

    /** ONNX/TensorRT 推理 */
    Eigen::VectorXf inferWithOnnx(const std::vector<float>& range_img) const;

    /** FPFH 直方图 fallback 描述子（Level 3） */
    Eigen::VectorXf computeFallbackDescriptor(const CloudXYZIPtr& cloud) const;
};

} // namespace automap_pro
