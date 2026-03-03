#include "automap_pro/loop_closure/overlap_transformer_infer.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#define MOD "OT_Infer"

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace automap_pro {

OverlapTransformerInfer::OverlapTransformerInfer() {
    const auto& cfg = ConfigManager::instance();
    proj_H_    = cfg.rangeImageH();
    proj_W_    = cfg.rangeImageW();
    fov_up_    = cfg.fovUp();
    fov_down_  = cfg.fovDown();
    max_range_ = cfg.maxRange();
}

bool OverlapTransformerInfer::loadModel(const std::string& model_path, bool use_cuda) {
#ifdef USE_TORCH
    std::lock_guard<std::mutex> lk(mutex_);
    if (model_path.empty()) {
        model_loaded_ = false;
        return false;
    }
    try {
        // 加载 TorchScript .pt 模型（由 gen_libtorch_model.py 生成）
        model_ = torch::jit::load(model_path);
        model_.eval();

        // 设备选择
        use_cuda_ = use_cuda && torch::cuda::is_available();
        torch::Device device(use_cuda_ ? torch::kCUDA : torch::kCPU);
        model_.to(device);

        // 预热（消除首次推理延迟，维度：[1,1,H,W]）
        auto dummy = torch::zeros({1, 1, proj_H_, proj_W_}, torch::kFloat);
        if (use_cuda_) dummy = dummy.to(torch::kCUDA);
        {
            torch::NoGradGuard no_grad;
            model_.forward({dummy});
        }
        model_loaded_ = true;

        // 输出推理设备信息
        printf("[OT] Model loaded: %s (device=%s, H=%d W=%d)\n",
               model_path.c_str(), use_cuda_ ? "CUDA" : "CPU", proj_H_, proj_W_);
        return true;
    } catch (const c10::Error& e) {
        fprintf(stderr, "[OT] Failed to load model '%s': %s\n",
                model_path.c_str(), e.what());
        model_loaded_ = false;
        return false;
    } catch (const std::exception& e) {
        fprintf(stderr, "[OT] Exception loading model: %s\n", e.what());
        model_loaded_ = false;
        return false;
    }
#else
    (void)model_path;
    (void)use_cuda;
    model_loaded_ = false;
    printf("[OT] LibTorch not available – using fallback descriptor\n");
    return false;
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// 生成范围图（完整移植自 OT_libtorch/ws/fast_ot.cpp gen_range_image）
// ─────────────────────────────────────────────────────────────────────────────
std::vector<float> OverlapTransformerInfer::generateRangeImage(
    const CloudXYZIPtr& cloud) const
{
    const int len = proj_H_ * proj_W_;
    std::vector<float> img(len, -1.0f);

    const float fov_up_rad   = fov_up_   * static_cast<float>(M_PI) / 180.0f;
    const float fov_down_rad = fov_down_ * static_cast<float>(M_PI) / 180.0f;
    const float fov_range    = std::abs(fov_down_rad) + std::abs(fov_up_rad);

    for (const auto& pt : cloud->points) {
        float depth = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
        if (depth >= max_range_ || depth <= 0.0f) continue;

        // 水平角（负号使正前方在图像中间）
        float yaw   = -std::atan2(pt.y, pt.x);
        // 垂直角
        float pitch = std::asin(pt.z / depth);

        // 投影到像素坐标
        float proj_x = 0.5f * (yaw / static_cast<float>(M_PI) + 1.0f);  // [0,1]
        float proj_y = 1.0f - (pitch + std::abs(fov_down_rad)) / fov_range;

        proj_x *= proj_W_;
        proj_y *= proj_H_;

        int col = std::max(0, std::min(proj_W_ - 1, static_cast<int>(std::floor(proj_x))));
        int row = std::max(0, std::min(proj_H_ - 1, static_cast<int>(std::floor(proj_y))));

        int idx = row * proj_W_ + col;
        float old_depth = img[idx];
        // 取最近深度（same as fast_ot.cpp: if depth < old_depth）
        if (old_depth < 0.0f || depth < old_depth) {
            img[idx] = depth;
        }
    }
    return img;
}

// ─────────────────────────────────────────────────────────────────────────────
// LibTorch 推理（Level 1）
// ─────────────────────────────────────────────────────────────────────────────
#ifdef USE_TORCH
Eigen::VectorXf OverlapTransformerInfer::inferWithTorch(
    const std::vector<float>& range_img) const
{
    torch::Device device(use_cuda_ ? torch::kCUDA : torch::kCPU);

    // float 数组 → Tensor [1,1,H,W]，克隆一份避免悬空指针
    torch::Tensor t = torch::from_blob(
        const_cast<float*>(range_img.data()),
        {1, 1, proj_H_, proj_W_},
        torch::kFloat32).clone();

    if (use_cuda_) t = t.to(device);

    torch::Tensor result;
    {
        torch::NoGradGuard no_grad;
        result = model_.forward({t}).toTensor();
    }

    // L2 归一化（参考 OverlapTransformer 原始实现）
    result = torch::nn::functional::normalize(
        result,
        torch::nn::functional::NormalizeFuncOptions().p(2).dim(1));

    if (use_cuda_) result = result.to(torch::kCPU);
    result = result.squeeze();   // [256]

    // Tensor → Eigen::VectorXf
    return Eigen::Map<Eigen::VectorXf>(result.data_ptr<float>(), result.numel());
}
#endif

// ─────────────────────────────────────────────────────────────────────────────
// FPFH 全局直方图 fallback（Level 3）
// ─────────────────────────────────────────────────────────────────────────────
Eigen::VectorXf OverlapTransformerInfer::computeFallbackDescriptor(
    const CloudXYZIPtr& cloud) const
{
    const int dim = 256;
    Eigen::VectorXf desc = Eigen::VectorXf::Zero(dim);
    if (!cloud || cloud->empty()) return desc;

    // 使用深度直方图（与 OverlapTransformer Python fallback 一致）
    auto range_img = generateRangeImage(cloud);
    int bins = dim;
    float bin_width = max_range_ / bins;

    for (float d : range_img) {
        if (d <= 0.0f) continue;
        int bin = std::min(bins - 1, static_cast<int>(d / bin_width));
        desc(bin) += 1.0f;
    }

    // L2 归一化
    float norm = desc.norm();
    if (norm > 1e-6f) desc /= norm;
    return desc;
}

// ─────────────────────────────────────────────────────────────────────────────
// 主入口：计算描述子
// ─────────────────────────────────────────────────────────────────────────────
Eigen::VectorXf OverlapTransformerInfer::computeDescriptor(
    const CloudXYZIPtr& cloud) const
{
    auto t0 = std::chrono::steady_clock::now();
    Eigen::VectorXf desc;

#ifdef USE_TORCH
    if (model_loaded_) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto range_img = generateRangeImage(cloud);
        desc = inferWithTorch(range_img);
    } else {
        desc = computeFallbackDescriptor(cloud);
    }
#else
    desc = computeFallbackDescriptor(cloud);
#endif

    auto t1 = std::chrono::steady_clock::now();
    last_infer_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
    total_inferences_++;
    if (last_infer_ms_ > 50.0) {
        ALOG_WARN(MOD, "Descriptor compute SLOW: {:.1f}ms (total_calls={})", last_infer_ms_, total_inferences_);
    } else {
        ALOG_DEBUG(MOD, "Descriptor computed in {:.1f}ms (mode={}, total={})",
                   last_infer_ms_, model_loaded_ ? "LibTorch" : "fallback", total_inferences_);
    }
    return desc;
}

// ─────────────────────────────────────────────────────────────────────────────
// 候选检索（余弦相似度 Top-K + 多重过滤）
// ─────────────────────────────────────────────────────────────────────────────
std::vector<OverlapTransformerInfer::Candidate> OverlapTransformerInfer::retrieve(
    const Eigen::VectorXf& query_desc,
    const std::vector<std::shared_ptr<SubMap>>& db_submaps,
    int    top_k,
    float  threshold,
    int    min_submap_gap,
    double min_time_gap,
    double gps_radius_m,
    const Eigen::Vector3d& query_gps_pos,
    bool   query_has_gps) const
{
    std::vector<std::pair<float, int>> scored;

    for (int i = 0; i < (int)db_submaps.size(); ++i) {
        const auto& sm = db_submaps[i];
        if (!sm->has_descriptor) continue;

        // ① 余弦相似度
        float score = query_desc.dot(sm->overlap_descriptor) /
                      (query_desc.norm() * sm->overlap_descriptor.norm() + 1e-8f);
        if (score < threshold) continue;

        // ② 子图间隔过滤（同 session 才检查）
        // (由 LoopDetector 在 processSubmap 中用 query->id 检查)

        // ③ GPS 半径过滤（双方都有 GPS 时才用）
        if (gps_radius_m > 0.0 && query_has_gps && sm->has_valid_gps) {
            double dist = (query_gps_pos - sm->gps_center).norm();
            if (dist > gps_radius_m) continue;
        }

        scored.push_back({score, i});
    }

    // Top-K 排序
    std::partial_sort(scored.begin(),
                      scored.begin() + std::min(top_k, (int)scored.size()),
                      scored.end(),
                      [](const auto& a, const auto& b) { return a.first > b.first; });

    std::vector<Candidate> result;
    for (int i = 0; i < std::min(top_k, (int)scored.size()); ++i) {
        const auto& sm = db_submaps[scored[i].second];
        result.push_back({sm->id, sm->session_id, scored[i].first});
    }
    return result;
}

} // namespace automap_pro
