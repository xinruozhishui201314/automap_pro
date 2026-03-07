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
    std::vector<float> img(len, 0.0f);  // 初始化为0而非-1，便于后续补齐

    const float fov_up_rad   = fov_up_   * static_cast<float>(M_PI) / 180.0f;
    const float fov_down_rad = fov_down_ * static_cast<float>(M_PI) / 180.0f;
    const float fov_range    = std::abs(fov_down_rad) + std::abs(fov_up_rad);

    // ✅ 第一遍：将点投影到范围图（取最近深度）
    std::vector<bool> valid_pixel(len, false);  // 标记有效像素
    for (const auto& pt : cloud->points) {
        float depth = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
        if (depth >= max_range_ || depth <= 0.0f) continue;

        float yaw   = -std::atan2(pt.y, pt.x);
        float pitch = std::asin(pt.z / depth);

        float proj_x = 0.5f * (yaw / static_cast<float>(M_PI) + 1.0f);
        float proj_y = 1.0f - (pitch + std::abs(fov_down_rad)) / fov_range;

        proj_x *= proj_W_;
        proj_y *= proj_H_;

        int col = std::max(0, std::min(proj_W_ - 1, static_cast<int>(std::floor(proj_x))));
        int row = std::max(0, std::min(proj_H_ - 1, static_cast<int>(std::floor(proj_y))));

        int idx = row * proj_W_ + col;
        if (img[idx] <= 0.0f || depth < img[idx]) {
            img[idx] = depth;
            valid_pixel[idx] = true;
        }
    }

    // ✅ 第二遍：快速补齐稀疏像素（行列扫描，O(H×W) 而非 O(H×W×9)）
    // 水平传播（从左到右）
    for (int row = 0; row < proj_H_; ++row) {
        for (int col = 1; col < proj_W_; ++col) {
            int idx = row * proj_W_ + col;
            if (valid_pixel[idx]) continue;
            int left_idx = idx - 1;
            if (valid_pixel[left_idx] && img[left_idx] > 0.0f) {
                img[idx] = img[left_idx];
                valid_pixel[idx] = true;
            }
        }
    }
    // 水平传播（从右到左，补齐右侧空洞）
    for (int row = 0; row < proj_H_; ++row) {
        for (int col = proj_W_ - 2; col >= 0; --col) {
            int idx = row * proj_W_ + col;
            if (valid_pixel[idx]) continue;
            int right_idx = idx + 1;
            if (valid_pixel[right_idx] && img[right_idx] > 0.0f) {
                img[idx] = img[right_idx];
                valid_pixel[idx] = true;
            }
        }
    }
    
    // 垂直传播（从上到下）
    for (int col = 0; col < proj_W_; ++col) {
        for (int row = 1; row < proj_H_; ++row) {
            int idx = row * proj_W_ + col;
            if (valid_pixel[idx]) continue;
            int up_idx = idx - proj_W_;
            if (valid_pixel[up_idx] && img[up_idx] > 0.0f) {
                img[idx] = img[up_idx];
                valid_pixel[idx] = true;
            }
        }
    }
    // 垂直传播（从下到上，补齐下侧空洞）
    for (int col = 0; col < proj_W_; ++col) {
        for (int row = proj_H_ - 2; row >= 0; --row) {
            int idx = row * proj_W_ + col;
            if (valid_pixel[idx]) continue;
            int down_idx = idx + proj_W_;
            if (valid_pixel[down_idx] && img[down_idx] > 0.0f) {
                img[idx] = img[down_idx];
                valid_pixel[idx] = true;
            }
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

    // 先拷贝到局部缓冲区，避免多线程下 range_img 被修改导致悬空指针
    std::vector<float> range_img_copy(range_img.begin(), range_img.end());
    torch::Tensor t = torch::from_blob(
        range_img_copy.data(),
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
    auto t_start = std::chrono::steady_clock::now();
    
    auto t_phase = std::chrono::steady_clock::now();
    
    Eigen::VectorXf desc;

#ifdef USE_TORCH
    if (model_loaded_) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto range_img = generateRangeImage(cloud);
        auto t_range = std::chrono::steady_clock::now();
        double range_ms = std::chrono::duration<double, std::milli>(t_range - t_phase).count();
        
        desc = inferWithTorch(range_img);
        auto t_infer = std::chrono::steady_clock::now();
        double infer_ms = std::chrono::duration<double, std::milli>(t_infer - t_range).count();
        
        if (range_ms > 10.0 || infer_ms > 20.0) {
            ALOG_WARN(MOD, "[PERF] Range generation: {:.1f}ms, Inference: {:.1f}ms", 
                      range_ms, infer_ms);
        }
    } else {
        desc = computeFallbackDescriptor(cloud);
    }
#else
    desc = computeFallbackDescriptor(cloud);
#endif

    auto t_norm = std::chrono::steady_clock::now();
    // ✅ 验证 L2 归一化
    float norm = desc.norm();
    if (norm > 1e-6f) {
        desc /= norm;
    } else {
        ALOG_WARN(MOD, "Descriptor norm too small ({:.6f}), using uniform fallback", norm);
        desc = Eigen::VectorXf::Constant(256, 1.0f / std::sqrt(256.0f));
    }

    auto t_end = std::chrono::steady_clock::now();
    last_infer_ms_ = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    total_inferences_++;
    
    if (last_infer_ms_ > 30.0) {
        ALOG_WARN(MOD, "[PERF] Descriptor compute SLOW: {:.1f}ms (target: <20ms for 50Hz) pts={}", 
                  last_infer_ms_, cloud ? cloud->size() : 0);
    } else if (total_inferences_ % 50 == 0) {
        // 每50次输出一次平均耗时
        ALOG_DEBUG(MOD, "[PERF] Avg descriptor: {:.1f}ms (calls={})",
                   last_infer_ms_, total_inferences_);
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
    
    // ✅ 预计算 query norm，避免重复计算
    float query_norm_cache = query_desc.norm();
    if (query_norm_cache < 1e-6f) {
        query_norm_cache = 1.0f;  // 保底值
    }

    for (int i = 0; i < (int)db_submaps.size(); ++i) {
        const auto& sm = db_submaps[i];
        if (!sm->has_descriptor) continue;

        // ① 余弦相似度（使用缓存的 norm 值）
        float score = query_desc.dot(sm->overlap_descriptor) /
                      (query_norm_cache * sm->overlap_descriptor_norm + 1e-8f);
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
