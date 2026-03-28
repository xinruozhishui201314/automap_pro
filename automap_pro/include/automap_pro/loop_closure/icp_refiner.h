#pragma once
#include "automap_pro/core/data_types.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <unordered_set>
#include <vector>

namespace automap_pro {

// ─────────────────────────────────────────────────────────────────────────────
// IcpRefiner: 点云配准优化器
// 支持多种配准算法：ICP、GICP、NDT
// 功能：
//   - 点到面ICP（Point-to-Plane ICP）
//   - 广义ICP（Generalized ICP）考虑点云协方差
//   - NDT（Normal Distributions Transform）
//   - 多尺度策略（从粗到精）
//   - 对应关系可视化（调试用）
// ─────────────────────────────────────────────────────────────────────────────

enum class ICPMethod {
    ICP,           // 标准ICP（Point-to-Point）
    POINT_TO_PLANE, // 点到面ICP（Point-to-Plane ICP）
    GICP,          // 广义ICP（Generalized ICP）
    NDT            // 正态分布变换（NDT）
};

class IcpRefiner {
public:
    struct Result {
        bool   converged = false;
        Pose3d T_refined = Pose3d::Identity();
        double rmse = 1e6;
        double fitness = 0.0;
        int    iterations = 0;
        double elapsed_ms = 0.0;
        int    correspondences = 0;
        
        // 详细统计
        double rotation_angle_deg = 0.0;
        double translation_norm = 0.0;
        double final_score = 0.0;
    };

    struct Config {
        ICPMethod method = ICPMethod::POINT_TO_PLANE;
        
        int    max_iterations = 50;
        double max_correspondence_distance = 0.5;
        double transformation_epsilon = 1e-8;
        double euclidean_fitness_epsilon = 1e-8;
        
        // GICP专用参数
        double gicp_correspondence_radius = 1.0;
        int    gicp_correspondence_knn = 10;
        
        // NDT专用参数
        double ndt_resolution = 1.0;
        double ndt_step_size = 0.1;
        double ndt_resolution_gradient = 0.1;
        
        // 多尺度策略
        bool  enable_multiscale = false;
        std::vector<float> scale_levels = {0.5f, 0.25f, 0.1f};
        
        // 对应关系过滤
        bool  enable_correspondence_filtering = true;
        double max_distance_ratio = 2.0;  // 最大距离比
        double max_angle_deg = 45.0;      // 最大角度差
        
        // 输入预处理
        bool  downsample_before_refine = true;
        float  downsample_voxel_size = 0.2f;  // 至少 0.2 避免 PCL 体素溢出
        
        // 验证
        bool  validate_result = true;
        double max_rotation_deg = 30.0;
        double max_translation_m = 5.0;
        double min_inlier_ratio = 0.3;

        // 预处理细粒度开关（回环点云已下采样时建议 false，且语义 ICP 需保留 intensity=label）
        bool preprocess_downsample = true;
        bool preprocess_sor = true;

        // ── SuMa++ 类语义加权 ICP：PointXYZI.intensity 存语义类 id，0=未知
        bool semantic_icp_enabled = false;
        double semantic_min_label_ratio = 0.06;
        float semantic_weight_match = 1.0f;
        float semantic_weight_mismatch = 0.18f;
        float semantic_weight_unknown = 0.5f;
        float semantic_weight_dynamic = 0.08f;
        int semantic_max_iterations = 40;
        std::vector<int> semantic_dynamic_class_ids;
        /** true：标签占比 ∈ [min_ratio, 2*min_ratio) 时线性降低语义权重、向几何 ICP 靠拢 */
        bool semantic_gray_zone_linear_trust = true;
    };

    IcpRefiner();
    explicit IcpRefiner(const Config& config);
    ~IcpRefiner() = default;

    // 配置
    void setConfig(const Config& config);
    Config getConfig() const;

    // 配准（主要接口）
    Result refine(const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
                 const Pose3d& initial) const;

    /**
     * 语义加权点对点迭代（加权 Procrustes）。要求 src/tgt 的 intensity 为类别 id。
     * 若标签覆盖率不足则内部退回 refineICP(..., point_to_plane)。
     */
    Result refineSemanticWeighted(const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
                                  const Pose3d& initial) const;
    
    // 多尺度配准
    Result refineMultiscale(const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
                           const Pose3d& initial) const;
    
    // 不同算法的配准
    Result refineICP(const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
                    const Pose3d& initial, bool point_to_plane = false) const;
    Result refineGICP(const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
                     const Pose3d& initial) const;
    Result refineNDT(const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
                    const Pose3d& initial) const;
    
    // 验证配准结果
    bool validateResult(const Result& res, const CloudXYZIPtr& src, 
                       const CloudXYZIPtr& tgt) const;
    
    // 统计信息
    Mat66d computeCorrespondenceCovariance(
        const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
        const Pose3d& T_src_tgt) const;

private:
    Config config_;

    static int intensityToSemanticLabel(float intensity);
    static double semanticLabelRatio(const CloudXYZIPtr& cloud);
    double semanticCorrespondenceWeight(int ls, int lt,
                                        const std::unordered_set<int>& dynamic_set) const;
    
    // 预处理
    CloudXYZIPtr preprocessCloud(const CloudXYZIPtr& cloud) const;
    void computeNormals(const CloudXYZIPtr& cloud) const;
    
    // 对应关系分析
    std::vector<std::pair<int, int>> findCorrespondences(
        const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
        const Pose3d& T_src_tgt, float max_dist) const;
    void filterCorrespondences(
        std::vector<std::pair<int, int>>& correspondences,
        const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
        const Pose3d& T_src_tgt) const;
    
    // 结果分析
    double computeFitnessScore(const CloudXYZIPtr& aligned, 
                             const CloudXYZIPtr& tgt) const;
    void computePoseChange(const Pose3d& T_initial, const Pose3d& T_final,
                          double& angle_deg, double& translation) const;
};

} // namespace automap_pro
