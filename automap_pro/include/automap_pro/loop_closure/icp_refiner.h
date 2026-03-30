#pragma once
/**
 * @file icp_refiner.h
 * @brief 点云配准（ICP / GICP / NDT）与语义加权刚性对齐的接口声明。
 *
 * @details
 * 坐标约定：@c Pose3d 将源点从源坐标系变到目标坐标系，即对点 @f$\mathbf{p}@f$ 有
 * @f$\mathbf{p}_{tgt} = \mathbf{R}\mathbf{p}_{src} + \mathbf{t}@f$（与 Eigen::Isometry3d 一致）。
 */
#include "automap_pro/core/data_types.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <unordered_set>
#include <vector>

namespace automap_pro {

/**
 * @brief 配准后端枚举。
 *
 * @details
 * - ICP（点对点）：最小化 @f$\sum_i \|\mathbf{R}\mathbf{p}_i + \mathbf{t} - \mathbf{q}_i\|^2@f$。
 * - Point-to-Plane：最小化 @f$\sum_i \big(\mathbf{n}_i^\top (\mathbf{R}\mathbf{p}_i + \mathbf{t} - \mathbf{q}_i)\big)^2@f$，
 *   @f$\mathbf{n}_i@f$ 为目标侧法向。
 * - GICP：在对应点上使用协方差加权马氏距离（PCL Generalized ICP 实现）。
 * - NDT：将目标体素化为高斯分布，最大化源点落在目标分布下的对数似然；体素边长见 Config::ndt_resolution。
 */
enum class ICPMethod {
    ICP,            ///< 标准点对点 ICP（PCL IterativeClosestPoint）
    POINT_TO_PLANE, ///< 点到面 ICP（需目标有法向；本实现委托 PCL 默认行为）
    GICP,           ///< 广义 ICP（PCL GeneralizedIterativeClosestPoint）
    NDT             ///< 正态分布变换（PCL NormalDistributionsTransform）
};

/**
 * @class IcpRefiner
 * @brief 多算法点云配准与可选语义加权刚性迭代求精。
 *
 * @details
 * 多尺度路径：按 Config::scale_levels 体素降采样后逐层 refine()，最后在原始分辨率上再 refine() 一次。
 */
class IcpRefiner {
public:
    /** @brief 单次配准的输出统计。 */
    struct Result {
        bool   converged = false;       ///< PCL hasConverged()
        Pose3d T_refined = Pose3d::Identity(); ///< 求精后 @f$T_{src\to tgt}@f$
        double rmse = 1e6;              ///< 残差度量（语义路径为加权 RMS；其余见 PCL getFitnessScore）
        double fitness = 0.0;
        int    iterations = 0;
        double elapsed_ms = 0.0;
        int    correspondences = 0;     ///< 语义加权迭代最后一轮有效点对数

        double rotation_angle_deg = 0.0; ///< @f$\mathrm{angle}(\mathbf{R}_\delta)@f$，@f$\mathbf{T}_\delta = T_{init}^{-1} T_{final}@f$
        double translation_norm = 0.0; ///< @f$\|\mathbf{t}_\delta\|@f$
        double final_score = 0.0;
    };

    /** @brief 算法与阈值参数。数值含义与 PCL 对应 setter 一致。 */
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
        /**
         * @brief 灰区信任缩放：当标签占比 @f$r \in [r_{min}, 2r_{min})@f$ 时，
         *        令 @f$s = \mathrm{clamp}((r-r_{min})/r_{min}, 0, 1)@f$，对应点权
         *        @f$w \leftarrow s\,w + (1-s)@f$，使低覆盖时更接近纯几何 ICP。
         */
        bool semantic_gray_zone_linear_trust = true;
    };

    IcpRefiner();
    explicit IcpRefiner(const Config& config);
    ~IcpRefiner() = default;

    void setConfig(const Config& config);
    Config getConfig() const;

    /**
     * @brief 按 Config::method 选择算法的一次配准入口。
     * @param src 源点云（传感器/子图坐标系）
     * @param tgt 目标点云（固定参考系）
     * @param initial 初值 @f$T_{src\to tgt}@f$
     */
    Result refine(const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
                 const Pose3d& initial) const;

    /**
     * @brief 语义加权刚性迭代对齐（加权 Procrustes / SVD）。
     *
     * @details
     * 每步：对当前 @f$T@f$ 建立最近邻对应 @f$(\mathbf{p}_k, \mathbf{q}_k)@f$，权 @f$w_k>0@f$。
     * 闭式解最小化 @f$\sum_k w_k \|\mathbf{R}\mathbf{p}_k + \mathbf{t} - \mathbf{q}_k\|^2@f$：
     * @f$\bar{\mathbf{p}} = (\sum w_k)^{-1}\sum w_k \mathbf{p}_k@f$，
     * @f$\bar{\mathbf{q}}@f$ 同理；
     * @f$\mathbf{H} = \sum_k w_k (\mathbf{p}_k-\bar{\mathbf{p}})(\mathbf{q}_k-\bar{\mathbf{q}})^\top@f$；
     * SVD @f$\mathbf{H} = \mathbf{U}\boldsymbol{\Sigma}\mathbf{V}^\top@f$，@f$\mathbf{R} = \mathbf{V}\mathbf{U}^\top@f$
     *（若 @f$\det(\mathbf{R})<0@f$ 修正最后一列）；
     * @f$\mathbf{t} = \bar{\mathbf{q}} - \mathbf{R}\bar{\mathbf{p}}@f$。
     * intensity 解释为语义类 id；覆盖率不足时退回点到面 ICP。
     */
    Result refineSemanticWeighted(const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
                                  const Pose3d& initial) const;

    /** @brief 多尺度体素金字塔 + 最终全分辨率 refine()。 */
    Result refineMultiscale(const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
                           const Pose3d& initial) const;

    /**
     * @brief PCL ICP 封装。
     * @param point_to_plane true：点到面；false：点对点。
     */
    Result refineICP(const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
                    const Pose3d& initial, bool point_to_plane = false) const;
    Result refineGICP(const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
                     const Pose3d& initial) const;
    Result refineNDT(const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
                    const Pose3d& initial) const;

    /**
     * @brief 后验几何合理性检查（角、平移、启发式内点率）。
     * @details 内点率使用 @f$1 - \mathrm{rmse}/d_{max}@f$ 的简化下界，非严格概率解释。
     */
    bool validateResult(const Result& res, const CloudXYZIPtr& src,
                       const CloudXYZIPtr& tgt) const;

    /**
     * @brief 对已有对应求误差二阶矩对角块（平移分量），用于粗不确定性可视化/调试。
     * @details 对每对 @f$(i,j)@f$，@f$\mathbf{e} = \mathbf{R}\mathbf{p}_i - \mathbf{q}_j@f$，
     *          累加 @f$\mathrm{cov}(k,k) += e_k^2@f$，再除以对数。
     */
    Mat66d computeCorrespondenceCovariance(
        const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
        const Pose3d& T_src_tgt) const;

private:
    Config config_;

    static int intensityToSemanticLabel(float intensity);
    static double semanticLabelRatio(const CloudXYZIPtr& cloud);
    double semanticCorrespondenceWeight(int ls, int lt,
                                        const std::unordered_set<int>& dynamic_set) const;

    CloudXYZIPtr preprocessCloud(const CloudXYZIPtr& cloud) const;
    void computeNormals(const CloudXYZIPtr& cloud) const;

    /** @brief 将 src 用 @f$T@f$ 变到 tgt 系后做最近邻，距离阈值 @f$d < d_{max}@f$。 */
    std::vector<std::pair<int, int>> findCorrespondences(
        const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
        const Pose3d& T_src_tgt, float max_dist) const;
    /** @brief 距离比过滤：@f$\|\mathbf{R}\mathbf{p}_i+\mathbf{t}-\mathbf{q}_j\| \le \rho \|\mathbf{p}_i\|@f$。 */
    void filterCorrespondences(
        std::vector<std::pair<int, int>>& correspondences,
        const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
        const Pose3d& T_src_tgt) const;

    /** @brief 对齐云到 tgt 的最近邻距离算术平均。 */
    double computeFitnessScore(const CloudXYZIPtr& aligned,
                             const CloudXYZIPtr& tgt) const;
    /** @brief @f$\mathbf{T}_\delta = T_i^{-1} T_f@f$，输出角轴角（弧度转度）与平移范数。 */
    void computePoseChange(const Pose3d& T_initial, const Pose3d& T_final,
                          double& angle_deg, double& translation) const;
};

} // namespace automap_pro
