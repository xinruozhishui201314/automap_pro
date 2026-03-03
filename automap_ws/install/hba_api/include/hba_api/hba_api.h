#pragma once

/**
 * HBA (Hierarchical Bundle Adjustment) C++ API
 *
 * 直接调用 HBA 核心优化算法，无需文件 I/O，无需 std::system() 进程调用。
 *
 * 使用方式（automap_pro 后端调用示例）：
 *   #include <hba_api/hba_api.h>
 *
 *   hba_api::Config cfg;
 *   cfg.total_layer_num = 3;
 *   cfg.thread_num      = 8;
 *   cfg.enable_gps      = false;
 *
 *   hba_api::HBAOptimizer opt(cfg);
 *   opt.addKeyFrame(pose_q, pose_t, cloud_ptr);
 *   // ...
 *   hba_api::Result result = opt.optimize();
 *   if (result.success) {
 *       for (size_t i = 0; i < result.optimized_poses.size(); i++)
 *           kf[i]->T_w_b_optimized = result.optimized_poses[i];
 *   }
 */

#include <vector>
#include <memory>
#include <functional>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace hba_api {

using PointType = pcl::PointXYZINormal;
using CloudPtr  = pcl::PointCloud<PointType>::Ptr;

// ─────────────────────────────────────────────────────────────────────────────
// 配置
// ─────────────────────────────────────────────────────────────────────────────
struct Config {
    int    total_layer_num = 3;   // 分层数（典型值 2~4）
    int    thread_num      = 8;   // 并行线程数
    double voxel_size      = 0.5; // 体素大小（米）
    double eigen_ratio     = 0.1; // 平面判定阈值（λ_min/λ_max < eigen_ratio）
    int    win_size        = 10;  // 滑动窗口帧数
    int    gap             = 1;   // 窗口步长
    bool   enable_gps      = false;

    // GPS 数据（enable_gps=true 时使用）
    struct GPSEntry {
        double timestamp;
        double lat, lon, alt;
    };
    std::vector<GPSEntry> gps_entries;
};

// ─────────────────────────────────────────────────────────────────────────────
// 关键帧输入
// ─────────────────────────────────────────────────────────────────────────────
struct KeyFrameInput {
    double             timestamp;
    Eigen::Quaterniond rotation;
    Eigen::Vector3d    translation;
    CloudPtr           cloud;     // body 系下点云（PointXYZINormal）
};

// ─────────────────────────────────────────────────────────────────────────────
// 优化结果
// ─────────────────────────────────────────────────────────────────────────────
struct Result {
    bool success = false;
    std::vector<Eigen::Isometry3d> optimized_poses; // 与输入 keyframes 顺序一致
    double final_mme       = 0.0;  // Mean Map Entropy
    double elapsed_ms      = 0.0;
    int    total_keyframes = 0;
    std::string error_msg;
};

// ─────────────────────────────────────────────────────────────────────────────
// 进度回调（逐层完成时调用）
// ─────────────────────────────────────────────────────────────────────────────
using ProgressCallback = std::function<void(int current_layer, int total_layers,
                                             float progress_percent)>;

// ─────────────────────────────────────────────────────────────────────────────
// HBA 优化器（主接口）
// ─────────────────────────────────────────────────────────────────────────────
class HBAOptimizer {
public:
    explicit HBAOptimizer(const Config& cfg);
    ~HBAOptimizer();

    /** 逐帧添加关键帧（位姿 + 点云） */
    void addKeyFrame(const KeyFrameInput& kf);

    /** 批量设置关键帧（替换已有） */
    void setKeyFrames(const std::vector<KeyFrameInput>& kfs);

    /** 清空已有关键帧 */
    void clear();

    /** 当前关键帧数量 */
    size_t keyFrameCount() const;

    /**
     * 执行 HBA 优化（阻塞，但在内部使用多线程）
     * @param progress_cb  进度回调（可为空）
     * @return 优化结果
     */
    Result optimize(ProgressCallback progress_cb = nullptr);

    /**
     * 计算 Mean Map Entropy（地图质量评估指标）
     * 值越小 = 地图越精准（平面越"扁"）
     */
    static double computeMME(const std::vector<KeyFrameInput>& kfs,
                              const std::vector<Eigen::Isometry3d>& poses,
                              double voxel_size = 0.5);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace hba_api
