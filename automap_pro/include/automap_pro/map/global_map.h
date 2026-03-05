#pragma once

#include <memory>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>

#include <thread>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "automap_pro/core/data_types.h"

namespace automap_pro {

// ─────────────────────────────────────────────────────────────────────────────
// GlobalMap: 高效全局点云地图管理
// 特性：
//   - 增量式更新，支持后台异步更新
//   - KD树加速空间查询
//   - 自适应降采样控制内存
//   - 支持ROI查询（半径/包围盒）
// ─────────────────────────────────────────────────────────────────────────────

class GlobalMap {
public:
    struct Config {
        float voxel_size = 0.1f;           // 降采样体素大小
        float max_distance = 200.0f;        // 最大距离（超出丢弃）
        int   max_points = 5000000;         // 最大点数（超出触发降采样）
        bool  enable_background_update = true;  // 启用后台异步更新
        int   update_batch_size = 10000;    // 批量更新点数
        int   kd_tree_leaf_size = 10;       // KD树叶子大小
    };

    using UpdateCallback = std::function<void(int total_points)>;
    
    GlobalMap();
    explicit GlobalMap(const Config& config);
    ~GlobalMap();

    // 配置
    void setConfig(const Config& config);
    Config getConfig() const;

    // 增量更新
    void addCloud(const CloudXYZIPtr& cloud, const Pose3d& T_w_b = Pose3d::Identity());
    void addCloudAsync(const CloudXYZIPtr& cloud, const Pose3d& T_w_b = Pose3d::Identity());
    
    // 等待后台更新完成
    void waitForUpdate();

    // 获取全局地图
    CloudXYZIPtr getMap(float downsample_voxel = -1.0f) const;
    CloudXYZIPtr getMapROI(const Eigen::Vector3d& center, float radius, 
                         float downsample_voxel = -1.0f) const;
    CloudXYZIPtr getMapBBox(const Eigen::Vector3d& min_pt, const Eigen::Vector3d& max_pt,
                           float downsample_voxel = -1.0f) const;

    // 空间查询
    std::vector<int> queryRadius(const Eigen::Vector3d& center, float radius) const;
    std::vector<int> queryKNN(const Eigen::Vector3d& center, int k) const;
    std::vector<Eigen::Vector3d> queryKNNPositions(const Eigen::Vector3d& center, int k) const;
    
    // 地图统计
    int getPointCount() const;
    Eigen::Vector3d getMapMin() const;
    Eigen::Vector3d getMapMax() const;
    Eigen::Vector3d getMapCenter() const;
    double getMapRadius() const;

    // 清理
    void clear();
    void clearROI(const Eigen::Vector3d& center, float radius);
    void keepROI(const Eigen::Vector3d& center, float radius);

    // 持久化
    bool savePCD(const std::string& path, bool binary = true) const;
    bool loadPCD(const std::string& path);

    // 回调
    void registerUpdateCallback(UpdateCallback cb);

    // 内存管理
    void optimizeMemory();  // 降采样减少内存
    size_t getMemorySizeMB() const;

private:
    struct UpdateTask {
        CloudXYZIPtr cloud;
        Pose3d pose;
    };

    mutable std::mutex mutex_;
    
    Config config_;
    
    CloudXYZIPtr global_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kd_tree_;
    bool kd_tree_needs_update_ = true;
    
    std::queue<UpdateTask> update_queue_;
    std::condition_variable update_cv_;  // 条件变量，用于通知后台线程
    std::thread update_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> update_pending_;
    
    std::vector<UpdateCallback> update_cbs_;
    
    // 地图边界缓存
    Eigen::Vector3d min_pt_ = Eigen::Vector3d::Constant(1e6);
    Eigen::Vector3d max_pt_ = Eigen::Vector3d::Constant(-1e6);
    
    // 后台更新线程
    void updateThreadLoop();
    void processUpdateTask(const UpdateTask& task);
    
    // KD树管理
    void ensureKDTree() const;
    void updateKDTree();
    void buildKDTree();
    
    // 辅助函数
    void transformCloud(const CloudXYZIPtr& input, const Pose3d& T,
                      CloudXYZI& output) const;
    void downsampleCloud(CloudXYZIPtr& cloud, float voxel_size) const;
    void updateMapBounds(const CloudXYZIPtr& cloud);
    void notifyUpdateCallbacks();
};

} // namespace automap_pro
