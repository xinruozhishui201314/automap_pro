#pragma once

#include <memory>
#include <vector>
#include <mutex>
#include <functional>
#include <queue>
#include <thread>
#include <atomic>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "automap_pro/core/data_types.h"
#include "automap_pro/map/global_map.h"

namespace automap_pro {

// ─────────────────────────────────────────────────────────────────────────────
// MapBuilder: 增量式地图构建器
// 功能：
//   - 从关键帧/子图增量构建地图
//   - 支持多分辨率地图
//   - 后台异步更新
//   - 动态地图清理（去重、去噪）
// ─────────────────────────────────────────────────────────────────────────────

class MapBuilder {
public:
    struct Config {
        float base_voxel_size = 0.05f;       // 基础分辨率
        float coarse_voxel_size = 0.2f;       // 粗糙分辨率
        float fine_voxel_size = 0.02f;        // 精细分辨率

        int   outlier_knn = 20;               // 统计去噪KNN数
        float outlier_std_mul = 2.0f;          // 统计去噪标准差倍数

        bool  enable_background_update = true;   // 后台异步更新
        int   update_batch_size = 50;         // 批量更新关键帧数
        int   max_queue_size = 100;           // 最大队列大小（防止内存溢出）

        float max_map_range = 300.0f;          // 最大地图范围
        bool  enable_cleanup = true;            // 启用地图清理
        int   cleanup_interval = 100;           // 清理间隔（帧数）

        bool  save_intermediate = false;       // 保存中间地图
        std::string save_dir = "/tmp/automap/maps";
    };

    using ProgressCallback = std::function<void(int processed, int total)>;
    using BuildDoneCallback = std::function<void(bool success)>;
    
    MapBuilder();
    explicit MapBuilder(const Config& config);
    ~MapBuilder();

    // 配置
    void setConfig(const Config& config);
    Config getConfig() const;

    // 地图构建
    void buildFromKeyFrames(const std::vector<KeyFrame::Ptr>& keyframes);
    void buildFromSubmaps(const std::vector<SubMap::Ptr>& submaps);
    void buildAsyncFromKeyFrames(const std::vector<KeyFrame::Ptr>& keyframes);
    
    // 增量更新
    void addKeyFrame(const KeyFrame::Ptr& kf);
    void addSubmap(const SubMap::Ptr& sm);
    void updateSubmapPose(int submap_id, const Pose3d& new_pose);
    
    // 获取地图
    std::shared_ptr<GlobalMap> getGlobalMap() const;
    CloudXYZIPtr getBaseMap() const;
    CloudXYZIPtr getCoarseMap() const;
    CloudXYZIPtr getFineMap() const;
    
    // 等待构建完成
    void waitForBuild();
    
    // 地图清理
    void removeOutliers();
    void removeDuplicatePoints(float distance_threshold = 0.01f);
    void cleanup();
    
    // 统计信息
    int getProcessedKeyFrames() const;
    int getProcessedSubmaps() const;
    size_t getBaseMapSize() const;
    size_t getCoarseMapSize() const;
    size_t getFineMapSize() const;
    
    // 回调
    void registerProgressCallback(ProgressCallback cb);
    void registerBuildDoneCallback(BuildDoneCallback cb);
    
    // 重置
    void reset();

private:
    struct BuildTask {
        std::vector<KeyFrame::Ptr> keyframes;
        std::vector<SubMap::Ptr> submaps;
    };

    mutable std::mutex mutex_;
    
    Config config_;
    
    // 多分辨率地图
    std::shared_ptr<GlobalMap> global_map_;
    CloudXYZIPtr base_map_;
    CloudXYZIPtr coarse_map_;
    CloudXYZIPtr fine_map_;
    
    // 后台构建线程
    std::queue<BuildTask> build_queue_;
    std::thread build_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> build_pending_;
    
    // 统计
    std::atomic<int> processed_kf_count_;
    std::atomic<int> processed_sm_count_;
    std::atomic<int> frame_since_cleanup_;
    
    // 回调
    std::vector<ProgressCallback> progress_cbs_;
    std::vector<BuildDoneCallback> build_done_cbs_;
    
    // 后台构建循环
    void buildThreadLoop();
    void processBuildTask(const BuildTask& task);
    
    // 地图构建实现
    void buildFromKeyFramesImpl(const std::vector<KeyFrame::Ptr>& keyframes);
    void buildFromSubmapsImpl(const std::vector<SubMap::Ptr>& submaps);
    void addKeyFrameImpl(const KeyFrame::Ptr& kf);
    void addSubmapImpl(const SubMap::Ptr& sm);
    
    // 地图处理
    void updateMultiResolutionMaps();
    void applyOutlierRemoval();
    void applyDownsampling();
    void cleanupMaps();
    
    // 辅助函数
    void transformCloudToWorld(const CloudXYZIPtr& cloud, const Pose3d& T_w_b,
                              CloudXYZIPtr& output) const;
    void mergeCloud(const CloudXYZIPtr& src, CloudXYZIPtr& dst);
    void saveIntermediateMap(const std::string& suffix) const;
    void notifyProgressCallbacks(int processed, int total);
    void notifyBuildDoneCallbacks(bool success);
};

} // namespace automap_pro
