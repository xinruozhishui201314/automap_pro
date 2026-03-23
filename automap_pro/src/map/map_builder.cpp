#include "automap_pro/map/map_builder.h"
#include "automap_pro/core/logger.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <algorithm>

namespace fs = std::filesystem;
#define MOD "MapBuilder"

namespace automap_pro {

MapBuilder::MapBuilder()
    : MapBuilder(Config()) {
}

MapBuilder::MapBuilder(const Config& config)
    : config_(config)
    , global_map_(std::make_shared<GlobalMap>())
    , base_map_(new CloudXYZI)
    , coarse_map_(new CloudXYZI)
    , fine_map_(new CloudXYZI)
    , running_(true)
    , build_pending_(false)
    , processed_kf_count_(0)
    , processed_sm_count_(0)
    , frame_since_cleanup_(0)
{
    // 配置全局地图
    GlobalMap::Config gm_config;
    gm_config.voxel_size = config_.base_voxel_size;
    gm_config.max_distance = config_.max_map_range;
    gm_config.max_points = 5000000;
    gm_config.enable_background_update = config_.enable_background_update;
    global_map_->setConfig(gm_config);
    
    ALOG_INFO(MOD, "MapBuilder initialized: base_vox={:.3f}m coarse={:.3f}m fine={:.3f}m",
              config_.base_voxel_size, config_.coarse_voxel_size, config_.fine_voxel_size);
    
    if (config_.enable_background_update) {
        build_thread_ = std::thread(&MapBuilder::buildThreadLoop, this);
        ALOG_INFO(MOD, "Background build thread started");
    }
}

MapBuilder::~MapBuilder() {
    running_ = false;
    
    if (build_thread_.joinable()) {
        // 发送空任务唤醒线程
        {
            std::lock_guard<std::mutex> lk(mutex_);
            BuildTask empty_task;
            build_queue_.push(empty_task);
            build_cv_.notify_one();
        }
        build_thread_.join();
    }
    
    ALOG_INFO(MOD, "MapBuilder destroyed: processed {} KFs, {} submaps",
              processed_kf_count_.load(), processed_sm_count_.load());
}

void MapBuilder::setConfig(const Config& config) {
    std::lock_guard<std::mutex> lk(mutex_);
    config_ = config;
    
    // 更新全局地图配置
    GlobalMap::Config gm_config = global_map_->getConfig();
    gm_config.voxel_size = config_.base_voxel_size;
    gm_config.max_distance = config_.max_map_range;
    global_map_->setConfig(gm_config);
}

MapBuilder::Config MapBuilder::getConfig() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return config_;
}

// ─────────────────────────────────────────────────────────────────────────────
// 地图构建
// ─────────────────────────────────────────────────────────────────────────────

void MapBuilder::buildFromKeyFrames(const std::vector<KeyFrame::Ptr>& keyframes) {
    if (keyframes.empty()) return;
    
    ALOG_INFO(MOD, "Building map from {} keyframes", keyframes.size());
    
    buildFromKeyFramesImpl(keyframes);
    updateMultiResolutionMaps();
    
    if (config_.enable_cleanup) {
        cleanupMaps();
    }
    
    ALOG_INFO(MOD, "Map built from {} keyframes: base={} coarse={} fine={} pts",
              keyframes.size(), base_map_->size(), coarse_map_->size(), fine_map_->size());
}

void MapBuilder::buildFromSubmaps(const std::vector<SubMap::Ptr>& submaps) {
    if (submaps.empty()) return;
    
    ALOG_INFO(MOD, "Building map from {} submaps", submaps.size());
    
    buildFromSubmapsImpl(submaps);
    updateMultiResolutionMaps();
    
    if (config_.enable_cleanup) {
        cleanupMaps();
    }
    
    ALOG_INFO(MOD, "Map built from {} submaps: base={} coarse={} fine={} pts",
              submaps.size(), base_map_->size(), coarse_map_->size(), fine_map_->size());
}

void MapBuilder::buildAsyncFromKeyFrames(const std::vector<KeyFrame::Ptr>& keyframes) {
    if (keyframes.empty()) return;

    BuildTask task;
    task.keyframes = keyframes;

    {
        std::lock_guard<std::mutex> lk(mutex_);

        // 队列大小限制：丢弃最旧的任务，保留最新的
        while (build_queue_.size() >= static_cast<size_t>(config_.max_queue_size)) {
            build_queue_.pop();
            ALOG_WARN(MOD, "Build queue overflow (size={}), dropping oldest task", config_.max_queue_size);
        }

        build_queue_.push(task);
        build_pending_ = true;
        build_cv_.notify_one();
    }

    ALOG_DEBUG(MOD, "Queued async build: {} keyframes (queue size: {})",
               keyframes.size(), build_queue_.size());
}

// ─────────────────────────────────────────────────────────────────────────────
// 增量更新
// ─────────────────────────────────────────────────────────────────────────────

void MapBuilder::addKeyFrame(const KeyFrame::Ptr& kf) {
    if (!kf) return;
    
    addKeyFrameImpl(kf);
    
    processed_kf_count_++;
    frame_since_cleanup_++;
    
    // 定期更新多分辨率地图
    if (processed_kf_count_ % 10 == 0) {
        updateMultiResolutionMaps();
    }
    
    // 定期清理
    if (config_.enable_cleanup && frame_since_cleanup_ >= config_.cleanup_interval) {
        cleanupMaps();
        frame_since_cleanup_ = 0;
    }
    
    ALOG_DEBUG(MOD, "Added keyframe #{} to map, total: {} KFs",
               kf->id, processed_kf_count_.load());
}

void MapBuilder::addSubmap(const SubMap::Ptr& sm) {
    if (!sm) return;
    
    addSubmapImpl(sm);
    
    processed_sm_count_++;
    frame_since_cleanup_++;
    
    // 定期更新多分辨率地图
    if (processed_sm_count_ % 5 == 0) {
        updateMultiResolutionMaps();
    }
    
    // 定期清理
    if (config_.enable_cleanup && frame_since_cleanup_ >= config_.cleanup_interval) {
        cleanupMaps();
        frame_since_cleanup_ = 0;
    }
    
    ALOG_DEBUG(MOD, "Added submap #{} to map, total: {} submaps",
               sm->id, processed_sm_count_.load());
}

void MapBuilder::updateSubmapPose(int submap_id, const Pose3d& new_pose) {
    // 子图位姿更新时不再清空全局地图，避免性能抖动与数据丢失。
    // 若需反映新位姿，由调用方在适当时机调用 buildFromSubmaps() 重新构建。
    (void)new_pose;
    ALOG_DEBUG(MOD, "Submap #{} pose updated (map not cleared; call buildFromSubmaps to refresh)", submap_id);
}

// ─────────────────────────────────────────────────────────────────────────────
// 获取地图
// ─────────────────────────────────────────────────────────────────────────────

std::shared_ptr<GlobalMap> MapBuilder::getGlobalMap() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return global_map_;
}

CloudXYZIPtr MapBuilder::getBaseMap() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return base_map_;
}

CloudXYZIPtr MapBuilder::getCoarseMap() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return coarse_map_;
}

CloudXYZIPtr MapBuilder::getFineMap() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return fine_map_;
}

void MapBuilder::waitForBuild() {
    while (build_pending_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 地图清理
// ─────────────────────────────────────────────────────────────────────────────

void MapBuilder::removeOutliers() {
    std::lock_guard<std::mutex> lk(mutex_);
    
    if (base_map_->empty()) return;
    
    size_t old_size = base_map_->size();
    
    // 统计去噪
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(base_map_);
    sor.setMeanK(config_.outlier_knn);
    sor.setStddevMulThresh(config_.outlier_std_mul);
    
    CloudXYZIPtr filtered(new CloudXYZI);
    sor.filter(*filtered);
    
    base_map_ = filtered;
    
    ALOG_INFO(MOD, "Outlier removal: {} -> {} points", old_size, base_map_->size());
}

void MapBuilder::removeDuplicatePoints(float distance_threshold) {
    std::lock_guard<std::mutex> lk(mutex_);
    
    if (base_map_->empty()) return;
    
    size_t old_size = base_map_->size();
    
    // 体素化去重
    CloudXYZIPtr filtered(new CloudXYZI);
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(base_map_);
    vg.setLeafSize(distance_threshold, distance_threshold, distance_threshold);
    vg.filter(*filtered);
    
    base_map_ = filtered;
    
    ALOG_INFO(MOD, "Duplicate removal: {} -> {} points", old_size, base_map_->size());
}

void MapBuilder::cleanup() {
    std::lock_guard<std::mutex> lk(mutex_);
    
    cleanupMaps();
}

// ─────────────────────────────────────────────────────────────────────────────
// 统计信息
// ─────────────────────────────────────────────────────────────────────────────

int MapBuilder::getProcessedKeyFrames() const {
    return processed_kf_count_.load();
}

int MapBuilder::getProcessedSubmaps() const {
    return processed_sm_count_.load();
}

size_t MapBuilder::getBaseMapSize() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return base_map_->size();
}

size_t MapBuilder::getCoarseMapSize() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return coarse_map_->size();
}

size_t MapBuilder::getFineMapSize() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return fine_map_->size();
}

// ─────────────────────────────────────────────────────────────────────────────
// 回调
// ─────────────────────────────────────────────────────────────────────────────

void MapBuilder::registerProgressCallback(ProgressCallback cb) {
    std::lock_guard<std::mutex> lk(mutex_);
    progress_cbs_.push_back(std::move(cb));
}

void MapBuilder::registerBuildDoneCallback(BuildDoneCallback cb) {
    std::lock_guard<std::mutex> lk(mutex_);
    build_done_cbs_.push_back(std::move(cb));
}

// ─────────────────────────────────────────────────────────────────────────────
// 重置
// ─────────────────────────────────────────────────────────────────────────────

void MapBuilder::reset() {
    std::lock_guard<std::mutex> lk(mutex_);
    
    base_map_->clear();
    coarse_map_->clear();
    fine_map_->clear();
    global_map_->clear();
    
    processed_kf_count_ = 0;
    processed_sm_count_ = 0;
    frame_since_cleanup_ = 0;
    
    ALOG_INFO(MOD, "MapBuilder reset");
}

// ─────────────────────────────────────────────────────────────────────────────
// 后台构建循环
// ─────────────────────────────────────────────────────────────────────────────

void MapBuilder::buildThreadLoop() {
    while (running_.load()) {
        BuildTask task;
        {
            std::unique_lock<std::mutex> lk(mutex_);
            build_cv_.wait(lk, [this] {
                return !build_queue_.empty() || !running_.load();
            });
            
            if (!running_.load() && build_queue_.empty()) break;
            
            if (!build_queue_.empty()) {
                task = build_queue_.front();
                build_queue_.pop();
            }
        }
        
        processBuildTask(task);
        
        build_pending_ = false;
    }
}

void MapBuilder::processBuildTask(const BuildTask& task) {
    if (!task.keyframes.empty()) {
        ALOG_DEBUG(MOD, "Processing async build: {} keyframes", task.keyframes.size());
        buildFromKeyFramesImpl(task.keyframes);
        updateMultiResolutionMaps();
        
        if (config_.enable_cleanup) {
            cleanupMaps();
        }
        
        notifyBuildDoneCallbacks(true);
    } else if (!task.submaps.empty()) {
        ALOG_DEBUG(MOD, "Processing async build: {} submaps", task.submaps.size());
        buildFromSubmapsImpl(task.submaps);
        updateMultiResolutionMaps();
        
        if (config_.enable_cleanup) {
            cleanupMaps();
        }
        
        notifyBuildDoneCallbacks(true);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 地图构建实现
// ─────────────────────────────────────────────────────────────────────────────

void MapBuilder::buildFromKeyFramesImpl(const std::vector<KeyFrame::Ptr>& keyframes) {
    if (keyframes.empty()) return;
    
    std::lock_guard<std::mutex> lk(mutex_);
    
    size_t total = keyframes.size();
    size_t processed = 0;
    
    for (size_t i = 0; i < keyframes.size(); ++i) {
        const auto& kf = keyframes[i];
        
        if (!kf || !kf->cloud_body || kf->cloud_body->empty()) continue;
        
        // 变换到地图坐标系（统一使用优化后位姿，保证 GPS/回环/HBA 后地图与轨迹一致）
        CloudXYZI map_cloud;
        transformCloudToMap(kf->cloud_body, kf->T_map_b_optimized, map_cloud);
        
        // 合并到基础地图
        mergeCloud(std::make_shared<CloudXYZI>(map_cloud), base_map_);
        
        // 同时添加到全局地图
        global_map_->addCloud(kf->cloud_body, kf->T_map_b_optimized);
        
        processed++;
        
        if (processed % 10 == 0) {
            notifyProgressCallbacks(static_cast<int>(processed), static_cast<int>(total));
        }
    }
    
    notifyProgressCallbacks(static_cast<int>(total), static_cast<int>(total));
    
    ALOG_INFO(MOD, "Built map from {} keyframes: {} pts", processed, base_map_->size());
}

void MapBuilder::buildFromSubmapsImpl(const std::vector<SubMap::Ptr>& submaps) {
    if (submaps.empty()) return;
    
    std::lock_guard<std::mutex> lk(mutex_);
    
    for (const auto& sm : submaps) {
        if (!sm) continue;

        // 重要：不要直接复用 sm->merged_cloud。
        // merged_cloud 是冻结时用当时的 T_odom_b 变换得到的“局部系点云”，后端优化后会变“旧位姿”。
        // 这里统一从每个关键帧的 body 点云 + T_map_b_optimized 重投影，保证每次重建都反映最新位姿。
        if (sm->keyframes.empty()) continue;
        for (const auto& kf : sm->keyframes) {
            if (!kf || !kf->cloud_body || kf->cloud_body->empty()) continue;
            CloudXYZI map_cloud;
            transformCloudToMap(kf->cloud_body, kf->T_map_b_optimized, map_cloud);
            if (!map_cloud.empty()) {
                mergeCloud(std::make_shared<CloudXYZI>(map_cloud), base_map_);
            }
        }
    }
    
    ALOG_INFO(MOD, "Built map from {} submaps: {} pts", submaps.size(), base_map_->size());
}

void MapBuilder::addKeyFrameImpl(const KeyFrame::Ptr& kf) {
    if (!kf || !kf->cloud_body || kf->cloud_body->empty()) return;
    
    std::lock_guard<std::mutex> lk(mutex_);
    
    // 变换到地图坐标系
    CloudXYZI map_cloud;
    transformCloudToMap(kf->cloud_body, kf->T_map_b_optimized, map_cloud);
    
    // 合并到基础地图
    mergeCloud(std::make_shared<CloudXYZI>(map_cloud), base_map_);
    
    // 同时添加到全局地图
    global_map_->addCloud(kf->cloud_body, kf->T_map_b_optimized);
}

void MapBuilder::addSubmapImpl(const SubMap::Ptr& sm) {
    if (!sm || !sm->merged_cloud || sm->merged_cloud->empty()) return;
    
    std::lock_guard<std::mutex> lk(mutex_);
    
    // 直接使用子图的合并点云
    mergeCloud(sm->merged_cloud, base_map_);
    
    // 将子图点云添加到全局地图
    global_map_->addCloud(sm->downsampled_cloud, sm->pose_map_anchor_optimized);
}

// ─────────────────────────────────────────────────────────────────────────────
// 地图处理
// ─────────────────────────────────────────────────────────────────────────────

void MapBuilder::updateMultiResolutionMaps() {
    std::lock_guard<std::mutex> lk(mutex_);
    
    if (base_map_->empty()) return;
    
    ALOG_DEBUG(MOD, "Updating multi-resolution maps...");
    
    // 粗糙分辨率
    if (coarse_map_->empty() || base_map_->size() - coarse_map_->size() > 10000) {
        pcl::VoxelGrid<pcl::PointXYZI> vg_coarse;
        vg_coarse.setInputCloud(base_map_);
        vg_coarse.setLeafSize(config_.coarse_voxel_size, 
                             config_.coarse_voxel_size, 
                             config_.coarse_voxel_size);
        vg_coarse.filter(*coarse_map_);
    }
    
    // 精细分辨率
    if (fine_map_->empty() || base_map_->size() - fine_map_->size() > 5000) {
        pcl::VoxelGrid<pcl::PointXYZI> vg_fine;
        vg_fine.setInputCloud(base_map_);
        vg_fine.setLeafSize(config_.fine_voxel_size, 
                          config_.fine_voxel_size, 
                          config_.fine_voxel_size);
        vg_fine.filter(*fine_map_);
    }
    
    ALOG_DEBUG(MOD, "Multi-resolution maps updated: base={} coarse={} fine={} pts",
               base_map_->size(), coarse_map_->size(), fine_map_->size());
}

void MapBuilder::applyOutlierRemoval() {
    if (base_map_->size() < 1000) return;
    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(base_map_);
    sor.setMeanK(config_.outlier_knn);
    sor.setStddevMulThresh(config_.outlier_std_mul);
    
    CloudXYZIPtr filtered(new CloudXYZI);
    sor.filter(*filtered);
    
    size_t old_size = base_map_->size();
    base_map_ = filtered;
    
    ALOG_INFO(MOD, "Outlier removal: {} -> {} pts", old_size, base_map_->size());
}

void MapBuilder::applyDownsampling() {
    if (base_map_->size() < 1000) return;
    
    size_t old_size = base_map_->size();
    
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(base_map_);
    vg.setLeafSize(config_.base_voxel_size, config_.base_voxel_size, config_.base_voxel_size);
    vg.filter(*base_map_);
    
    ALOG_INFO(MOD, "Downsampling: {} -> {} pts", old_size, base_map_->size());
}

void MapBuilder::cleanupMaps() {
    ALOG_INFO(MOD, "Cleaning up maps...");
    
    applyOutlierRemoval();
    applyDownsampling();
    
    // 更新多分辨率地图
    updateMultiResolutionMaps();
    
    if (config_.save_intermediate) {
        saveIntermediateMap("_cleanup");
    }
    
    ALOG_INFO(MOD, "Map cleanup done");
}

// ─────────────────────────────────────────────────────────────────────────────
// 辅助函数
// ─────────────────────────────────────────────────────────────────────────────

void MapBuilder::transformCloudToMap(const CloudXYZIPtr& cloud, const Pose3d& T_map_b,
                                      CloudXYZI& output) const {
    Eigen::Affine3f T_f = T_map_b.cast<float>();
    pcl::transformPointCloud(*cloud, output, T_f);
}

void MapBuilder::mergeCloud(const CloudXYZIPtr& src, CloudXYZIPtr& dst) {
    *dst += *src;
}

void MapBuilder::saveIntermediateMap(const std::string& suffix) const {
    if (!config_.save_intermediate) return;
    
    try {
        fs::create_directories(config_.save_dir);
        
        std::string timestamp = std::to_string(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        
        std::string path = config_.save_dir + "/map_" + timestamp + suffix + ".pcd";
        
        pcl::PCDWriter writer;
        writer.writeBinary(path, *base_map_);
        
        ALOG_DEBUG(MOD, "Saved intermediate map: {}", path);
    } catch (const std::exception& e) {
        ALOG_WARN(MOD, "Failed to save intermediate map: {}", e.what());
    }
}

void MapBuilder::notifyProgressCallbacks(int processed, int total) {
    for (auto& cb : progress_cbs_) {
        cb(processed, total);
    }
}

void MapBuilder::notifyBuildDoneCallbacks(bool success) {
    for (auto& cb : build_done_cbs_) {
        cb(success);
    }
}

} // namespace automap_pro
