#include "automap_pro/map/global_map.h"
#include "automap_pro/core/logger.h"
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/extract_clusters.h>
#include <thread>
#include <algorithm>
#include <numeric>

#define MOD "GlobalMap"

namespace automap_pro {

GlobalMap::GlobalMap() 
    : GlobalMap(Config()) {
}

GlobalMap::GlobalMap(const Config& config)
    : config_(config)
    , global_cloud_(new CloudXYZI)
    , kd_tree_(new pcl::KdTreeFLANN<pcl::PointXYZI>)
    , running_(true)
    , update_pending_(false)
{
    ALOG_INFO(MOD, "GlobalMap initialized: voxel_size={}m max_points={} max_dist={}m",
              config_.voxel_size, config_.max_points, config_.max_distance);
    
    if (config_.enable_background_update) {
        update_thread_ = std::thread(&GlobalMap::updateThreadLoop, this);
    }
}

GlobalMap::~GlobalMap() {
    running_ = false;
    
    if (update_thread_.joinable()) {
        // 发送空任务唤醒线程
        {
            std::lock_guard<std::mutex> lk(mutex_);
            UpdateTask empty_task;
            empty_task.cloud = nullptr;
            update_queue_.push(empty_task);
        }
        update_thread_.join();
    }
}

void GlobalMap::setConfig(const Config& config) {
    std::lock_guard<std::mutex> lk(mutex_);
    config_ = config;
    kd_tree_needs_update_ = true;
}

GlobalMap::Config GlobalMap::getConfig() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return config_;
}

// ─────────────────────────────────────────────────────────────────────────────
// 增量更新（同步）
// ─────────────────────────────────────────────────────────────────────────────

void GlobalMap::addCloud(const CloudXYZIPtr& cloud, const Pose3d& T_w_b) {
    if (!cloud || cloud->empty()) return;
    
    std::lock_guard<std::mutex> lk(mutex_);
    
    // 变换到世界坐标系
    CloudXYZIPtr transformed(new CloudXYZI);
    transformCloud(cloud, T_w_b, *transformed);
    
    // 按距离过滤
    CloudXYZIPtr filtered(new CloudXYZI);
    filtered->reserve(transformed->size());
    
    for (const auto& pt : transformed->points) {
        double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if (dist <= config_.max_distance) {
            filtered->push_back(pt);
        }
    }
    
    if (filtered->empty()) return;
    
    // 合并到全局地图
    *global_cloud_ += *filtered;
    
    // 更新地图边界
    updateMapBounds(filtered);
    
    // 内存管理
    if (global_cloud_->size() > static_cast<size_t>(config_.max_points)) {
        downsampleCloud(global_cloud_, config_.voxel_size);
    }
    
    kd_tree_needs_update_ = true;
    
    ALOG_DEBUG(MOD, "Added {} pts to global map, total: {} pts", 
               filtered->size(), global_cloud_->size());
    
    notifyUpdateCallbacks();
}

// ─────────────────────────────────────────────────────────────────────────────
// 增量更新（异步，后台线程）
// ─────────────────────────────────────────────────────────────────────────────

void GlobalMap::addCloudAsync(const CloudXYZIPtr& cloud, const Pose3d& T_w_b) {
    if (!cloud || cloud->empty()) return;
    
    UpdateTask task;
    task.cloud = cloud;
    task.pose = T_w_b;
    
    {
        std::lock_guard<std::mutex> lk(mutex_);
        update_queue_.push(task);
        update_pending_ = true;
    }
    
    // 通知后台线程有新任务
    update_cv_.notify_one();
    
    ALOG_DEBUG(MOD, "Queued async update: {} pts", cloud->size());
}

void GlobalMap::waitForUpdate() {
    while (update_pending_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 后台更新线程
// ─────────────────────────────────────────────────────────────────────────────

void GlobalMap::updateThreadLoop() {
    while (running_.load()) {
        UpdateTask task;
        {
            std::unique_lock<std::mutex> lk(mutex_);
            // 使用 condition_variable 等待任务或停止信号
            update_cv_.wait(lk, [this] {
                return !update_queue_.empty() || !running_.load();
            });
            
            if (!running_.load() && update_queue_.empty()) break;
            
            if (!update_queue_.empty()) {
                task = update_queue_.front();
                update_queue_.pop();
            }
        }
        
        if (task.cloud) {
            processUpdateTask(task);
        }
        
        update_pending_.store(false);
    }
}

void GlobalMap::processUpdateTask(const UpdateTask& task) {
    if (!task.cloud || task.cloud->empty()) return;
    
    ALOG_DEBUG(MOD, "Processing async update: {} pts", task.cloud->size());
    
    std::lock_guard<std::mutex> lk(mutex_);
    
    // 变换到世界坐标系
    CloudXYZIPtr transformed(new CloudXYZI);
    transformCloud(task.cloud, task.pose, *transformed);
    
    // 按距离过滤
    CloudXYZIPtr filtered(new CloudXYZI);
    filtered->reserve(transformed->size());
    
    for (const auto& pt : transformed->points) {
        double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if (dist <= config_.max_distance) {
            filtered->push_back(pt);
        }
    }
    
    if (filtered->empty()) return;
    
    // 合并到全局地图
    *global_cloud_ += *filtered;
    
    // 更新地图边界
    updateMapBounds(filtered);
    
    // 内存管理
    if (global_cloud_->size() > static_cast<size_t>(config_.max_points)) {
        downsampleCloud(global_cloud_, config_.voxel_size);
    }
    
    kd_tree_needs_update_ = true;
    
    ALOG_DEBUG(MOD, "Async update done: total {} pts", global_cloud_->size());
    
    notifyUpdateCallbacks();
}

// ─────────────────────────────────────────────────────────────────────────────
// 获取全局地图
// ─────────────────────────────────────────────────────────────────────────────

CloudXYZIPtr GlobalMap::getMap(float downsample_voxel) const {
    std::lock_guard<std::mutex> lk(mutex_);
    
    if (global_cloud_->empty()) return nullptr;
    
    // 如果指定降采样
    if (downsample_voxel > 0) {
        CloudXYZIPtr output(new CloudXYZI);
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(global_cloud_);
        vg.setLeafSize(downsample_voxel, downsample_voxel, downsample_voxel);
        vg.filter(*output);
        return output;
    }
    
    return global_cloud_;
}

CloudXYZIPtr GlobalMap::getMapROI(const Eigen::Vector3d& center, 
                                   float radius, 
                                   float downsample_voxel) const {
    std::lock_guard<std::mutex> lk(mutex_);
    
    if (global_cloud_->empty()) return nullptr;
    
    // 使用KD树查询ROI内点
    ensureKDTree();
    
    std::vector<int> indices;
    std::vector<float> distances;
    
    pcl::PointXYZI query_pt;
    query_pt.x = center.x();
    query_pt.y = center.y();
    query_pt.z = center.z();
    query_pt.intensity = 0;
    
    kd_tree_->radiusSearch(query_pt, radius, indices, distances);
    
    if (indices.empty()) return nullptr;
    
    CloudXYZIPtr roi_cloud(new CloudXYZI);
    roi_cloud->reserve(indices.size());
    
    for (int idx : indices) {
        roi_cloud->push_back(global_cloud_->points[idx]);
    }
    
    // 降采样
    if (downsample_voxel > 0) {
        CloudXYZIPtr downsampled(new CloudXYZI);
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(roi_cloud);
        vg.setLeafSize(downsample_voxel, downsample_voxel, downsample_voxel);
        vg.filter(*downsampled);
        return downsampled;
    }
    
    return roi_cloud;
}

CloudXYZIPtr GlobalMap::getMapBBox(const Eigen::Vector3d& min_pt,
                                    const Eigen::Vector3d& max_pt,
                                    float downsample_voxel) const {
    std::lock_guard<std::mutex> lk(mutex_);
    
    if (global_cloud_->empty()) return nullptr;
    
    CloudXYZIPtr bbox_cloud(new CloudXYZI);
    bbox_cloud->reserve(global_cloud_->size());
    
    for (const auto& pt : global_cloud_->points) {
        if (pt.x >= min_pt.x() && pt.x <= max_pt.x() &&
            pt.y >= min_pt.y() && pt.y <= max_pt.y() &&
            pt.z >= min_pt.z() && pt.z <= max_pt.z()) {
            bbox_cloud->push_back(pt);
        }
    }
    
    if (bbox_cloud->empty()) return nullptr;
    
    // 降采样
    if (downsample_voxel > 0) {
        CloudXYZIPtr downsampled(new CloudXYZI);
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(bbox_cloud);
        vg.setLeafSize(downsample_voxel, downsample_voxel, downsample_voxel);
        vg.filter(*downsampled);
        return downsampled;
    }
    
    return bbox_cloud;
}

// ─────────────────────────────────────────────────────────────────────────────
// 空间查询
// ─────────────────────────────────────────────────────────────────────────────

std::vector<int> GlobalMap::queryRadius(const Eigen::Vector3d& center, 
                                        float radius) const {
    std::lock_guard<std::mutex> lk(mutex_);
    
    if (global_cloud_->empty()) return {};
    
    ensureKDTree();
    
    std::vector<int> indices;
    std::vector<float> distances;
    
    pcl::PointXYZI query_pt;
    query_pt.x = center.x();
    query_pt.y = center.y();
    query_pt.z = center.z();
    query_pt.intensity = 0;
    
    kd_tree_->radiusSearch(query_pt, radius, indices, distances);
    
    return indices;
}

std::vector<int> GlobalMap::queryKNN(const Eigen::Vector3d& center, int k) const {
    std::lock_guard<std::mutex> lk(mutex_);
    
    if (global_cloud_->empty()) return {};
    
    ensureKDTree();
    
    std::vector<int> indices(k);
    std::vector<float> distances(k);
    
    pcl::PointXYZI query_pt;
    query_pt.x = center.x();
    query_pt.y = center.y();
    query_pt.z = center.z();
    query_pt.intensity = 0;
    
    int found = kd_tree_->nearestKSearch(query_pt, k, indices, distances);
    indices.resize(found);
    
    return indices;
}

std::vector<Eigen::Vector3d> GlobalMap::queryKNNPositions(const Eigen::Vector3d& center,
                                                          int k) const {
    std::vector<Eigen::Vector3d> positions;
    positions.reserve(k);
    
    // ✅ 修复：在同一个锁作用域内完成所有操作（避免悬空引用）
    std::lock_guard<std::mutex> lk(mutex_);
    
    // 检查 KD 树是否有效
    if (!kd_tree_ || !global_cloud_ || global_cloud_->empty()) {
        return positions;
    }

    pcl::PointXYZI query_pt;
    query_pt.x = center.x();
    query_pt.y = center.y();
    query_pt.z = center.z();
    query_pt.intensity = 0;

    std::vector<int> indices(k);
    std::vector<float> distances(k);

    int found = kd_tree_->nearestKSearch(query_pt, k, indices, distances);
    indices.resize(found);

    // ✅ 修复：在锁保护下访问点云数据
    for (int idx : indices) {
        const auto& pt = global_cloud_->points[idx];
        positions.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
    }

    return positions;
}

// ─────────────────────────────────────────────────────────────────────────────
// 地图统计
// ─────────────────────────────────────────────────────────────────────────────

int GlobalMap::getPointCount() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return static_cast<int>(global_cloud_->size());
}

Eigen::Vector3d GlobalMap::getMapMin() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return min_pt_;
}

Eigen::Vector3d GlobalMap::getMapMax() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return max_pt_;
}

Eigen::Vector3d GlobalMap::getMapCenter() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return (min_pt_ + max_pt_) / 2.0;
}

double GlobalMap::getMapRadius() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return (max_pt_ - min_pt_).norm() / 2.0;
}

// ─────────────────────────────────────────────────────────────────────────────
// 清理
// ─────────────────────────────────────────────────────────────────────────────

void GlobalMap::clear() {
    std::lock_guard<std::mutex> lk(mutex_);
    
    global_cloud_->clear();
    min_pt_ = Eigen::Vector3d::Constant(1e6);
    max_pt_ = Eigen::Vector3d::Constant(-1e6);
    kd_tree_needs_update_ = true;
    
    ALOG_INFO(MOD, "Global map cleared");
}

void GlobalMap::clearROI(const Eigen::Vector3d& center, float radius) {
    std::lock_guard<std::mutex> lk(mutex_);
    
    CloudXYZIPtr filtered(new CloudXYZI);
    filtered->reserve(global_cloud_->size());
    
    // 修复类型转换：center_f 应该是 Vector3f 而不是 Vector3d
    Eigen::Vector3f center_f = center.cast<float>();
    
    for (const auto& pt : global_cloud_->points) {
        Eigen::Vector3f pt_vec(pt.x, pt.y, pt.z);
        if ((pt_vec - center_f).norm() > radius) {
            filtered->push_back(pt);
        }
    }
    
    global_cloud_ = filtered;
    kd_tree_needs_update_ = true;
    
    ALOG_INFO(MOD, "Cleared ROI: center=[{:.2f},{:.2f},{:.2f}] radius={:.1f}m, remaining: {} pts",
              center.x(), center.y(), center.z(), radius, global_cloud_->size());
}

void GlobalMap::keepROI(const Eigen::Vector3d& center, float radius) {
    std::lock_guard<std::mutex> lk(mutex_);
    
    CloudXYZIPtr filtered(new CloudXYZI);
    filtered->reserve(global_cloud_->size());
    
    // 修复类型转换
    Eigen::Vector3f center_f = center.cast<float>();
    
    for (const auto& pt : global_cloud_->points) {
        Eigen::Vector3f pt_vec(pt.x, pt.y, pt.z);
        if ((pt_vec - center_f).norm() <= radius) {
            filtered->push_back(pt);
        }
    }
    
    global_cloud_ = filtered;
    kd_tree_needs_update_ = true;
    
    ALOG_INFO(MOD, "Kept ROI: center=[{:.2f},{:.2f},{:.2f}] radius={:.1f}m, remaining: {} pts",
              center.x(), center.y(), center.z(), radius, global_cloud_->size());
}

// ─────────────────────────────────────────────────────────────────────────────
// 持久化
// ─────────────────────────────────────────────────────────────────────────────

bool GlobalMap::savePCD(const std::string& path, bool binary) const {
    std::lock_guard<std::mutex> lk(mutex_);
    
    try {
        if (binary) {
            return pcl::io::savePCDFileBinary(path, *global_cloud_) == 0;
        } else {
            return pcl::io::savePCDFileASCII(path, *global_cloud_) == 0;
        }
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "Failed to save PCD: {}", e.what());
        return false;
    }
}

bool GlobalMap::loadPCD(const std::string& path) {
    std::lock_guard<std::mutex> lk(mutex_);
    
    try {
        if (pcl::io::loadPCDFile(path, *global_cloud_) == -1) {
            ALOG_ERROR(MOD, "Failed to load PCD: {}", path);
            return false;
        }
        
        // 更新地图边界
        updateMapBounds(global_cloud_);
        kd_tree_needs_update_ = true;
        
        ALOG_INFO(MOD, "Loaded PCD: {} points", global_cloud_->size());
        return true;
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "Failed to load PCD: {}", e.what());
        return false;
    }
}

void GlobalMap::registerUpdateCallback(UpdateCallback cb) {
    std::lock_guard<std::mutex> lk(mutex_);
    update_cbs_.push_back(std::move(cb));
}

// ─────────────────────────────────────────────────────────────────────────────
// 内存管理
// ─────────────────────────────────────────────────────────────────────────────

void GlobalMap::optimizeMemory() {
    std::lock_guard<std::mutex> lk(mutex_);
    
    if (global_cloud_->size() > static_cast<size_t>(config_.max_points / 2)) {
        size_t old_size = global_cloud_->size();
        downsampleCloud(global_cloud_, config_.voxel_size * 0.8f);  // 更激进降采样
        ALOG_INFO(MOD, "Memory optimized: {} -> {} points", old_size, global_cloud_->size());
    }
}

size_t GlobalMap::getMemorySizeMB() const {
    std::lock_guard<std::mutex> lk(mutex_);
    size_t bytes = global_cloud_->points.size() * sizeof(pcl::PointXYZI);
    return bytes / (1024 * 1024);
}

// ─────────────────────────────────────────────────────────────────────────────
// KD树管理
// ─────────────────────────────────────────────────────────────────────────────

void GlobalMap::ensureKDTree() const {
    if (kd_tree_needs_update_) {
        const_cast<GlobalMap*>(this)->buildKDTree();
    }
}

void GlobalMap::buildKDTree() {
    if (global_cloud_->empty()) return;
    
    kd_tree_->setInputCloud(global_cloud_);
    kd_tree_needs_update_ = false;
    
    ALOG_DEBUG(MOD, "KD-tree built: {} points", global_cloud_->size());
}

// ─────────────────────────────────────────────────────────────────────────────
// 辅助函数
// ─────────────────────────────────────────────────────────────────────────────

void GlobalMap::transformCloud(const CloudXYZIPtr& input, const Pose3d& T, 
                              CloudXYZI& output) const {
    Eigen::Affine3f T_f = T.cast<float>();
    pcl::transformPointCloud(*input, output, T_f);
}

void GlobalMap::downsampleCloud(CloudXYZIPtr& cloud, float voxel_size) const {
    if (cloud->size() < 1000) return;
    
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(voxel_size, voxel_size, voxel_size);
    vg.filter(*cloud);
}

void GlobalMap::updateMapBounds(const CloudXYZIPtr& cloud) {
    for (const auto& pt : cloud->points) {
        min_pt_.x() = std::min(min_pt_.x(), static_cast<double>(pt.x));
        min_pt_.y() = std::min(min_pt_.y(), static_cast<double>(pt.y));
        min_pt_.z() = std::min(min_pt_.z(), static_cast<double>(pt.z));
        
        max_pt_.x() = std::max(max_pt_.x(), static_cast<double>(pt.x));
        max_pt_.y() = std::max(max_pt_.y(), static_cast<double>(pt.y));
        max_pt_.z() = std::max(max_pt_.z(), static_cast<double>(pt.z));
    }
}

void GlobalMap::notifyUpdateCallbacks() {
    for (auto& cb : update_cbs_) {
        cb(static_cast<int>(global_cloud_->size()));
    }
}

} // namespace automap_pro
