#include "automap_pro/map/map_exporter.h"
#include "automap_pro/core/logger.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/gp3.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <filesystem>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
using json = nlohmann::json;
#define MOD "MapExporter"

namespace automap_pro {

MapExporter::MapExporter()
    : MapExporter(Config()) {
}

MapExporter::MapExporter(const Config& config)
    : config_(config)
    , last_export_size_(0)
    , running_(true)
    , export_pending_(false)
{
    ensureOutputDir(config_.output_dir);
    
    ALOG_INFO(MOD, "MapExporter initialized: output_dir={} format={}",
              config_.output_dir, config_.trajectory_format);
    
    export_thread_ = std::thread(&MapExporter::exportThreadLoop, this);
}

MapExporter::~MapExporter() {
    running_ = false;
    
    if (export_thread_.joinable()) {
        // 发送空任务唤醒线程
        {
            std::lock_guard<std::mutex> lk(mutex_);
            ExportTask empty_task;
            export_queue_.push(empty_task);
        }
        export_thread_.join();
    }
    
    ALOG_INFO(MOD, "MapExporter destroyed");
}

void MapExporter::setConfig(const Config& config) {
    std::lock_guard<std::mutex> lk(mutex_);
    config_ = config;
    ensureOutputDir(config_.output_dir);
}

MapExporter::Config MapExporter::getConfig() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return config_;
}

// ─────────────────────────────────────────────────────────────────────────────
// 导出地图
// ─────────────────────────────────────────────────────────────────────────────

bool MapExporter::exportMap(const CloudXYZIPtr& cloud, const std::string& filename) {
    if (!cloud || cloud->empty()) {
        ALOG_ERROR(MOD, "Cannot export empty cloud");
        return false;
    }
    
    std::lock_guard<std::mutex> lk(mutex_);
    
    std::string extension = fs::path(filename).extension().string();
    std::string full_path = config_.output_dir + "/" + filename;
    
    ALOG_INFO(MOD, "Exporting map: {} points -> {}", cloud->size(), full_path);
    
    notifyProgressCallbacks(0, 100, "Preparing map");
    
    // 降采样（如果指定）
    CloudXYZIPtr export_cloud = cloud;
    if (config_.base_resolution > 0) {
        export_cloud = downsampleCloud(cloud, config_.base_resolution);
        notifyProgressCallbacks(20, 100, "Downsampling");
    }
    
    bool success = false;
    if (extension == ".pcd") {
        success = exportPCD(export_cloud, full_path);
    } else if (extension == ".ply") {
        success = exportPLY(export_cloud, full_path);
    } else if (extension == ".obj") {
        success = exportOBJ(export_cloud, full_path);
    } else {
        // 默认使用PCD
        success = exportPCD(export_cloud, full_path);
    }
    
    if (success) {
        last_export_size_ = export_cloud->size();
        last_export_path_ = full_path;
        notifyProgressCallbacks(100, 100, "Done");
    }
    
    return success;
}

bool MapExporter::exportMap(const std::shared_ptr<GlobalMap>& global_map,
                           const std::string& filename) {
    if (!global_map) {
        ALOG_ERROR(MOD, "GlobalMap is null");
        return false;
    }
    
    CloudXYZIPtr cloud = global_map->getMap(config_.base_resolution);
    if (!cloud || cloud->empty()) {
        ALOG_ERROR(MOD, "Global map is empty");
        return false;
    }
    
    return exportMap(cloud, filename);
}

bool MapExporter::exportSubmaps(const std::vector<SubMap::Ptr>& submaps,
                               const std::string& filename_prefix) {
    if (submaps.empty()) {
        ALOG_WARN(MOD, "No submaps to export");
        return false;
    }
    
    bool all_success = true;
    int success_count = 0;
    
    notifyProgressCallbacks(0, static_cast<int>(submaps.size()), "Exporting submaps");
    
    for (size_t i = 0; i < submaps.size(); ++i) {
        const auto& sm = submaps[i];
        if (!sm || !sm->merged_cloud || sm->merged_cloud->empty()) continue;
        
        std::string filename = filename_prefix + "_" + std::to_string(sm->id) + ".pcd";
        std::string full_path = config_.output_dir + "/" + filename;
        
        // 降采样
        CloudXYZIPtr export_cloud = downsampleCloud(sm->merged_cloud, config_.base_resolution);
        
        bool success = exportPCD(export_cloud, full_path);
        if (success) success_count++;
        else all_success = false;
        
        notifyProgressCallbacks(static_cast<int>(i + 1), 
                             static_cast<int>(submaps.size()), 
                             "Exporting submaps");
    }
    
    ALOG_INFO(MOD, "Exported {}/{} submaps", success_count, submaps.size());
    return all_success;
}

// ─────────────────────────────────────────────────────────────────────────────
// 导出轨迹
// ─────────────────────────────────────────────────────────────────────────────

bool MapExporter::exportTrajectory(const std::vector<KeyFrame::Ptr>& keyframes,
                                  const std::string& filename) {
    if (keyframes.empty()) {
        ALOG_WARN(MOD, "No keyframes to export");
        return false;
    }
    
    std::string full_path = config_.output_dir + "/" + filename;
    
    ALOG_INFO(MOD, "Exporting trajectory: {} keyframes -> {}", keyframes.size(), full_path);
    
    std::ofstream out(full_path);
    if (!out.is_open()) {
        ALOG_ERROR(MOD, "Failed to open file: {}", full_path);
        return false;
    }
    
    // 写入头部
    out << "# AutoMap-Pro Trajectory\n";
    out << "# timestamp x y z qx qy qz qw\n";
    
    // 写入数据
    for (const auto& kf : keyframes) {
        if (!kf) continue;
        
        const auto& T = kf->T_w_b;
        Eigen::Vector3d t = T.translation();
        Eigen::Quaterniond q(T.rotation());
        
        out << std::fixed << std::setprecision(9)
            << kf->timestamp << " "
            << t.x() << " " << t.y() << " " << t.z() << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }
    
    out.close();
    ALOG_INFO(MOD, "Trajectory exported: {} keyframes", keyframes.size());
    return true;
}

bool MapExporter::exportTrajectoryKML(const std::vector<KeyFrame::Ptr>& keyframes,
                                     const std::string& filename) {
    if (keyframes.empty()) return false;

    std::string full_path = config_.output_dir + "/" + filename;

    // 检查原点是否有效
    double origin_lat = config_.origin_latitude;
    double origin_lon = config_.origin_longitude;
    double origin_alt = config_.origin_altitude;

    if (!config_.origin_valid ||
        (std::abs(origin_lat) < 0.001 && std::abs(origin_lon) < 0.001)) {
        ALOG_WARN(MOD, "KML export: origin (lat/lon/alt) not properly configured. "
                  "Set origin_latitude, origin_longitude, origin_altitude, and origin_valid=true "
                  "in config for accurate WGS84 conversion. Using approximate fallback.");
    }

    std::ofstream out(full_path);
    if (!out.is_open()) return false;

    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
    out << "  <Document>\n";
    out << "    <name>AutoMap-Pro Trajectory</name>\n";
    out << "    <Placemark>\n";
    out << "      <LineString>\n";
    out << "        <coordinates>\n";

    for (const auto& kf : keyframes) {
        if (!kf) continue;

        Eigen::Vector3d t_enu = kf->T_w_b.translation();
        Eigen::Vector3d wgs84;

        if (config_.origin_valid) {
            // 使用精确 ENU→WGS84 转换
            wgs84 = enuToWGS84(t_enu, origin_lat, origin_lon, origin_alt);
        } else {
            // 回退到简化近似（仅适用于小范围）
            constexpr double R_EARTH = 6378137.0;
            double lat_rad = origin_lat * M_PI / 180.0;
            double lat = origin_lat + t_enu.y() / R_EARTH * (180.0 / M_PI);
            double lon = origin_lon + t_enu.x() / (R_EARTH * std::cos(lat_rad)) * (180.0 / M_PI);
            double alt = origin_alt + t_enu.z();
            wgs84 = Eigen::Vector3d(lat, lon, alt);
        }

        // KML 格式：lon,lat,alt（注意顺序！）
        out << std::fixed << std::setprecision(9)
            << wgs84.y() << "," << wgs84.x() << "," << wgs84.z() << " ";
    }

    out << "\n";
    out << "        </coordinates>\n";
    out << "      </LineString>\n";
    out << "    </Placemark>\n";
    out << "  </Document>\n";
    out << "</kml>\n";

    out.close();
    ALOG_INFO(MOD, "KML trajectory exported: {} points -> {}", keyframes.size(), full_path);
    return true;
}

bool MapExporter::exportTrajectoryCSV(const std::vector<KeyFrame::Ptr>& keyframes,
                                    const std::string& filename) {
    if (keyframes.empty()) return false;
    
    std::string full_path = config_.output_dir + "/" + filename;
    
    std::ofstream out(full_path);
    if (!out.is_open()) return false;
    
    // 写入CSV头部
    out << "timestamp,x,y,z,qx,qy,qz,qw\n";
    
    // 写入数据
    for (const auto& kf : keyframes) {
        if (!kf) continue;
        
        const auto& T = kf->T_w_b;
        Eigen::Vector3d t = T.translation();
        Eigen::Quaterniond q(T.rotation());
        
        out << std::fixed << std::setprecision(9)
            << kf->timestamp << ","
            << t.x() << "," << t.y() << "," << t.z() << ","
            << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "\n";
    }
    
    out.close();
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// 导出元数据
// ─────────────────────────────────────────────────────────────────────────────

bool MapExporter::exportMetadata(const ExportMetadata& metadata,
                                const std::string& filename) {
    std::string full_path = config_.output_dir + "/" + filename;
    
    std::ofstream out(full_path);
    if (!out.is_open()) {
        ALOG_ERROR(MOD, "Failed to open metadata file: {}", full_path);
        return false;
    }
    
    json meta;
    meta["version"] = metadata.version;
    meta["timestamp"] = metadata.timestamp;
    meta["total_keyframes"] = metadata.total_keyframes;
    meta["total_submaps"] = metadata.total_submaps;
    meta["total_points"] = metadata.total_points;
    meta["coordinate_system"] = metadata.coordinate_system;
    
    meta["map_center"] = {
        metadata.map_center[0],
        metadata.map_center[1],
        metadata.map_center[2]
    };
    meta["map_radius"] = metadata.map_radius;
    
    meta["sensor"] = {
        {"lidar_type", metadata.lidar_type},
        {"lidar_config", metadata.lidar_config},
        {"imu_type", metadata.imu_type}
    };
    
    meta["optimization"] = {
        {"has_loop_closure", metadata.has_loop_closure},
        {"loop_closure_count", metadata.loop_closure_count},
        {"has_gps", metadata.has_gps},
        {"gps_constraints", metadata.gps_constraints}
    };
    
    out << meta.dump(2);
    out.close();
    
    ALOG_INFO(MOD, "Metadata exported: {}", full_path);
    return true;
}

bool MapExporter::exportMapInfo(const std::vector<SubMap::Ptr>& submaps,
                                const std::vector<KeyFrame::Ptr>& keyframes,
                                const std::string& filename) {
    std::string full_path = config_.output_dir + "/" + filename;
    
    std::ofstream out(full_path);
    if (!out.is_open()) return false;
    
    out << "# AutoMap-Pro Map Info\n";
    out << "version: 2.0\n";
    out << "timestamp: " << getCurrentTimestamp() << "\n\n";
    
    out << "keyframes: " << keyframes.size() << "\n";
    out << "submaps: " << submaps.size() << "\n\n";
    
    // 导出子图信息
    out << "submaps:\n";
    for (const auto& sm : submaps) {
        if (!sm) continue;
        
        out << "  - id: " << sm->id << "\n";
        out << "    session_id: " << sm->session_id << "\n";
        out << "    keyframes: " << sm->keyframes.size() << "\n";
        out << "    time_range: [" << sm->t_start << ", " << sm->t_end << "]\n";
        out << "    spatial_extent: " << sm->spatial_extent_m << "m\n";
        out << "    has_gps: " << (sm->has_valid_gps ? "true" : "false") << "\n";
    }
    
    out.close();
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// 批量导出
// ─────────────────────────────────────────────────────────────────────────────

bool MapExporter::exportAll(const std::vector<SubMap::Ptr>& submaps,
                             const std::vector<KeyFrame::Ptr>& keyframes,
                             const std::string& output_name) {
    std::string export_dir = config_.output_dir + "/" + output_name;
    ensureOutputDir(export_dir);
    
    ALOG_INFO(MOD, "Starting full export to: {}", export_dir);
    
    bool all_success = true;
    
    // 1. 导出全局地图
    if (!submaps.empty()) {
        // 合并所有子图点云
        CloudXYZIPtr global_cloud(new CloudXYZI);
        for (const auto& sm : submaps) {
            if (sm && sm->merged_cloud && !sm->merged_cloud->empty()) {
                *global_cloud += *sm->merged_cloud;
            }
        }
        
        if (!global_cloud->empty()) {
            std::string map_path = export_dir + "/global_map.pcd";
            CloudXYZIPtr ds_cloud = downsampleCloud(global_cloud, config_.base_resolution);
            all_success &= exportPCD(ds_cloud, map_path);
        }
    }
    
    // 2. 导出轨迹
    if (config_.save_trajectory && !keyframes.empty()) {
        if (config_.trajectory_format == "txt") {
            all_success &= exportTrajectory(keyframes, export_dir + "/trajectory.txt");
        } else if (config_.trajectory_format == "kml") {
            all_success &= exportTrajectoryKML(keyframes, export_dir + "/trajectory.kml");
        } else if (config_.trajectory_format == "csv") {
            all_success &= exportTrajectoryCSV(keyframes, export_dir + "/trajectory.csv");
        }
    }
    
    // 3. 导出子图
    if (config_.save_submaps) {
        all_success &= exportSubmaps(submaps, export_dir + "/submap");
    }
    
    // 4. 导出元数据
    if (config_.save_metadata) {
        ExportMetadata meta = buildMetadata(submaps, keyframes);
        all_success &= exportMetadata(meta, export_dir + "/metadata.json");
        all_success &= exportMapInfo(submaps, keyframes, export_dir + "/map_info.yaml");
    }
    
    ALOG_INFO(MOD, "Full export {}: {}", output_name, all_success ? "SUCCESS" : "PARTIAL");
    return all_success;
}

void MapExporter::exportAllAsync(const std::vector<SubMap::Ptr>& submaps,
                                 const std::vector<KeyFrame::Ptr>& keyframes,
                                 const std::string& output_name) {
    ExportTask task;
    task.submaps = submaps;
    task.keyframes = keyframes;
    task.output_name = output_name;
    
    {
        std::lock_guard<std::mutex> lk(mutex_);
        export_queue_.push(task);
        export_pending_ = true;
    }
    
    ALOG_DEBUG(MOD, "Queued async export: {}", output_name);
}

void MapExporter::waitForExport() {
    while (export_pending_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 坐标转换
// ─────────────────────────────────────────────────────────────────────────────

Eigen::Vector3d MapExporter::enuToWGS84(const Eigen::Vector3d& enu_pt,
                                         double origin_lat, double origin_lon,
                                         double origin_alt) const {
    // WGS84椭球参数
    constexpr double a = 6378137.0;           // 长半轴（米）
    constexpr double f = 1.0 / 298.257223563; // 扁率
    constexpr double e2 = 2 * f - f * f;      // 第一偏心率的平方

    // 将原点经纬度转换为弧度
    double lat0 = origin_lat * M_PI / 180.0;
    double lon0 = origin_lon * M_PI / 180.0;

    // 计算原点处的椭球曲率半径
    double sin_lat0 = std::sin(lat0);
    double N0 = a / std::sqrt(1.0 - e2 * sin_lat0 * sin_lat0);

    // 计算原点处的子午圈曲率半径
    double M0 = a * (1.0 - e2) / std::pow(1.0 - e2 * sin_lat0 * sin_lat0, 1.5);

    // ENU → 大地坐标变化量（弧度）
    double dlat = enu_pt.y() / M0;  // north → latitude change
    double dlon = enu_pt.x() / (N0 * std::cos(lat0));  // east → longitude change
    double dalt = enu_pt.z();  // up → altitude change

    // 新的经纬度（弧度转度）
    double lat = (lat0 + dlat) * 180.0 / M_PI;
    double lon = (lon0 + dlon) * 180.0 / M_PI;
    double alt = origin_alt + dalt;

    return Eigen::Vector3d(lat, lon, alt);
}

bool MapExporter::convertToUTM(const CloudXYZIPtr& cloud_enu,
                                 double origin_lat, double origin_lon,
                                 CloudXYZIPtr& cloud_utm) const {
    // TODO: 实现ENU到UTM的精确转换
    // 需要使用GeographicLib或Proj4库
    if (!cloud_enu || cloud_enu->empty()) return false;
    
    cloud_utm = CloudXYZIPtr(new CloudXYZI);
    cloud_utm->resize(cloud_enu->size());
    
    // 简化实现（近似）
    double lat_per_m = 1.0 / 111319.0;
    double lon_per_m = 1.0 / (111319.0 * std::cos(origin_lat * M_PI / 180.0));
    
    for (size_t i = 0; i < cloud_enu->size(); ++i) {
        const auto& pt_src = cloud_enu->points[i];
        auto& pt_dst = cloud_utm->points[i];
        
        pt_dst.x = origin_lon + pt_src.x * lon_per_m;
        pt_dst.y = origin_lat + pt_src.y * lat_per_m;
        pt_dst.z = pt_src.z;
        pt_dst.intensity = pt_src.intensity;
    }
    
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// 回调
// ─────────────────────────────────────────────────────────────────────────────

void MapExporter::registerProgressCallback(ProgressCallback cb) {
    std::lock_guard<std::mutex> lk(mutex_);
    progress_cbs_.push_back(std::move(cb));
}

void MapExporter::registerDoneCallback(DoneCallback cb) {
    std::lock_guard<std::mutex> lk(mutex_);
    done_cbs_.push_back(std::move(cb));
}

// ─────────────────────────────────────────────────────────────────────────────
// 统计信息
// ─────────────────────────────────────────────────────────────────────────────

size_t MapExporter::getLastExportSize() const {
    return last_export_size_.load();
}

std::string MapExporter::getLastExportPath() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return last_export_path_;
}

// ─────────────────────────────────────────────────────────────────────────────
// 后台导出循环
// ─────────────────────────────────────────────────────────────────────────────

void MapExporter::exportThreadLoop() {
    while (running_.load()) {
        ExportTask task;
        {
            std::unique_lock<std::mutex> lk(mutex_);
            export_queue_.wait(lk, [this] {
                return !export_queue_.empty() || !running_.load();
            });
            
            if (!running_.load() && export_queue_.empty()) break;
            
            if (!export_queue_.empty()) {
                task = export_queue_.front();
                export_queue_.pop();
            }
        }
        
        processExportTask(task);
        
        export_pending_ = false;
    }
}

void MapExporter::processExportTask(const ExportTask& task) {
    ALOG_DEBUG(MOD, "Processing async export: {}", task.output_name);
    
    bool success = exportAll(task.submaps, task.keyframes, task.output_name);
    
    std::string export_path = config_.output_dir + "/" + task.output_name;
    notifyDoneCallbacks(success, export_path);
}

// ─────────────────────────────────────────────────────────────────────────────
// 导出实现
// ─────────────────────────────────────────────────────────────────────────────

bool MapExporter::exportPCD(const CloudXYZIPtr& cloud, const std::string& path) {
    if (config_.compress) {
        return pcl::io::savePCDFileBinaryCompressed(path, *cloud) == 0;
    } else {
        return pcl::io::savePCDFileBinary(path, *cloud) == 0;
    }
}

bool MapExporter::exportPLY(const CloudXYZIPtr& cloud, const std::string& path) {
    // PLY支持RGB（可选）
    if (config_.save_rgb) {
        // 需要转换到PointXYZRGB
        return pcl::io::savePLYFileBinary(path, *cloud) == 0;
    } else {
        return pcl::io::savePLYFileBinary(path, *cloud) == 0;
    }
}

bool MapExporter::exportOBJ(const CloudXYZIPtr& cloud, const std::string& path) {
    std::ofstream out(path);
    if (!out.is_open()) return false;
    
    // OBJ格式：顶点列表
    for (const auto& pt : cloud->points) {
        out << "v " << pt.x << " " << pt.y << " " << pt.z << "\n";
    }
    
    out.close();
    return true;
}

bool MapExporter::exportLAS(const CloudXYZIPtr& cloud, const std::string& path) {
    // TODO: 实现LAS格式导出
    // 需要使用liblas或pdal库
    ALOG_WARN(MOD, "LAS export not yet implemented");
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// 降采样
// ─────────────────────────────────────────────────────────────────────────────

CloudXYZIPtr MapExporter::downsampleCloud(const CloudXYZIPtr& cloud, float resolution) const {
    if (resolution <= 0) return cloud;
    
    CloudXYZIPtr downsampled(new CloudXYZI);
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(resolution, resolution, resolution);
    vg.filter(*downsampled);
    
    return downsampled;
}

// ─────────────────────────────────────────────────────────────────────────────
// 辅助函数
// ─────────────────────────────────────────────────────────────────────────────

std::string MapExporter::getCurrentTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

void MapExporter::ensureOutputDir(const std::string& dir) {
    try {
        fs::create_directories(dir);
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "Failed to create output directory {}: {}", dir, e.what());
    }
}

MapExporter::ExportMetadata MapExporter::buildMetadata(const std::vector<SubMap::Ptr>& submaps,
                                                    const std::vector<KeyFrame::Ptr>& keyframes) const {
    ExportMetadata meta;
    
    meta.version = "2.0";
    meta.timestamp = getCurrentTimestamp();
    meta.total_keyframes = static_cast<int>(keyframes.size());
    meta.total_submaps = static_cast<int>(submaps.size());
    meta.coordinate_system = config_.coordinate_system;
    
    // 计算总点数
    size_t total_points = 0;
    for (const auto& sm : submaps) {
        if (sm && sm->merged_cloud) {
            total_points += sm->merged_cloud->size();
        }
    }
    meta.total_points = total_points;
    
    // 计算地图边界
    Eigen::Vector3d min_pt(1e6, 1e6, 1e6);
    Eigen::Vector3d max_pt(-1e6, -1e6, -1e6);
    
    for (const auto& kf : keyframes) {
        if (!kf) continue;
        
        Eigen::Vector3d t = kf->T_w_b.translation();
        min_pt = min_pt.cwiseMin(t);
        max_pt = max_pt.cwiseMax(t);
    }
    
    Eigen::Vector3d center = (min_pt + max_pt) / 2.0;
    double radius = (max_pt - min_pt).norm() / 2.0;
    
    meta.map_center[0] = center.x();
    meta.map_center[1] = center.y();
    meta.map_center[2] = center.z();
    meta.map_radius = radius;
    
    // TODO: 填充传感器信息和优化信息
    meta.lidar_type = "Unknown";
    meta.imu_type = "Unknown";
    meta.has_loop_closure = false;
    meta.has_gps = false;
    
    return meta;
}

void MapExporter::notifyProgressCallbacks(int current, int total, const std::string& stage) {
    for (auto& cb : progress_cbs_) {
        cb(current, total, stage);
    }
}

void MapExporter::notifyDoneCallbacks(bool success, const std::string& path) {
    for (auto& cb : done_cbs_) {
        cb(success, path);
    }
}

} // namespace automap_pro
