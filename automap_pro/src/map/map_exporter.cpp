#include "automap_pro/map/map_exporter.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/config_manager.h"
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
#include <cstdint>
#include <cstring>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
using json = nlohmann::json;
#define MOD "MapExporter"

namespace automap_pro {

namespace {

/**
 * 与 exportSubmaps、SubMapManager::buildGlobalMap 主路径一致：用各关键帧 body 点云经 T_map_b_optimized 变换后拼接全局图。
 * 禁止直接拼接 merged_cloud（冻结/里程计系），否则后端回环/GPS/HBA 优化后点云与轨迹不一致，表现为重影与视觉“发糊”。
 */
CloudXYZIPtr buildGlobalCloudFromSubmapsOptimized(const std::vector<SubMap::Ptr>& submaps) {
    CloudXYZIPtr global(new CloudXYZI);
    for (const auto& sm : submaps) {
        if (!sm) continue;
        if (!sm->keyframes.empty()) {
            for (const auto& kf : sm->keyframes) {
                if (!kf || !kf->cloud_body || kf->cloud_body->empty()) continue;
                CloudXYZI tmp;
                Eigen::Affine3f T_f = kf->T_map_b_optimized.cast<float>();
                pcl::transformPointCloud(*kf->cloud_body, tmp, T_f);
                *global += tmp;
            }
        } else if (sm->merged_cloud && !sm->merged_cloud->empty()) {
            *global += *sm->merged_cloud;
        }
    }
    return global;
}

}  // namespace

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
            export_cv_.notify_one();
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
    } else if (extension == ".las") {
        success = exportLAS(export_cloud, full_path);
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
        if (!sm) continue;
        
        std::string filename = filename_prefix + "_" + std::to_string(sm->id) + ".pcd";
        std::string full_path = config_.output_dir + "/" + filename;
        
        // 注意：不要直接导出 sm->merged_cloud。
        // merged_cloud 是冻结时按当时的 T_odom_b 变换得到的“局部系点云”，后端优化（GPS/回环/HBA）后会与轨迹不一致。
        // 这里按 KeyFrame 的 body 点云 + T_map_b_optimized 重投影后再导出，保证导出的子图与最终位姿一致。
        CloudXYZIPtr world(new CloudXYZI);
        if (!sm->keyframes.empty()) {
            for (const auto& kf : sm->keyframes) {
                if (!kf || !kf->cloud_body || kf->cloud_body->empty()) continue;
                CloudXYZI tmp;
                Eigen::Affine3f T_f = kf->T_map_b_optimized.cast<float>();
                pcl::transformPointCloud(*kf->cloud_body, tmp, T_f);
                *world += tmp;
            }
        } else if (sm->merged_cloud && !sm->merged_cloud->empty()) {
            // 兜底：若无 keyframes 列表，则退化为 merged_cloud（可能与优化后不一致）
            *world = *sm->merged_cloud;
        }
        if (!world || world->empty()) continue;

        // 降采样
        CloudXYZIPtr export_cloud = downsampleCloud(world, config_.base_resolution);
        
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
    
    // 写入数据（与 buildGlobalMap/saveMapToFiles 一致：优先优化位姿，未优化则用原始位姿）
    for (const auto& kf : keyframes) {
        if (!kf) continue;
        Pose3d T = kf->T_map_b_optimized;
        if (T.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-6) && !kf->T_odom_b.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-6))
            T = kf->T_odom_b;
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
        Pose3d T = kf->T_map_b_optimized;
        if (T.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-6) && !kf->T_odom_b.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-6))
            T = kf->T_odom_b;
        Eigen::Vector3d t_enu = T.translation();
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
    
    // 写入数据（与轨迹 txt 一致：优先优化位姿）
    for (const auto& kf : keyframes) {
        if (!kf) continue;
        Pose3d T = kf->T_map_b_optimized;
        if (T.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-6) && !kf->T_odom_b.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-6))
            T = kf->T_odom_b;
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
    
    // 1. 导出全局地图（与 exportSubmaps 一致：kf->cloud_body + T_map_b_optimized，勿直接拼 merged_cloud）
    if (!submaps.empty()) {
        CloudXYZIPtr global_cloud = buildGlobalCloudFromSubmapsOptimized(submaps);

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
        export_cv_.notify_one();
    }
    
    ALOG_DEBUG(MOD, "Queued async export: {}", output_name);
}

void MapExporter::waitForExport() {
    while (export_pending_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 坐标转换（地图坐标系统一为 ENU：x=East, y=North, z=Up，单位米）
// ─────────────────────────────────────────────────────────────────────────────

Eigen::Vector3d MapExporter::enuToWGS84(const Eigen::Vector3d& enu_pt,
                                         double origin_lat, double origin_lon,
                                         double origin_alt) const {
    // ENU 约定：enu_pt.x()=East, enu_pt.y()=North, enu_pt.z()=Up（米）
    // WGS84 椭球参数
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
            export_cv_.wait(lk, [this] {
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
    // LAS 1.2 二进制格式，Point Data Record Format 0（X,Y,Z,Intensity 等），无外部依赖
    if (!cloud || cloud->empty()) {
        ALOG_WARN(MOD, "exportLAS: empty cloud");
        return false;
    }

    const double scale = 0.0001;  // 0.1mm 精度
    double min_x = 1e38, min_y = 1e38, min_z = 1e38;
    double max_x = -1e38, max_y = -1e38, max_z = -1e38;
    for (const auto& pt : cloud->points) {
        min_x = std::min(min_x, static_cast<double>(pt.x));
        min_y = std::min(min_y, static_cast<double>(pt.y));
        min_z = std::min(min_z, static_cast<double>(pt.z));
        max_x = std::max(max_x, static_cast<double>(pt.x));
        max_y = std::max(max_y, static_cast<double>(pt.y));
        max_z = std::max(max_z, static_cast<double>(pt.z));
    }
    const double off_x = min_x, off_y = min_y, off_z = min_z;

    std::ofstream out(path, std::ios::binary);
    if (!out.is_open()) {
        ALOG_ERROR(MOD, "exportLAS: cannot open {}", path);
        return false;
    }

    // Public Header Block (LAS 1.2, 227 bytes)
    char header[227];
    std::memset(header, 0, sizeof(header));
    std::memcpy(header + 0, "LASF", 4);
    header[24] = 1;
    header[25] = 2;
    std::memcpy(header + 26, "AutoMap-Pro ENU", 15);   // System Identifier (32 bytes)
    std::memcpy(header + 58, "AutoMap-Pro LAS 1.2", 19); // Generating Software (32 bytes)
    // creation day/year: 0 = unknown
    const uint16_t header_size = 227;
    std::memcpy(header + 94, &header_size, 2);
    const uint32_t point_data_offset = 227;
    std::memcpy(header + 96, &point_data_offset, 4);
    const uint32_t num_vlrs = 0;
    std::memcpy(header + 100, &num_vlrs, 4);
    const uint8_t point_format = 0;
    header[104] = point_format;
    const uint16_t record_len = 20;
    std::memcpy(header + 105, &record_len, 2);
    const uint32_t npts = static_cast<uint32_t>(std::min(cloud->size(), static_cast<size_t>(UINT32_MAX)));
    std::memcpy(header + 107, &npts, 4);
    std::memcpy(header + 131, &scale, 8);
    std::memcpy(header + 139, &scale, 8);
    std::memcpy(header + 147, &scale, 8);
    std::memcpy(header + 155, &off_x, 8);
    std::memcpy(header + 163, &off_y, 8);
    std::memcpy(header + 171, &off_z, 8);
    out.write(header, sizeof(header));

    // Point Data Records (Format 0: 20 bytes each)
    for (uint32_t i = 0; i < npts; ++i) {
        const auto& pt = cloud->points[i];
        int32_t ix = static_cast<int32_t>(std::round((pt.x - off_x) / scale));
        int32_t iy = static_cast<int32_t>(std::round((pt.y - off_y) / scale));
        int32_t iz = static_cast<int32_t>(std::round((pt.z - off_z) / scale));
        uint16_t intensity = static_cast<uint16_t>(std::min(65535.0f, std::max(0.0f, pt.intensity)));
        uint8_t return_byte = 1;
        uint8_t classification = 0;
        int8_t scan_angle = 0;
        uint8_t user_data = 0;
        uint16_t point_source_id = 0;

        out.write(reinterpret_cast<const char*>(&ix), 4);
        out.write(reinterpret_cast<const char*>(&iy), 4);
        out.write(reinterpret_cast<const char*>(&iz), 4);
        out.write(reinterpret_cast<const char*>(&intensity), 2);
        out.write(reinterpret_cast<const char*>(&return_byte), 1);
        out.write(reinterpret_cast<const char*>(&classification), 1);
        out.write(reinterpret_cast<const char*>(&scan_angle), 1);
        out.write(reinterpret_cast<const char*>(&user_data), 1);
        out.write(reinterpret_cast<const char*>(&point_source_id), 2);
    }

    out.close();
    if (!out.good()) {
        ALOG_ERROR(MOD, "exportLAS: write failed");
        return false;
    }
    ALOG_INFO(MOD, "exportLAS: wrote {} points to {}", npts, path);
    return true;
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
        Pose3d T = kf->T_map_b_optimized;
        if (T.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-6) && !kf->T_odom_b.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-6))
            T = kf->T_odom_b;
        Eigen::Vector3d t = T.translation();
        min_pt = min_pt.cwiseMin(t);
        max_pt = max_pt.cwiseMax(t);
    }
    
    Eigen::Vector3d center = (min_pt + max_pt) / 2.0;
    double radius = (max_pt - min_pt).norm() / 2.0;
    
    meta.map_center[0] = center.x();
    meta.map_center[1] = center.y();
    meta.map_center[2] = center.z();
    meta.map_radius = radius;

    // 传感器与优化信息：从 ConfigManager 读取，保持 ENU 坐标系语义
    const auto& cfg = ConfigManager::instance();
    int lidar_type_id = cfg.keyframePreprocessLidarType();
    switch (lidar_type_id) {
        case 1: meta.lidar_type = "Livox Avia";   break;
        case 2: meta.lidar_type = "Velodyne";      break;
        case 3: meta.lidar_type = "Ouster";        break;
        case 4: meta.lidar_type = "RealSense L515"; break;
        default: meta.lidar_type = "Unknown";      break;
    }
    meta.lidar_config = cfg.lidarTopic();
    meta.imu_type = "IMU";
    meta.has_gps = cfg.gpsEnabled();
    meta.gps_constraints = 0;  // 需调用方传入可填；此处无统计来源
    meta.has_loop_closure = false;  // 需调用方传入可填；此处无回环统计
    meta.loop_closure_count = 0;

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
