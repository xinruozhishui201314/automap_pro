#pragma once
/**
 * @file map/map_exporter.h
 * @brief 全局地图：增量构建、体素/KD、导出与滤波。
 */


#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <queue>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PolygonMesh.h>

#include "automap_pro/core/data_types.h"
#include "automap_pro/map/global_map.h"
#include "automap_pro/submap/submap_manager.h"

namespace automap_pro {

// ─────────────────────────────────────────────────────────────────────────────
// MapExporter: 地图导出与可视化工具
// 支持格式：
//   - PCD (Point Cloud Data)
//   - PLY (Polygon File Format)
//   - OBJ (Wavefront OBJ)
//   - Las/Laz (激光雷达格式，可选）
// 功能：
//   - 多分辨率地图导出
//   - 姿态轨迹导出（TXT/KML/CSV）
//   - 子图分割导出
//   - 增量式导出（仅新增部分）
//   - 压缩选项
// ─────────────────────────────────────────────────────────────────────────────

class MapExporter {
public:
    struct Config {
        std::string output_dir = "/data/automap_export";
        bool        compress = true;
        bool        save_trajectory = true;
        bool        save_submaps = true;
        bool        save_metadata = true;
        bool        save_rgb = false;       // PLY格式支持RGB

        float       base_resolution = 0.05f;
        float       fine_resolution = 0.02f;

        std::string trajectory_format = "txt";  // txt/kml/csv

        bool        export_incremental = false;   // 仅导出新增部分
        int         last_exported_kf = -1;     // 最后导出的关键帧ID

        std::string coordinate_system = "ENU";    // 地图坐标系统一为 ENU

        // WGS84 原点坐标（用于 ENU→WGS84 转换）
        double      origin_latitude = 0.0;    // 纬度（度）
        double      origin_longitude = 0.0;   // 经度（度）
        double      origin_altitude = 0.0;    // 高度（米）
        bool        origin_valid = false;     // 原点是否有效
    };

    struct ExportMetadata {
        std::string version = "2.0";
        std::string timestamp;
        int         total_keyframes = 0;
        int         total_submaps = 0;
        size_t      total_points = 0;
        std::string coordinate_system;
        double      map_center[3];
        double      map_radius;
        
        // 传感器信息
        std::string lidar_type;
        std::string lidar_config;
        std::string imu_type;
        
        // 优化信息
        bool        has_loop_closure = false;
        int         loop_closure_count = 0;
        bool        has_gps = false;
        int         gps_constraints = 0;
    };

    using ProgressCallback = std::function<void(int current, int total, const std::string& stage)>;
    using DoneCallback = std::function<void(bool success, const std::string& path)>;
    
    MapExporter();
    explicit MapExporter(const Config& config);
    ~MapExporter();

    // 配置
    void setConfig(const Config& config);
    Config getConfig() const;

    // 导出地图
    bool exportMap(const CloudXYZIPtr& cloud, 
                   const std::string& filename = "map.pcd");
    bool exportMap(const std::shared_ptr<GlobalMap>& global_map,
                   const std::string& filename = "global_map.pcd");
    bool exportSubmaps(const std::vector<SubMap::Ptr>& submaps,
                       const std::string& filename_prefix = "submap");
    
    // 导出轨迹
    bool exportTrajectory(const std::vector<KeyFrame::Ptr>& keyframes,
                          const std::string& filename = "trajectory.txt");
    bool exportTrajectoryKML(const std::vector<KeyFrame::Ptr>& keyframes,
                              const std::string& filename = "trajectory.kml");
    bool exportTrajectoryCSV(const std::vector<KeyFrame::Ptr>& keyframes,
                            const std::string& filename = "trajectory.csv");
    
    // 导出元数据
    bool exportMetadata(const ExportMetadata& metadata,
                       const std::string& filename = "metadata.json");
    bool exportMapInfo(const std::vector<SubMap::Ptr>& submaps,
                       const std::vector<KeyFrame::Ptr>& keyframes,
                       const std::string& filename = "map_info.yaml");
    
    // 批量导出
    bool exportAll(const std::vector<SubMap::Ptr>& submaps,
                   const std::vector<KeyFrame::Ptr>& keyframes,
                   const std::string& output_name = "automap_export");
    
    // 异步导出
    void exportAllAsync(const std::vector<SubMap::Ptr>& submaps,
                       const std::vector<KeyFrame::Ptr>& keyframes,
                       const std::string& output_name = "automap_export");
    
    // 坐标转换（地图坐标系统一为 ENU）
    /**
     * ENU坐标转WGS84经纬度
     * @param enu_pt ENU坐标点 (east, north, up)
     * @param origin_lat 原点纬度（度）
     * @param origin_lon 原点经度（度）
     * @param origin_alt 原点高度（米）
     * @return WGS84坐标 (latitude, longitude, altitude)
     */
    Eigen::Vector3d enuToWGS84(const Eigen::Vector3d& enu_pt,
                                double origin_lat, double origin_lon,
                                double origin_alt) const;
    
    // 回调
    void registerProgressCallback(ProgressCallback cb);
    void registerDoneCallback(DoneCallback cb);
    
    // 等待导出完成
    void waitForExport();
    
    // 统计信息
    size_t getLastExportSize() const;
    std::string getLastExportPath() const;

private:
    struct ExportTask {
        std::vector<SubMap::Ptr> submaps;
        std::vector<KeyFrame::Ptr> keyframes;
        std::string output_name;
    };

    mutable std::mutex mutex_;
    
    Config config_;
    
    // 导出统计
    std::atomic<size_t> last_export_size_;
    std::string last_export_path_;
    
    // 后台导出线程
    std::queue<ExportTask> export_queue_;
    std::condition_variable export_cv_;
    std::thread export_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> export_pending_;
    
    // 回调
    std::vector<ProgressCallback> progress_cbs_;
    std::vector<DoneCallback> done_cbs_;
    
    // 后台导出循环
    void exportThreadLoop();
    void processExportTask(const ExportTask& task);
    
    // 导出实现
    bool exportPCD(const CloudXYZIPtr& cloud, const std::string& path);
    bool exportPLY(const CloudXYZIPtr& cloud, const std::string& path);
    bool exportOBJ(const CloudXYZIPtr& cloud, const std::string& path);
    bool exportLAS(const CloudXYZIPtr& cloud, const std::string& path);  // 可选
    
    // 降采样
    CloudXYZIPtr downsampleCloud(const CloudXYZIPtr& cloud, float resolution) const;
    
    // 辅助函数
    std::string getCurrentTimestamp() const;
    void ensureOutputDir(const std::string& dir);
    ExportMetadata buildMetadata(const std::vector<SubMap::Ptr>& submaps,
                               const std::vector<KeyFrame::Ptr>& keyframes) const;
    void notifyProgressCallbacks(int current, int total, const std::string& stage);
    void notifyDoneCallbacks(bool success, const std::string& path);
};

} // namespace automap_pro
