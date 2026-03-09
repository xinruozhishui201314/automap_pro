# AutoMap-Pro 未实现模块使用指南

> 生成时间: 2026-03-04
> 模块: PoseGraph, GlobalMap, MapBuilder, MapExporter, IcpRefiner增强

---

## 快速开始

### 1. 编译项目
```bash
cd /home/wqs/Documents/github/automap_pro/automap_ws
colcon build --packages-select automap_pro --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2. 运行建图
```bash
# 在线建图（实时传感器数据）
ros2 launch automap_pro automap_composable.launch.py \
    config:=/path/to/automap_pro/config/system_config.yaml

# 离线建图（ROS2 bag回放）
ros2 launch automap_pro automap_offline.launch.py \
    config:=/path/to/system_config_nya02.yaml \
    rosbag:=/path/to/nya_02_ros2.db3
```

### 3. 导出地图
```bash
# 服务调用导出
ros2 service call /automap/save_map "output_dir: '/tmp/my_map_export'"
```

### 4. 可视化（RViz2）

建图运行时，可用 RViz2 查看前端、后端、回环、GPS、HBA 等可视化：

```bash
# 使用包内配置（推荐）
rviz2 -d $(ros2 pkg prefix automap_pro)/share/automap_pro/rviz/automap.rviz
```

**Fixed Frame**：建议设为 `map`（与建图系统一致）；仅看前端时可设为 `camera_init`。

| 类别 | 话题 | 说明 |
|------|------|------|
| 点云 | `/automap/global_map` | 全局合并点云 |
| 点云 | `/automap/current_cloud` | 当前关键帧点云 |
| 点云 | `/automap/colored_cloud` | 按高度/强度着色点云 |
| 轨迹 | `/automap/odom_path` | 里程计轨迹 |
| 轨迹 | `/automap/optimized_path` | 优化后轨迹 |
| 回环 | `/automap/loop_markers` | 回环约束连线 |
| 子图 | `/automap/submap_boundaries` | 子图 ID 标签 |
| 子图 | `/automap/submap_bboxes` | 子图 3D 边界框 |
| 子图 | `/automap/submap_graph` | 子图连接图 |
| GPS | `/automap/gps_markers` | GPS 位置球体 |
| 后端 | `/automap/coordinate_frames` | 坐标轴 |
| 后端 | `/automap/hba_result` | HBA 优化位姿 |
| 后端 | `/automap/factor_graph` | 因子图（节点+ODOM/LOOP/GPS 边） |
| 调试 | `/automap/convergence_residual` | 优化残差序列（Float64MultiArray，供 PlotJuggler） |

更多话题见 `automap_pro/rviz/automap.rviz` 中的 Displays 列表。

---

## 模块使用示例

### PoseGraph（位姿图）

```cpp
#include "automap_pro/backend/pose_graph.h"

// 创建位姿图
automap_pro::PoseGraph graph;

// 添加节点
automap_pro::GraphNode node;
node.id = 0;
node.pose = automap_pro::Pose3d::Identity();
node.fixed = true;  // 锚定第一个节点
int node_id = graph.addNode(node);

// 添加Odom边
automap_pro::GraphEdge edge;
edge.from = 0;
edge.to = 1;
edge.type = automap_pro::EdgeType::ODOM;
edge.measurement = T_odom;  // 相对变换
edge.information = odom_cov;
graph.addEdge(edge);

// 添加Loop边
automap_pro::GraphEdge loop_edge;
loop_edge.from = 100;
loop_edge.to = 10;
loop_edge.type = automap_pro::EdgeType::LOOP;
loop_edge.measurement = T_loop;
loop_edge.information = loop_cov;
graph.addEdge(loop_edge);

// 查询最短路径
auto path = graph.findPath(0, 100);
double path_length = graph.computePathLength(path);

// 获取统计信息
int node_count = graph.nodeCount();
int edge_count = graph.edgeCount();
```

### GlobalMap（全局点云地图）

```cpp
#include "automap_pro/map/global_map.h"

// 创建全局地图
automap_pro::GlobalMap::Config config;
config.voxel_size = 0.1f;
config.max_distance = 200.0f;
config.enable_background_update = true;

automap_pro::GlobalMap global_map(config);

// 添加点云
automap_pro::CloudXYZIPtr cloud = ...;  // 你的点云
automap_pro::Pose3d T_w_b = ...;    // 位姿

// 同步添加
global_map.addCloud(cloud, T_w_b);

// 异步添加（推荐用于大规模数据）
global_map.addCloudAsync(cloud, T_w_b);
global_map.waitForUpdate();

// ROI查询（球体）
Eigen::Vector3d center(10.0, 20.0, 0.0);
auto roi_cloud = global_map.getMapROI(center, 50.0f);

// KNN查询（K近邻）
auto knn_positions = global_map.queryKNNPositions(center, 10);

// 获取地图统计
int point_count = global_map.getPointCount();
auto map_center = global_map.getMapCenter();
double map_radius = global_map.getMapRadius();

// 保存地图
global_map.savePCD("/tmp/global_map.pcd", true);

// 内存优化
global_map.optimizeMemory();
```

### MapBuilder（地图构建器）

```cpp
#include "automap_pro/map/map_builder.h"

// 创建地图构建器
automap_pro::MapBuilder::Config config;
config.base_voxel_size = 0.05f;
config.coarse_voxel_size = 0.2f;
config.fine_voxel_size = 0.02f;
config.enable_cleanup = true;
config.save_intermediate = false;

automap_pro::MapBuilder builder(config);

// 注册进度回调
builder.registerProgressCallback([](int processed, int total) {
    RCLCPP_INFO(rclcpp::get_logger("builder"), "Progress: %d/%d", processed, total);
});

// 从关键帧构建
std::vector<automap_pro::KeyFrame::Ptr> keyframes = ...;
builder.buildFromKeyFrames(keyframes);

// 增量添加关键帧
auto kf = std::make_shared<automap_pro::KeyFrame>();
kf->cloud_body = point_cloud;
kf->T_w_b = pose;
builder.addKeyFrame(kf);

// 获取多分辨率地图
auto base_map = builder.getBaseMap();      // 0.05m分辨率
auto coarse_map = builder.getCoarseMap();   // 0.2m分辨率
auto fine_map = builder.getFineMap();      // 0.02m分辨率

// 异步构建
builder.buildAsyncFromKeyFrames(new_keyframes);
builder.waitForBuild();

// 地图清理
builder.removeOutliers();
builder.removeDuplicatePoints(0.01f);
builder.cleanup();

// 获取统计信息
int processed_kf = builder.getProcessedKeyFrames();
int processed_sm = builder.getProcessedSubmaps();
size_t base_size = builder.getBaseMapSize();
```

### MapExporter（地图导出器）

```cpp
#include "automap_pro/map/map_exporter.h"

// 创建地图导出器
automap_pro::MapExporter::Config config;
config.output_dir = "/tmp/my_map_export";
config.compress = true;
config.save_trajectory = true;
config.save_submaps = true;
config.trajectory_format = "kml";

automap_pro::MapExporter exporter(config);

// 注册进度回调
exporter.registerProgressCallback([](int current, int total, const std::string& stage) {
    RCLCPP_INFO(rclcpp::get_logger("exporter"), "[%s] Progress: %d/%d", stage.c_str(), current, total);
});

// 导出地图
auto global_map = std::make_shared<automap_pro::GlobalMap>();
// ... 填充地图
exporter.exportMap(global_map, "my_map.pcd");

// 导出子图
std::vector<automap_pro::SubMap::Ptr> submaps = ...;
exporter.exportSubmaps(submaps, "submap");

// 导出轨迹
std::vector<automap_pro::KeyFrame::Ptr> keyframes = ...;
exporter.exportTrajectory(keyframes, "trajectory.txt");
exporter.exportTrajectoryKML(keyframes, "trajectory.kml");
exporter.exportTrajectoryCSV(keyframes, "trajectory.csv");

// 批量导出（地图+轨迹+子图+元数据）
exporter.exportAll(submaps, keyframes, "full_export");

// 异步导出
exporter.registerDoneCallback([](bool success, const std::string& path) {
    RCLCPP_INFO(rclcpp::get_logger("exporter"), "Export %s: %s", 
                success ? "SUCCESS" : "FAILED", path.c_str());
});
exporter.exportAllAsync(submaps, keyframes, "async_export");
exporter.waitForExport();

// 导出元数据
automap_pro::MapExporter::ExportMetadata meta;
meta.version = "2.0";
meta.timestamp = "2026-03-04 10:00:00";
meta.total_keyframes = keyframes.size();
meta.total_submaps = submaps.size();
meta.coordinate_system = "ENU";
exporter.exportMetadata(meta, "metadata.json");
```

### IcpRefiner（ICP配准器）

```cpp
#include "automap_pro/loop_closure/icp_refiner.h"

// 创建ICP配准器
automap_pro::IcpRefiner::Config config;
config.method = automap_pro::ICPMethod::POINT_TO_PLANE;
config.max_iterations = 50;
config.max_correspondence_distance = 0.5;
config.validate_result = true;
config.max_rotation_deg = 30.0;
config.max_translation_m = 5.0;

// 多尺度配准策略
config.enable_multiscale = true;
config.scale_levels = {0.5f, 0.25f, 0.1f};

automap_pro::IcpRefiner refiner(config);

// 基础配准
auto src_cloud = ...;
auto tgt_cloud = ...;
automap_pro::Pose3d initial_guess = ...;

auto result = refiner.refine(src_cloud, tgt_cloud, initial_guess);

if (result.converged) {
    std::cout << "ICP converged!" << std::endl;
    std::cout << "RMSE: " << result.rmse << std::endl;
    std::cout << "Rotation: " << result.rotation_angle_deg << " deg" << std::endl;
    std::cout << "Translation: " << result.translation_norm << " m" << std::endl;
    std::cout << "Time: " << result.elapsed_ms << " ms" << std::endl;
} else {
    std::cout << "ICP failed!" << std::endl;
}

// 多尺度配准
auto multiscale_result = refiner.refineMultiscale(src_cloud, tgt_cloud, initial_guess);

// 不同算法
config.method = automap_pro::ICPMethod::ICP;
auto icp_result = refiner.refine(src_cloud, tgt_cloud, initial_guess);

config.method = automap_pro::ICPMethod::GICP;
auto gicp_result = refiner.refine(src_cloud, tgt_cloud, initial_guess);

config.method = automap_pro::ICPMethod::NDT;
auto ndt_result = refiner.refine(src_cloud, tgt_cloud, initial_guess);

// 计算对应关系协方差
Eigen::Matrix6d cov = refiner.computeCorrespondenceCovariance(
    src_cloud, tgt_cloud, result.T_refined);
```

---

## 集成示例

### 完整建图+导出流程

```cpp
#include "automap_pro/map/global_map.h"
#include "automap_pro/map/map_builder.h"
#include "automap_pro/map/map_exporter.h"

// 1. 创建全局地图和构建器
automap_pro::GlobalMap global_map;
automap_pro::MapBuilder builder;
automap_pro::MapExporter exporter;

// 2. 注册回调
builder.registerProgressCallback([&](int processed, int total) {
    RCLCPP_INFO(rclcpp::get_logger("system"), "Building map: %d/%d", processed, total);
});

exporter.registerProgressCallback([&](int current, int total, const std::string& stage) {
    RCLCPP_INFO(rclcpp::get_logger("system"), "[%s] Exporting: %d/%d", stage.c_str(), current, total);
});

// 3. 处理关键帧
std::vector<automap_pro::KeyFrame::Ptr> keyframes;
for (const auto& raw_kf : raw_data) {
    auto kf = std::make_shared<automap_pro::KeyFrame>();
    kf->cloud_body = raw_kf.point_cloud;
    kf->T_w_b = raw_kf.pose;
    kf->covariance = raw_kf.cov;
    keyframes.push_back(kf);
    
    // 增量添加到构建器
    builder.addKeyFrame(kf);
}

// 4. 构建地图
builder.buildFromKeyFrames(keyframes);

// 5. 获取全局地图
auto global_cloud = global_map.getMap();

// 6. 导出地图
exporter.exportMap(global_cloud, "final_map.pcd");
exporter.exportTrajectory(keyframes, "final_trajectory.kml");
exporter.exportAll({}, keyframes, "final_export");
```

### 位姿图+回环集成

```cpp
#include "automap_pro/backend/pose_graph.h"
#include "automap_pro/loop_closure/icp_refiner.h"

automap_pro::PoseGraph pose_graph;
automap_pro::IcpRefiner icp_refiner;

// 1. 添加节点（关键帧位姿）
for (const auto& kf : keyframes) {
    automap_pro::GraphNode node;
    node.id = kf->id;
    node.pose = kf->T_w_b;
    pose_graph.addNode(node);
}

// 2. 添加Odom边
for (const auto& kf : keyframes) {
    automap_pro::GraphEdge edge;
    edge.from = kf->id;
    edge.to = kf->id + 1;
    edge.type = automap_pro::EdgeType::ODOM;
    edge.measurement = relative_pose;
    edge.information = odom_cov;
    pose_graph.addEdge(edge);
}

// 3. 回环检测后添加Loop边
auto loop_result = detectLoop(...);
if (loop_result.valid) {
    automap_pro::GraphEdge loop_edge;
    loop_edge.from = loop_result.query_id;
    loop_edge.to = loop_result.target_id;
    loop_edge.type = automap_pro::EdgeType::LOOP;
    
    // ICP精配准
    auto icp_result = icp_refiner.refine(
        query_cloud, target_cloud, loop_result.initial_guess);
    
    if (icp_result.converged) {
        loop_edge.measurement = icp_result.T_refined;
        loop_edge.information = compute_information(icp_result);
        pose_graph.addEdge(loop_edge);
    }
}

// 4. 查询路径验证
auto path = pose_graph.findPath(start_id, end_id);
double path_length = pose_graph.computePathLength(path);
```

---

## 配置调优建议

### PoseGraph
- **内存优化**: 定期清理旧节点
  ```cpp
  pose_graph.removeOldNodes(3600.0);  // 清理1小时前的节点
  ```

- **查询优化**: 使用邻接表加速
  ```cpp
  auto neighbors = pose_graph.getNeighborNodes(node_id);
  ```

### GlobalMap
- **性能优化**: 启用后台异步更新
  ```cpp
  config.enable_background_update = true;
  config.update_batch_size = 10000;
  ```

- **内存优化**: 设置最大点数
  ```cpp
  config.max_points = 5000000;  // 500万点
  config.voxel_size = 0.1f;     // 降采样
  ```

### MapBuilder
- **质量优化**: 启用去噪和清理
  ```cpp
  config.outlier_knn = 20;
  config.outlier_std_mul = 2.0;
  config.enable_cleanup = true;
  config.cleanup_interval = 100;
  ```

- **性能优化**: 合理设置分辨率
  ```cpp
  config.base_voxel_size = 0.05f;     // 基础分辨率
  config.coarse_voxel_size = 0.2f;     // 粗糙分辨率
  config.fine_voxel_size = 0.02f;      // 精细分辨率
  ```

### IcpRefiner
- **精度优化**: 使用多尺度策略
  ```cpp
  config.enable_multiscale = true;
  config.scale_levels = {0.5f, 0.25f, 0.1f};
  ```

- **鲁棒性优化**: 启用结果验证
  ```cpp
  config.validate_result = true;
  config.max_rotation_deg = 30.0;
  config.max_translation_m = 5.0;
  config.min_inlier_ratio = 0.3;
  ```

---

## 故障排查

### 问题1: 编译错误
**现象**: undefined reference to `PoseGraph::...`
**解决**: 
```bash
cd automap_ws
colcon build --packages-select automap_pro --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 问题2: 地图导出失败
**现象**: Failed to save PCD
**解决**: 检查目录权限和磁盘空间
```bash
ls -lh /data/automap_export
df -h
```

### 问题3: 内存占用过高
**现象**: OOM或内存持续增长
**解决**: 启用降采样和清理
```cpp
global_map.optimizeMemory();
builder.cleanup();
```

### 问题4: ICP配准失败
**现象**: ICP NOT converged
**解决**: 
- 调整初始猜测（更接近真实值）
- 增大max_correspondence_distance
- 尝试不同算法（GICP/NDT）

### 问题5: 位姿图路径错误
**现象**: findPath返回空
**解决**: 
- 检查节点ID是否正确
- 确认边连接正确
- 检查图是否连通

---

## 性能基准

| 操作 | 预期性能 | 备注 |
|------|----------|------|
| PoseGraph.addNode() | < 0.1ms | 互斥锁开销 |
| PoseGraph.findPath() | O(V+E) | BFS算法 |
| GlobalMap.addCloud(1000pts) | < 10ms | 降采样后 |
| GlobalMap.queryRadius() | < 5ms | KD树O(log n) |
| MapBuilder.addKeyFrame() | < 20ms | 批量处理 |
| MapExporter.exportPCD(1M pts) | < 500ms | 压缩后 |
| IcpRefiner.refine(10k pts) | < 50ms | Point-to-Plane |

---

## 联系方式

- GitHub Issues: [automap_pro/issues]
- 文档: `docs/IMPLEMENTATION_SUMMARY.md`
- 配置: `docs/CONFIG_SUMMARY.md`
