# HBA 轨迹 vs GPS 轨迹 + 偏差 显示方案讨论

## 0. 目标

- **两条曲线**：HBA 优化轨迹、GPS 轨迹（map 系，关键帧级一一对应）。
- **偏差曲线**：能直观看到“偏差随轨迹/时间”的变化（曲线形态，而不只是线段或点云）。

---

## 1. RViz 是否依赖 VTK？

**结论：不依赖。**

| 组件 | 渲染/可视化后端 | 说明 |
|------|------------------|------|
| **RViz2** | **OGRE** (Object-Oriented Graphics Rendering Engine) | 通过 `rviz_rendering` + `rviz_ogre_vendor`，使用 OGRE 1.12 做 3D 渲染；不依赖 VTK。 |
| **PCL Visualizer** | **VTK** (Visualization Toolkit) | PCL 的 `pcl_visualization` 模块依赖 VTK；和 RViz 是两套独立技术栈。 |

因此：
- 在 **RViz 里** 显示 = 用 OGRE 渲染，发 ROS 消息（Path / Marker / PointCloud2）即可。
- 若要用 **VTK** 显示 = 需单独写基于 VTK（或 PCL Visualizer）的可视化程序，与 RViz 并行或替代 RViz。

---

## 2. 能否用 VTK 显示？

**可以。** 常见几种方式：

| 方式 | 依赖 | 集成方式 | 适用场景 |
|------|------|----------|----------|
| **A. 纯 VTK** | VTK (C++/Python) | 独立可执行程序/脚本，读 CSV 或订阅 ROS | 离线分析、定制 2D/3D 曲线、科研绘图 |
| **B. PCL Visualizer** | PCL + VTK | 独立节点或后处理工具，底层 VTK 渲染 | 已有 PCL 时顺带用 VTK 窗口 |
| **C. RViz** | OGRE，无 VTK | 当前工程已在用，发 Path/Marker/PointCloud2 | 在线建图后立刻看，与地图同屏 |

若希望“**两条曲线 + 偏差曲线**”形态更接近**折线图/曲线图**（横轴时间或里程、纵轴偏差），VTK 或 matplotlib 等更适合画 2D 曲线；RViz 更适合 3D 轨迹 + 3D 偏差线段。

---

## 3. “两条曲线 + 偏差曲线”的几种实现形态

### 3.1 当前已有（RViz，OGRE）

- **两条轨迹**：`/automap/optimized_path`（HBA）、`/automap/gps_keyframe_path`（GPS）→ 3D 空间两条折线。
- **偏差线段**：`/automap/hba_gps_deviation` → 每帧一段 3D 线段（HBA 点 → GPS 点）。
- **点云**：`/automap/hba_trajectory_points`（绿）、`/automap/gps_trajectory_points`（蓝）。

**优点**：无需新依赖，建图结束即可在 RViz 看。  
**缺点**：“偏差曲线”是 3D 线段，不是“偏差值 vs 时间/里程”的 2D 曲线。

---

### 3.2 在 RViz 里近似“偏差曲线”（仍用 OGRE）

思路：把**偏差随时间的标量**变成 3D 里的一条线（例如横轴=时间或里程，纵轴=0，竖轴=偏差米数），在 RViz 用 Path 或 Marker LINE_STRIP 发布。

- **曲线 1**：HBA 轨迹（已有）。
- **曲线 2**：GPS 轨迹（已有）。
- **曲线 3**：`(t_i, 0, deviation_i)` 或 `(distance_i, 0, deviation_i)` 作为 Path 发布，在 RViz 里看到“偏差随 t/距离”的起伏。

**优点**：不引入 VTK，仍在 RViz 内完成。  
**缺点**：是 3D 中的一条线，不是标准 2D 坐标轴曲线；需要选好缩放和视角才能看清。

---

### 3.3 用 VTK 显示（独立程序）

- 读 `trajectory_hba_poses_*.csv`（或订阅 ROS）。
- 用 VTK 画：
  - **两条 3D 曲线**：HBA (x,y,z)、GPS (x,y,z)；
  - **一条 2D 偏差曲线**：横轴=时间或里程，纵轴=偏差(m)，或 3D 中 (distance, 0, deviation)。

**优点**：完全可控的 2D/3D 曲线、图例、坐标轴、多视图。  
**缺点**：需额外可执行程序/脚本和 VTK 依赖，不与 RViz 同进程。

---

### 3.4 用 Python + Matplotlib / PyVista（轻量 2D 曲线 or 3D）

- 读同一 CSV，用 **matplotlib** 画 2D：横轴时间/里程，纵轴偏差（或两条轨迹的 x/y 对比）。
- 或用 **PyVista**（基于 VTK）画 3D 两条轨迹 + 偏差曲线。

**优点**：2D 曲线最直观；PyVista 可复用 VTK 能力。  
**缺点**：离线或需单独起脚本，非 RViz 内。

---

## 4. 方案对比小结

| 方案 | 两条轨迹曲线 | 偏差曲线形态 | 依赖 | 集成 |
|------|--------------|--------------|------|------|
| **现状（RViz）** | Path 两条 3D 折线 | 3D 线段（每帧一段） | 无新增 | 建图结束即看 |
| **RViz + 偏差 Path** | 同上 | 3D 一条线 (t/d, 0, dev) | 无 | 新增一个 Path 发布 |
| **VTK 独立程序** | VTK 3D 线 | 2D 或 3D 曲线 | VTK | 独立 exe/脚本 |
| **Python + matplotlib** | 2D 投影或 3D | 2D 偏差 vs 时间/里程 | Python + matplotlib | 离线脚本 |
| **Python + PyVista** | 3D | 2D/3D 偏差曲线 | VTK + PyVista | 离线脚本 |

---

## 5. 建议取舍

- **若希望“建图完立刻在现有环境看到两条曲线 + 偏差”且不增加依赖**  
  → 保留当前 RViz 两条 Path + 偏差线段，并**增加“偏差曲线”Path**：在 map 系下发布一条 `(时间或里程, 0, deviation)` 的 Path，在 RViz 中当“偏差随轨迹”的曲线看（见 3.2）。

- **若希望“标准 2D 偏差曲线图”（横轴时间/里程，纵轴偏差）**  
  → 用 **Python + matplotlib** 写一个读 `trajectory_hba_poses_*.csv` 的脚本，画两条轨迹（可选）+ **偏差 vs 时间/里程** 曲线；不依赖 VTK，易维护。

- **若希望完全用 VTK、且要 3D+2D 多视图**  
  → 单独用 **VTK（C++）或 PyVista（Python）** 写一个可视化程序，读同一 CSV，画两条 3D 曲线和偏差曲线；与 RViz 并行使用。

---

## 6. 工程内 C++ VTK 轨迹查看器（已实现）

工程内已提供基于 **VTK + ROS2** 的 C++ 工具 `vtk_trajectory_viewer`，用于在建图过程中显示两条曲线和偏差曲线。

### 6.1 功能

- 订阅 **/automap/optimized_path**（HBA 轨迹）、**/automap/gps_keyframe_path**（GPS 轨迹）。
- VTK 窗口显示：
  - **绿色**：HBA 轨迹（3D 折线）
  - **蓝色**：GPS 轨迹（3D 折线）
  - **红色**：偏差曲线（横轴=累积里程，竖轴=偏差 m，便于看偏差随路径变化）

### 6.2 依赖与构建

- **依赖**：VTK（如 Ubuntu：`sudo apt install libvtk9-dev`）。Docker 镜像中已安装 PCL（`libpcl-dev`）时，PCL 的 visualization 组件本身依赖 VTK，**多数情况下系统已带有 VTK**；工程内 `docker/dockerfile` 已显式加入 `libvtk9-dev`，保证镜像内能构建 `vtk_trajectory_viewer`。
- **构建**：VTK 为可选；未安装 VTK 时该目标不构建，不影响主工程。
  ```bash
  # 安装 VTK 后
  colcon build --packages-select automap_pro
  ```
- **运行**：
  - **自动启动（默认）**：HBA 优化完成后，建图进程会**自动在后台启动** `vtk_trajectory_viewer`，无需另开终端。两条 Path 使用 **TransientLocal** QoS，查看器启动后能收到 HBA 刚发布的最后一次数据，窗口会直接显示两条曲线与偏差曲线。
  - **关闭自动启动**：launch 时传参 `vtk_viewer_after_hba:=false`。
  - **手动运行**：也可随时手动执行 `ros2 run automap_pro vtk_trajectory_viewer`（建图运行且已发布过 Path 时，同样能收到最后一次数据）。

### 6.3 建图后自动保存精度结果（时间戳目录）

HBA 完成且 GPS 已对齐时，系统会：

1. **创建带时间戳的目录**：`<output_dir>/YYYYMMDD_HHMM`（如 `automap_output/20260317_2030`）。`output_dir` 来自配置 `system.output_dir` 或 launch 参数 `output_dir`。
2. **将点云与轨迹写入该目录**：调用 `saveMapToFiles(timestamped_dir)`，保存 `global_map.pcd`、`trajectory_tum.txt`、`keyframe_poses.pcd`、`gps_positions_map.pcd`、`trajectory_odom.csv`、session 归档等（与手动“保存地图”内容一致）。
3. **启动 VTK 查看器并传入该目录**：通过环境变量 `AUTOMAP_ACCURACY_SAVE_DIR` 传入时间戳目录路径。
4. **VTK 首次渲染成功后保存高清图**：将“两条轨迹 + 偏差曲线”窗口保存为 **accuracy_curves.png**（2x 分辨率）到同一时间戳目录。

建图结束后可直接打开 `<output_dir>/YYYYMMDD_HHMM` 查看，**精度相关、便于查看**的文件包括：

| 文件 | 说明 | 查看方式 |
|------|------|----------|
| **accuracy_curves.png** | 两条轨迹 + 偏差曲线高清图（与 VTK 窗口一致） | 图片查看器 |
| **accuracy_trajectories.csv** | 逐关键帧：HBA xyz、GPS xyz、偏差(m)、累积里程(m)，与图中曲线一一对应 | Excel / Python 绘图 |
| **deviation_curve.csv** | 两列：cum_dist_m, deviation_m，直接做“偏差随里程”2D 图 | Excel / matplotlib |
| **accuracy_summary.txt** | 摘要：keyframe_count, mean_deviation_m, max_deviation_m, cum_dist_total_m | 文本编辑器 |
| **global_map.pcd** | 全局点云地图 | PCL / CloudCompare |
| **trajectory_tum.txt** | 优化后轨迹（TUM 格式） | 文本/脚本 |
| **keyframe_poses.pcd** | 关键帧位置点云 | PCL / CloudCompare |
| **gps_positions_map.pcd** | 地图系下 GPS 位置点云 | PCL / CloudCompare |
| **trajectory_odom_*.csv** | 关键帧位姿 + GPS 列（含时间戳、协方差等） | Excel / 脚本 |
| **trajectory_hba_poses_*.csv** | 若存在：HBA 位姿序列 | Excel / 脚本 |

### 6.4 为何建图完成后时间戳目录下没有 accuracy_curves.png？（已修复）

此前**仅由 VTK 子进程**在首次渲染成功后写入 `accuracy_curves.png`、`accuracy_trajectories.csv`、`deviation_curve.csv`、`accuracy_summary.txt`。以下情况会导致目录下**没有这些文件**：

| 原因 | 说明 |
|------|------|
| VTK 未参与构建 | `find_package(VTK)` 未找到（未安装 `libvtk9-dev`）时，`vtk_trajectory_viewer` 不构建，HBA 后 `ros2 run automap_pro vtk_trajectory_viewer` 会失败。 |
| 无 DISPLAY | 无头环境或 SSH 无 X11 时，VTK 无法打开窗口，可能无法完成首次渲染与截图。 |
| 环境变量未传递 | VTK 由 `std::system("ros2 run ...")` 在子进程启动，个别环境下 `AUTOMAP_ACCURACY_SAVE_DIR` 未继承到子进程。 |
| 数据未就绪 | VTK 要求 `optimized_path` 与 `gps_keyframe_path` 点数一致；若未满足则 `pullData()` 一直为 false，不会保存。 |

**当前行为（修复后）**：在 GPS 已对齐时，**主进程**会在创建时间戳目录后：

1. **直接写入** `accuracy_trajectories.csv`、`deviation_curve.csv`、`accuracy_summary.txt`（与 VTK 格式一致）。
2. **调用** `scripts/plot_accuracy_curves.py` 生成 **accuracy_curves.png**（不依赖 VTK、不依赖 DISPLAY）。脚本需安装到 `share/automap_pro/scripts/`（`install(PROGRAMS scripts/plot_accuracy_curves.py ...)`），并需 `python3` + `matplotlib`（可选 `numpy`/`pandas`）。

因此建图完成后，只要 `gps_aligned_` 为 true，时间戳目录下应**至少**有 CSV 与 summary；若安装了脚本与 Python 依赖，还会有 **accuracy_curves.png**。VTK 仍可选用于交互查看，其保存的 PNG 会覆盖同路径文件。

**手动补生成图片**（若当次建图未生成 PNG）：

```bash
python3 /path/to/install/automap_pro/share/automap_pro/scripts/plot_accuracy_curves.py --dir /path/to/automap_output/20260317_2137
```

### 6.5 源码位置

- 节点实现：`automap_pro/src/tools/vtk_trajectory_viewer_node.cpp`
- HBA 后时间戳目录创建与保存：`automap_pro/src/system/automap_system.cpp`（`saveMapToFiles` + 主进程写精度 CSV/摘要 + 调用 `plot_accuracy_curves.py` + `setenv("AUTOMAP_ACCURACY_SAVE_DIR", ...)` + 启动 VTK）。
- 精度曲线图脚本：`automap_pro/scripts/plot_accuracy_curves.py`（安装到 `share/automap_pro/scripts/`）。
- CMake：`automap_pro/CMakeLists.txt` 中“VTK 轨迹曲线显示工具”段落与 `install(PROGRAMS scripts/plot_accuracy_curves.py ...)`。
