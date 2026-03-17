# RViz 显示的点云与本地保存的全局点云「整体偏移/旋转」说明

## 0. 结论摘要

| 现象 | 常见原因 | 处理要点 |
|------|----------|----------|
| **整体**平移+旋转（不是局部重影） | ① RViz Fixed Frame 与点云 frame_id 不一致且 TF 非单位变换；② 在其它软件中打开 PCD 时的坐标系/轴向约定与 RViz 不同；③ 加载 PCD 回 RViz 时使用了错误的 frame_id | 固定 Frame=map、确认 TF、统一坐标系约定 |

本仓库中：**发布到 RViz 的 `/automap/global_map` 与保存的 `global_map.pcd` 来自同一套 `buildGlobalMap()` 结果，且都处于「map」坐标系**，理论上应一致；若出现整体偏移/旋转，多为**显示/对照方式**（Fixed Frame、TF、外部查看器约定）导致。

---

## 1. 代码侧：发布与保存是否同一坐标系？

- **发布**（`automap_system.cpp` 约 2555–2562）：  
  `buildGlobalMap()` / `buildGlobalMapAsync()` → `cloud_msg.header.frame_id = "map"` → 发布到 `/automap/global_map`。
- **保存**（`automap_system.cpp` 约 3301–3307）：  
  同样 `buildGlobalMap()` / `buildGlobalMapAsync()` → `pcl::io::savePCDFileBinary(pcd_path, *global)`，**无额外变换**。

因此：**同一次运行中，发布到 RViz 的点云与写入 `global_map.pcd` 的数值是在同一坐标系（map）下的**，不会在代码里一个用 map、一个用 odom 或 ENU 导致整体偏移。

---

## 2. 整体偏移/旋转的常见原因

### 2.1 RViz Fixed Frame ≠ 点云 frame_id，且 TF 非单位

- 点云 `header.frame_id = "map"`。
- 若 RViz 的 **Fixed Frame** 设为 `odom`、`world`、`camera_init` 等，RViz 会用 TF 把点云从 `map` 变换到 Fixed Frame 再显示。
- 若存在 **map → Fixed Frame** 的非单位变换（例如某节点发布了 `map`→`odom` 或 `world`→`map` 且带平移/旋转），则：
  - 在 RViz 里看到的是「变换后的位置」；
  - 而 `global_map.pcd` 里存的是「map 系下的原始 XYZ」；
  - 二者就会呈现**整体平移+旋转**。

**处理**：

- 将 RViz **Fixed Frame** 设为 **`map`**（与 `/automap/global_map` 的 frame_id 一致），这样不再对点云做 TF 变换，显示即「map 系」。
- 若必须用其它 Fixed Frame，需保证 TF 树中 **map → 该 Frame** 为单位变换，或接受「在 RViz 中看到的是该 Frame 下的结果，与 PCD 的 map 系不同」这一事实。

### 2.2 Launch 中的 static_transform

- `automap_offline.launch.py` 中通常有：
  - `world` → `map`：单位变换 `(0,0,0,0,0,0)`；
  - `map` → `camera_init`：单位变换。
- 若你修改过或使用了其它 launch，发布了 **非单位**的 `world`→`map` 或 `map`→`xxx`，则以 `world`/`xxx` 为 Fixed Frame 时，就会相对「map 系下的 PCD」出现整体偏移/旋转。

**处理**：检查所有 `static_transform_publisher` 和任何发布 `map`/`world`/`odom` 的节点，确认与预期一致（若希望 RViz 与 PCD 一致，应保证以 `map` 为参考时无多余变换）。

### 2.3 在其它软件中打开 PCD 时的坐标系/轴向

- PCD 文件只存 XYZ（及强度等），**不存 frame_id 和朝向说明**。
- 本系统 **map 系** 与 **GPS 对齐后的 ENU** 一致：  
  **X 东、Y 北、Z 天**（右手系）。
- 若在 CloudCompare、MeshLab、MATLAB 等中打开 `global_map.pcd` 时，软件默认是 **Z 朝上** 或 **Y 朝上** 等不同约定，或对「东/北」的轴向解释不同，**同一组 XYZ 会看起来像整体旋转**（尤其是绕竖直轴）。

**处理**：在外部软件中查看时，按 **X=东、Y=北、Z=天** 理解；若软件有「坐标系/轴向」选项，设为与 ENU 一致再与 RViz 对比。

### 2.4 把 PCD 再加载回 RViz 时 frame_id 设错

- 若通过「从文件加载点云」等方式把 `global_map.pcd` 再在 RViz 里显示，且为该显示设置了 **错误的 frame_id**（例如 `odom`、`base_link`），而 TF 中该 frame 与 `map` 又有非单位变换，则会相对「实时发布的 `/automap/global_map`」出现整体偏移/旋转。

**处理**：加载的 PCD 在 RViz 中的 **Frame** 应设为 **`map`**，与实时话题一致。

---

## 3. 建议自检清单

1. **RViz**  
   - [ ] Fixed Frame = **`map`**（与 `/automap/global_map` 的 frame_id 一致）。  
   - [ ] 未对 `/automap/global_map` 做额外变换（例如误用 Transform 等）。

2. **TF**  
   - [ ] `ros2 run tf2_tools view_frames` 生成 TF 图，确认 `map` 与 `world`/`odom`/`camera_init` 等关系。  
   - [ ] 若 Fixed Frame 不是 `map`，确认 map→Fixed Frame 的变换是否符合预期（若希望与 PCD 一致，应为单位变换或你已知的固定变换）。

3. **外部查看 PCD**  
   - [ ] 按 **X 东、Y 北、Z 天（ENU）** 理解；  
   - [ ] 与 RViz 对比时，RViz Fixed Frame 为 `map`，且无多余 static_transform 导致 map 被整体变换。

4. **加载 PCD 回 RViz**  
   - [ ] 该显示的 Frame 设为 **`map`**。

---

## 4. 若仍存在整体偏移/旋转

可补充以下信息便于进一步排查：

- RViz **Fixed Frame** 当前设置（如 `map` / `world` / `odom` / `camera_init` 等）。
- 是否修改过 launch 中的 **static_transform_publisher**（特别是与 `map`、`world` 相关的）。
- 「本地记录的全局点云」具体指：**直接打开的 `global_map.pcd`**，还是 **录 bag 再播放的 `/automap/global_map`**，以及是在 **RViz 里对比** 还是在 **CloudCompare 等外部软件** 里对比。

---

*代码依据：`automap_system.cpp` 发布与保存均使用同一 `buildGlobalMap()` 结果，且未对保存再做变换；launch 见 `automap_offline.launch.py` 中 world→map、map→camera_init 的 static_transform。*
