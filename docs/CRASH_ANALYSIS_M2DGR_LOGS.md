# M2DGR 运行崩溃分析报告（automap.log / full.log）

## 0. Executive Summary

| 项目 | 结论 |
|------|------|
| **崩溃进程** | **fast_livo**（`fastlivo_mapping`），非 automap_system_node |
| **退出码** | **-11（SIGSEGV）**，段错误 |
| **发生时机** | 第 10 帧点云发布完成后（`published #10 done` 之后） |
| **配置文件** | 已确认**唯一**使用 `system_config_M2DGR.yaml` |
| **最可能原因** | 第 10 帧触发 `/Laser_map` 发布路径（体素下采样或发布）导致异常；次之为 PCD/轨迹写或下一帧处理路径 |

**建议**：用 GDB 单独挂载 `fastlivo_mapping` 复现并取 backtrace；临时将 `publish.laser_map_interval` 调大或设为 0 做规避验证。

---

## 1. 背景与目标

- **日志来源**：`logs/automap.log`（构建+运行总览）、`logs/full.log`（本次运行 ROS2/节点输出）。
- **运行方式**：离线 bag 回放，配置 **仅** 指定 `system_config_M2DGR.yaml`，GDB 仅挂载了 `automap_system_node`，未挂载 fast_livo。
- **目标**：定位崩溃根因、确认配置唯一性，并给出可落地的修复/规避与验证步骤。

---

## 2. 配置唯一性确认

从 **full.log** 可确认全工程唯一配置文件为 M2DGR 配置：

```
[automap_offline] [CONFIG] 全工程唯一配置文件: /root/automap_ws/src/automap_pro/config/system_config_M2DGR.yaml
[AutoMapSystem][CONFIG] config_file param=/root/automap_ws/src/automap_pro/config/system_config_M2DGR.yaml
[ConfigManager] Successfully loaded config from: /root/automap_ws/src/automap_pro/config/system_config_M2DGR.yaml
```

fast_livo 参数由同一配置通过 launch 生成临时 YAML 注入：

- `params_file_for_fast_livo=/tmp/automap_fl_params_offline_106.yaml`
- 来源：`get_fast_livo2_params` 从 `system_config_M2DGR.yaml` 的 `fast_livo` 等节合并生成。

**结论**：必须且唯一使用 `system_config_M2DGR.yaml` 的要求已满足。

---

## 3. 崩溃现象与时间线

### 3.1 崩溃事件

```
[fastlivo_mapping-2] [INFO] [fast_livo][PUB] about_to_publish #10 ts=1628249878.307 pts=15859 ...
[fastlivo_mapping-2] [INFO] [fast_livo][PUB] published #10 done (publish() took 0.00 ms; ...)
[ERROR] [fastlivo_mapping-2]: process has died [pid 132, exit code -11, cmd '.../fastlivo_mapping ...']
```

- **exit code -11**：SIGSEGV（非法内存访问）。
- **进程**：`fastlivo_mapping`（pid 132），即前端 fast_livo 节点。

### 3.2 时间线摘要

| 时间/顺序 | 事件 |
|-----------|------|
| 启动 | 使用 `system_config_M2DGR.yaml`，LIO-only（img_en=0），topic：/velodyne_points, /handsfree/imu |
| 前 2 帧 | 点云为空被跳过（No point!!!），未发布 /cloud_registered |
| 第 1 帧有效 | 发布 #1，创建 KF，sm_id=0 |
| 第 2–5 帧 | 正常发布；后端按 process_every_n_frames=5 处理 |
| 第 6 帧 | 后端 skip_no_kf（距离/旋转/间隔未达阈值） |
| ~第 6 帧后 | 首条 GPS 收到，ENU 原点设定，trajectory_gps_*.csv 创建 |
| 第 7–9 帧 | 正常发布 |
| **第 10 帧** | **about_to_publish #10 → published #10 done → 进程退出 -11** |

崩溃发生在「第 10 帧」发布日志之后、下一次 spin 或同一次 handleLIO 后续代码之间。

---

## 4. 代码路径与可能崩溃点

### 4.1 调用链（LIO 模式）

1. `run()` → `sync_packages()` → `stateEstimationAndMapping()` → `handleLIO()`  
2. `handleLIO()` 末尾：`publish_frame_world(pubLaserCloudFullRes, vio_manager);`（此处打出 "published #10 done"）  
3. `publish_frame_world()` 返回后，在 `handleLIO()` 中继续执行：
   - `*pcl_laser_map_accum_ += *pcl_w_wait_pub`
   - 若 `(frame_num + 1) % laser_map_pub_interval_ == 0`（默认 10），则进入 **/Laser_map 发布分支**：
     - 对 `pcl_laser_map_accum_` 做 **VoxelGrid 下采样**（`laser_map_voxel_size_` = 0.2）
     - `*pcl_laser_map_accum_ = *to_pub`（用下采样结果替换累积）
     - `pcl::toROSMsg(*to_pub, laser_map_msg)` 并 `pubLaserCloudMap->publish(laser_map_msg)`
   - 随后 `publish_path()`、`publish_mavros()`、`frame_num++` 等

第 10 帧时（frame_num 仍为 9）：`(frame_num+1) % 10 == 0`，**会首次进入上述 /Laser_map 发布分支**，且此时累积点云约 10 帧量级（约 15k×10 点），体素滤波与赋值、发布均可能触发异常（PCL 内部、序列化或 DDS）。

### 4.2 其他可能路径（次要）

- **publish_frame_world 内部**：PCD 保存、`fout_lidar_pos` 写入（1566–1562 行）。若文件未正确打开或并发写，理论上有风险，但当前为单线程 run loop，概率较低。
- **下一帧**：下一次 `sync_packages` / `handleLIO` 中体素图或点云处理。若崩溃在「published #10 done」之后极短时间内，则更符合「同帧内 publish 之后、Laser_map 或 path 发布」的路径。

### 4.3 LIO-only 与 vio_manager

- `img_en=0` 时未加载 camera、未初始化 VIO，`publish_frame_world` 中仅当 `LidarMeasures.lio_vio_flg == VIO` 时访问 `vio_manager`（如 RGB 投影、img_save）。
- 当前为 LIO，`lio_vio_flg == LIO`，故**不会**在 publish_frame_world 内走 VIO 分支，**崩溃与 vio_manager 解引用无关**。

---

## 5. 结论与根因归纳

| 结论项 | 说明 |
|--------|------|
| 配置 | 唯一使用 `system_config_M2DGR.yaml`，符合要求 |
| 崩溃进程 | fast_livo（fastlivo_mapping），SIGSEGV |
| 触发时机 | 第 10 帧点云发布完成后，极可能为**第 10 帧首次触发 /Laser_map 发布**（体素下采样 + 赋值 + 发布） |
| 次要可能 | PCD/轨迹写入、或下一帧 handleLIO 内体素/点云处理 |

---

## 6. 验证与修复建议

### 6.1 获取精确 backtrace（必做）

GDB 当前只接了 automap_system_node，**无法看到 fast_livo 的崩溃栈**。建议：

- 用 GDB **单独** 启动 fast_livo，复现崩溃后执行 `thread apply all bt full`，保存 backtrace。
- 或使用 launch 将 fast_livo 也通过 `run_under_gdb.sh` 启动，以便崩溃时自动打印栈。

### 6.2 规避验证：关闭或延后 /Laser_map 发布

在 **system_config_M2DGR.yaml**（唯一配置）中修改 fast_livo 发布参数：

- **方案 A**：关闭 Laser_map 发布  
  - 将 `publish.laser_map_interval` 设为 **0**（若代码支持 0 表示不发布）。
- **方案 B**：延后首次发布  
  - 将 `publish.laser_map_interval` 改为 **100** 或更大，使第 10 帧不再进入该分支。

若修改后崩溃消失，可基本确认崩溃在「第 10 帧 /Laser_map 发布」路径（体素滤波或 publish）。

### 6.3 代码层面防护（建议）

- 在 **LIVMapper.cpp** 的 Laser_map 发布分支（约 673–698 行）：
  - 对 `pcl_laser_map_accum_` 做空与大小检查后再做 VoxelGrid 与赋值。
  - 对 `pcl::toROSMsg` 与 `publish` 包在 try/catch 中，避免未捕获异常导致进程直接退出。
- 若 backtrace 指向 PCL 内部（如 VoxelGrid），可考虑：
  - 限制单次累积点数再下采样，或
  - 更换下采样方式/参数，或升级/替换 PCL 版本并回归。

### 6.4 运行与验证清单

1. **确认配置**：启动日志中仅出现 `system_config_M2DGR.yaml`，且无其他 config 覆盖。
2. **复现**：同一 bag + 同一配置复现崩溃，记录 exit code -11 与「published #10 done」顺序。
3. **规避验证**：改 `laser_map_interval` 后不再崩溃 → 支持「Laser_map 路径」为根因。
4. **backtrace**：GDB 挂载 fast_livo 复现，根据栈再精确定位到具体函数/行（PCL/ROS2 或本工程）。

---

## 7. 风险与回滚

- **仅改配置**：改 `laser_map_interval` 为 0 或大值，不影响建图主流程（仅少发 /Laser_map 或延后），可随时回滚为 10。
- **改代码**：建议在独立分支或 PR 中做 try/catch 与空/大小检查，通过 CI 与本地回放再合并。

---

## 8. 后续演进（可选）

- **MVP**：用 GDB 拿到 backtrace + 配置规避确认根因。
- **V1**：在 Laser_map 路径加防护与日志，并做压力测试（多段 bag、不同 laser_map_interval）。
- **V2**：若确认为 PCL 问题，评估升级 PCL 或替换体素下采样实现，并统一回归 M2DGR 与其它数据集。

---

## 附录：关键配置与代码位置

- **唯一配置**：`automap_pro/config/system_config_M2DGR.yaml`
- **fast_livo 发布**：`publish.laser_map_interval`（默认 10）、`publish.laser_map_voxel_size`（0.2）
- **代码**：`automap_pro/src/modular/fast-livo2-humble/src/LIVMapper.cpp`  
  - 发布计数与 "published #N done"：约 1446–1472 行  
  - Laser_map 累积与发布：约 671–698 行  
  - PCD/轨迹写：约 1505–1564、1566–1592 行
