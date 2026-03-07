# AutoMap-Pro 系统启动故障排查指南

**版本**: 2.0 | **更新**: 2026-03-05

---

## 级联故障链识别

您遇到的问题是**三层级联故障**：

```
┌─────────────────────────────────────────────────────────────┐
│ 【第 1 层】ROS2 Bag 元数据异常                              │
│ ├─ 错误: yaml-cpp: error at line 25, column 29: bad conversion│
│ ├─ 根因: offered_qos_profiles: []                            │
│ └─ 影响: ros2 bag play 崩溃，无消息发布                      │
├─────────────────────────────────────────────────────────────┤
│ 【第 2 层】Fast-LIVO 无输入数据                              │
│ ├─ 原因: Bag 回放失败 → LIO 无话题数据                       │
│ ├─ 症状: fast_livo 无法处理点云/IMU                          │
│ └─ 结果: pose.json 为空（0 字节）                             │
├─────────────────────────────────────────────────────────────┤
│ 【第 3 层】HBA 初始化失败（已通过默认关闭 standalone 节点缓解）│
│ ├─ 错误: std::runtime_error("zero poses...") / exit code -6   │
│ ├─ 原因: 离线同时启动时 pose.json 尚未由 fast_livo/automap 写入 │
│ └─ 修复: 离线 launch 默认 use_hba=false，优化由 HBAOptimizer 负责 │
└─────────────────────────────────────────────────────────────┘
```

---

## 快速修复（三步走）

### 步骤 1：修复 Metadata（解决第 1 层）

```bash
# 1a. 修复权限（如 metadata.yaml 由容器创建，宿主机无法写入）
sudo chmod u+w data/automap_input/M2DGR/street_03_ros2/metadata.yaml

# 1b. 自动修复 offered_qos_profiles 问题
python3 scripts/fix_ros2_bag_metadata.py data/automap_input/M2DGR/street_03_ros2

# 1c. 验证修复（用 Python YAML 检查）
python3 -c "import yaml; yaml.safe_load(open('data/automap_input/M2DGR/street_03_ros2/metadata.yaml'))"
# 无异常输出 = 成功 ✓
```

**判定成功**：
- 命令无异常
- 或脚本输出 "已修复: ..." 或 "无需修改"
- 重新运行 `python3 scripts/diagnose_and_fix_bag.py` 返回 exit code 0

### 步骤 2：降低 Bag 回放速率（解决第 2 层延迟问题）

```bash
# 使用 --bag-file 时添加 --bag-rate 参数，让系统有足够时间处理数据
bash run_automap.sh --offline \
  --bag-file data/automap_input/M2DGR/street_03_ros2 \
  --config system_config_M2DGR.yaml \
  --bag-rate 0.5   # 以 50% 速度回放（默认 100%）
```

**为什么**：
- fast_livo 处理激光点云需要时间
- 若回放速度过快，系统可能丢包或处理不及
- 降速给了 fast_livo 充分的 CPU 时间输出位姿

### 步骤 3：增加日志记录并启动（诊断第 3 层）

```bash
# 启用详细日志模式，便于看到 HBA 初始化过程
bash run_automap.sh --offline \
  --bag-file data/automap_input/M2DGR/street_03_ros2 \
  --config system_config_M2DGR.yaml \
  --verbose  # 如果脚本支持（默认已启用详细诊断）

# 后续查看完整日志
tail -f /tmp/automap_logs/*.log
```

---

## 诊断工具使用

### 1️⃣ 快速诊断 Bag

```bash
python3 scripts/diagnose_and_fix_bag.py <bag_dir> [--verbose] [--fix] [--list-topics] [--json]

# 示例
python3 scripts/diagnose_and_fix_bag.py data/automap_input/M2DGR/street_03_ros2 \
  --verbose \
  --list-topics

# 输出
# ✓ 存储格式: sqlite3
# ✓ Topic 列表: /velodyne_points (3535), /camera/imu (68389), ...
# ✓ Bag 可用，metadata.yaml 通过验证
```

### 2️⃣ 综合故障排查

```bash
bash scripts/diagnose_automap.sh [--log-dir /tmp/automap_logs] [--verbose]

# 检查项：
# - Bag 结构与 metadata
# - 运行日志中的错误
# - Docker 镜像与工作空间
# - 常见问题清单
```

### 3️⃣ 一键修复与启动

```bash
bash scripts/quick_fix_and_run.sh --offline \
  --bag-file data/automap_input/M2DGR/street_03_ros2 \
  --config system_config_M2DGR.yaml

# 此脚本会：
# 1. 自动检测 metadata 问题
# 2. 尝试修复权限与格式
# 3. 启动建图系统
```

---

## 详细问题定位

### 问题 A：yaml-cpp bad conversion (line 25, column 29)

**现象**：
```
[ros2-1] Exception on parsing info file: yaml-cpp: error at line 25, column 29: bad conversion
[ERROR] [ros2-1]: process has died [pid 107, exit code 1, ...]
```

**根本原因**：
```yaml
# ❌ 错误格式（行 25 附近）
offered_qos_profiles: []

# ✅ 修复后
offered_qos_profiles: ''
```

**检查与修复**：
```bash
# 检查问题
grep -n "offered_qos_profiles:\s*\[\]" data/automap_input/M2DGR/street_03_ros2/metadata.yaml

# 修复
python3 scripts/fix_ros2_bag_metadata.py data/automap_input/M2DGR/street_03_ros2

# 验证
python3 -c "import yaml; yaml.safe_load(open('data/automap_input/M2DGR/street_03_ros2/metadata.yaml'))"
```

**权限提示**：
- 若修复脚本报 "无写权限"，说明 metadata.yaml 由 Docker root 创建
- 解决方式：
  ```bash
  sudo chmod u+w data/automap_input/M2DGR/street_03_ros2/metadata.yaml
  python3 scripts/fix_ros2_bag_metadata.py data/automap_input/M2DGR/street_03_ros2
  ```

---

### 问题 B：pose.json 为空（LIO 位姿不输出）

**现象**：
```
[HBA] [DATA] lio_pose_orig.size()=0
[HBA] [DATA] pose 文件 存在=yes 大小=0 字节
[HBA] [FATAL] pose_vec.size()=0 < WIN_SIZE(10)
```

**根本原因**：fast_livo 无法处理 Bag 数据（通常因第 1 层故障）

**诊断步骤**：

```
1️⃣ 检查 ros2 bag play 是否成功
   ├─ 搜索日志中 "[ros2-1]" 的内容
   ├─ 看是否有 "Exception on parsing info file" (bad conversion)
   └─ 若有 → 返回问题 A，修复 metadata

2️⃣ 检查 fast_livo 是否正常启动
   ├─ 搜索 "[fastlivo_mapping-2]" 日志
   ├─ 看是否有错误或无输出
   ├─ 若有 "parameter_blackboard.model" 输出 ✓
   └─ 若无 → 可能 launch 参数问题

3️⃣ 检查 fast_livo 是否收到话题数据
   ├─ 在运行时另开终端
   ├─ 执行 docker ps（找容器 ID）
   ├─ 执行 docker exec <ID> bash
   ├─ 在容器内运行 ros2 topic echo /velodyne_points (若超时无数据 → bag play 失败)
   └─ Ctrl+C 退出

4️⃣ 检查 pose.json 写入权限
   ├─ ls -la /tmp/hba_data/pose.json (在容器内)
   └─ 若无文件 → fast_livo 未输出

5️⃣ 降速重试（给 fast_livo 更多时间处理）
   └─ bash run_automap.sh ... --bag-rate 0.5
```

**进阶调试**：

```bash
# 在容器内实时监控 pose.json
docker exec <container_id> bash -c "
  while true; do
    size=\$(stat -c%s /tmp/hba_data/pose.json 2>/dev/null || echo 0)
    echo \"pose.json size: \$size bytes at \$(date)\"
    sleep 1
  done
"

# 或者在容器启动后的 30 秒内手动查看
docker exec <container_id> tail -f /root/.ros/log/*/*/automap_system.log
```

---

### 问题 B2：AutoMapSystem 显示 `recv: odom=0 cloud=0`（M2DGR / LIVO 相机类型不匹配）

**现象**：
```
[AutoMapSystem][DATA_FLOW] recv: odom=0 cloud=0 last_ts=0.0 | kf=0 sm=0 | ...
```
同时 fast_livo 有 “get imu at time … imu size N”，但 LivoBridge 从未收到 odom/cloud。

**根本原因**：
- Fast-LIVO 在 **LIVO 模式**（`img_en=1`）下需要**图像 + 雷达 + IMU** 同步后才发布 odom/cloud。
- M2DGR bag 中相机话题为 **CompressedImage**（如 `/camera/head/image_raw/compressed`），而 Fast-LIVO 订阅的是 **sensor_msgs/msg/Image**，类型不一致导致图像回调从未触发 → `img_buffer` 始终为空 → `sync_packages()` 一直返回 false → `handleLIO`/`handleVIO` 不执行 → 无 odom/cloud 发布。

**修复**（任选其一）：
1. **推荐（无需改代码）**：在 M2DGR 配置中关闭图像，使用 **LIO-only**：
   - 在 `config/system_config_M2DGR.yaml` 的 `fast_livo.common` 中设置 `img_en: 0`（当前默认已改为 0）。
   - 重启离线回放后，Fast-LIVO 以 ONLY_LIO 模式运行，仅需雷达+IMU 即可输出 odom/cloud。
2. 若必须用 LIVO（相机+雷达融合）：先用 `image_transport` 或 republish 节点将 `/camera/head/image_raw/compressed` 转为 `sensor_msgs/msg/Image` 再播放，或改用未压缩图像话题并配置 `img_topic` 指向该话题。

**说明**：`frontend.odom_topic` / `frontend.cloud_topic` 与 Fast-LIVO 发布的话题名一致（`/aft_mapped_to_init`、`/cloud_registered`），无需修改。Fast-LIVO 当前不发布 `keyframe_info`，LivoBridge 未收到 kfinfo 为预期行为，不影响 odom/cloud 接收。

---

### 问题 B3：Fast-LIVO 刷屏 `IMU and LiDAR not synced! delta time: 0.5~0.6`

**现象**：
```
[laserMapping]: IMU and LiDAR not synced! delta time: 0.616468 .
[laserMapping]: last_timestamp_lidar: 1628249941.730370, timestamp: 1628249941.113902
```
即 LiDAR 时间戳领先 IMU 约 0.5s 以上，告警持续刷屏。

**原因**：
1. **Bag 回放过快**：若 `rate` > 1.0，数据按时间戳间隔被压缩播放，LiDAR 帧先到、IMU 尚未“跟上”，触发同步检查。
2. **数据集时间基准不一致**：录制时 LiDAR 与 IMU 使用不同时钟（如 LiDAR 为扫描结束时间、IMU 为采样时间），导致同一时刻两者 header.stamp 相差约 0.5s。

**处理（按顺序尝试）**：
1. **按传感器正常频率回放**：确保使用 `--rate 1.0`（实时）或 `--bag-rate 0.5`（半速），避免“读得太快”。
   - 示例：`run_automap.sh --offline --bag-file <path> --config system_config_M2DGR.yaml --bag-rate 1.0`
2. **对齐 LiDAR/IMU 时间（推荐用于 M2DGR 等已知不同步数据集）**：在对应 `system_config_*.yaml` 的 `fast_livo.common` 中设置 `ros_driver_bug_fix: true`，算法会将 IMU 时间对齐到 LiDAR，告警消失且 LIO 正常解算。
3. 若仍异常，检查 bag 内 `/velodyne_points` 与 IMU 话题的 `header.stamp` 是否同源、是否有固定偏移。

---

### 问题 C：RViz 界面无显示（点云/轨迹都不见）

**现象**：RViz 能启动，但 3D 视图里没有点云、路径或任何内容；或状态栏提示 “No transform from [camera_init] to [map]”。

**根本原因**（二者之一或兼有）：
1. **RViz 配置路径错误**：launch 使用 `config/automap.rviz`，实际配置文件在 `rviz/automap.rviz`，install 后仅 rviz 目录包含该文件 → RViz 加载失败或使用空配置。
2. **TF 帧不连通**：RViz Fixed Frame 为 `map`，Fast-LIVO2 发布的 `/cloud_registered`、`/path`、`/aft_mapped_to_init` 的 `frame_id` 为 `camera_init`。若未发布 `map` → `camera_init` 的变换，RViz 无法把数据变换到固定帧 → 不显示。

**修复**（已纳入 launch 的默认行为，若你使用未更新代码可手动对照）：
1. **RViz 配置**：launch 应优先使用 `pkg_share/rviz/automap.rviz`，不存在时再回退到 `config/automap.rviz`。若你自建 launch，请指向 `rviz/automap.rviz`。
2. **TF**：在 world→map 之外，增加 **map → camera_init** 的静态变换（单位变换即可），使 Fast-LIVO 的 `camera_init` 与 `map` 对齐，例如：
   ```bash
   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map camera_init
   ```
   离线/在线 launch 已默认启动 `map_camera_init_tf` 节点，一般无需手输。

**验证**：
- RViz 左侧 Displays 里 “cloud_registered”“path_frontend”“automap_odom_path” 等为 Enabled，Topic 正确。
- 左下角 Fixed Frame 为 `map`；Global Options 无 “No transform from … to map” 报错。
- 若仍无显示，在容器内执行 `ros2 run tf2_ros tf2_echo map camera_init`，应能看到有效变换。

---

### 问题 C2：pcl::VoxelGrid::applyFilter — Leaf size is too small / automap_system 崩溃 (exit code -11)

**现象**：
- 日志出现：`[pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.`
- 随后 `automap_system_node` 可能崩溃 (SIGSEGV, exit code -11)。

**原因**：PCL 的 VoxelGrid 使用 int 作为体素网格索引，当点云范围大且 leaf size 过小时，(max-min)/leaf_size 超过整数范围会导致溢出并可能崩溃。

**已做修复**（源码）：
- **utils**：`voxelDownsample` 在调用 PCL 前做溢出检查。PCL 内部检查 `(dx*dy*dz) > INT_MAX` 会告警并返回原云；单轴过大仍可能导致后续线性索引溢出与 SIGSEGV。因此将单轴上限设为 `kMaxVoxelAxisIndex=1290`（约 INT_MAX 的立方根），保证 `dx*dy*dz <= INT_MAX`，超限时自动放大 leaf 或返回无体素滤波的副本，避免触发 PCL 告警与崩溃。
- **SubMapManager**：子图冻结、合并前后的体素降采样均改为调用 `utils::voxelDownsample`，不再直接使用 `pcl::VoxelGrid`，从而避免未校验的大范围点云触发崩溃。
- **RvizPublisher**：全局地图发布时的二次降采样也统一走 `utils::voxelDownsample`。

若仍出现该警告（例如来自 fast_livo 前端），可在 `system_config` 中适当调大 Fast-LIVO 的体素/降采样相关参数；automap 后端建图与发布路径已防护。

**精准定位崩溃位置**：根据终端最后一条 `[AutoMapSystem][MAP]` / `[Utils] [MAP]` 日志可判断崩溃阶段；小点云（≤25 万点）已自动走单次体素滤波避免分块循环内 SIGSEGV。详见 **[LOGGING_AND_DIAGNOSIS.md](LOGGING_AND_DIAGNOSIS.md)**；配合 `--gdb` 或 core dump 可得到完整 backtrace，见 [DEBUG_WITH_GDB.md](DEBUG_WITH_GDB.md)。

---

### 问题 B3：启动时提示「无法获取 bag 话题列表」

**现象**：
```
[automap_offline] [BAG] 无法获取 bag 话题列表（请确认 bag 路径正确且为 ROS2 格式）
```

**原因**：`ros2 bag info <path>` 在部分环境（如 PATH 无 ros2、超时、或 Humble 输出格式差异）下未返回可解析的话题列表。

**当前行为**：launch 已增加**回退逻辑**：当 `ros2 bag info` 失败时，会尝试读取 bag 目录下的 **metadata.yaml**（rosbag2 标准），从中解析 `topics_with_message_count` 并打印话题列表。只要 bag 目录内存在有效的 `metadata.yaml`，即可正常显示话题。若仍无法获取，请确认：
- `bag_file` 指向的是**目录**（如 `.../street_03_ros2`）或包含 `metadata.yaml` 的父目录；
- 该目录下存在 `metadata.yaml` 且格式正确（可用 `diagnose_and_fix_bag.py` 检查）。

---

### 问题 C：HBA 进程崩溃 (exit code -6)

**现象**：
```
[ERROR] [hba-3]: process has died [pid 111, exit code -6, ...]
[HBA] [FATAL] pose_vec.size()=0 < WIN_SIZE(10)
terminate called after throwing an instance of 'std::runtime_error'
```

**根本原因**：
- 通常因问题 B（pose.json 为空）
- 偶尔因内存不足或编译产物过期

**解决方案**（按优先级）：

1. **修复 Bag metadata**（问题 A）
2. **降速回放**（问题 B 第 5 点）
3. **增加系统资源**：
   ```bash
   # 编辑 run_automap.sh 中的 docker run 参数
   # 添加 --memory 16g --cpus 8
   ```
4. **重新编译**（若问题仍存）：
   ```bash
   bash run_automap.sh --clean --offline --bag-file <path> --config <yaml>
   ```

**新增的强化日志**：

从本版本起，HBA 初始化输出已升级，会逐步打印：
```
[HBA] [INIT] Step 1: 检查 LIO 位姿文件: ...
[HBA] [INIT] Step 2: 读取 LIO 位姿数据...
[HBA] [INIT] Step 3: GPS 融合配置
[HBA] [INIT] Step 4: 位姿充分性检查
[HBA] [WARN] 位姿不足警告 (若 < WIN_SIZE)
```

**查看这些日志可快速定位问题**。

---

## 预防性措施

### 1️⃣ 录制 Bag 时的最佳实践

```bash
# 使用 MCAP 格式而非 SQLite3（更兼容）
ros2 bag record -o record.mcap \
  --storage-preset mcap \
  /velodyne_points /camera/imu /ublox/fix ...
```

### 2️⃣ 定期检查 Bag 完整性

```bash
python3 scripts/diagnose_and_fix_bag.py <bag_dir> --list-topics --verbose

# 验证：
# - 所有必要 topic 都存在
# - 消息数量合理（不为 0）
# - metadata 可解析
```

### 3️⃣ 配置与 Bag 的一致性检查

在 `system_config_M2DGR.yaml` 中，确保：
```yaml
frontend:
  fast_livo2:
    subscribe_topics:
      lidar: "/velodyne_points"        # ← 与 bag 中的 topic 一致
      imu: "/dvs/imu"                   # ← 检查 bag 是否包含此 topic
      camera: "/camera/color/image_raw/compressed"
```

通过以下命令快速验证：
```bash
python3 scripts/diagnose_and_fix_bag.py <bag_dir> --list-topics | grep -E "velodyne|imu|image"
```

---

## 性能优化建议

### 当前瓶颈

| 组件 | 耗时 | 瓶颈 |
|------|------|------|
| Bag 回放 | 35秒 | 磁盘 I/O（SQLite3） |
| Fast-LIVO2 | 50秒 | CPU（点云处理） |
| HBA 优化 | 20秒 | 多层优化 |
| **总耗时** | **~105秒** | **实时处理困难** |

### 优化方案

1. **使用 MCAP 格式**：加快 Bag 读取 → `-20% 耗时`
2. **配置线程数**：
   ```yaml
   backend:
     hba:
       thread_num: 4  # 根据 CPU 核心数调整
   ```
3. **启用 CUDA**（若可用）：
   ```yaml
   frontend:
     fast_livo2:
       use_cuda: true
   ```

---

## 参考：完整启动命令

```bash
# 【推荐】使用一键脚本（自动修复 + 诊断）
bash scripts/quick_fix_and_run.sh --offline \
  --bag-file data/automap_input/M2DGR/street_03_ros2 \
  --config system_config_M2DGR.yaml

# 【备选】手动启动（完整控制）
# 1. 修复 metadata
python3 scripts/fix_ros2_bag_metadata.py data/automap_input/M2DGR/street_03_ros2

# 2. 启动建图
bash run_automap.sh --offline \
  --bag-file data/automap_input/M2DGR/street_03_ros2 \
  --config system_config_M2DGR.yaml

# 【调试】启用详细日志
bash run_automap.sh --offline \
  --bag-file data/automap_input/M2DGR/street_03_ros2 \
  --config system_config_M2DGR.yaml \
  --no-rviz  # 关闭 RViz 以减少资源占用
```

---

## 故障排查快速清单

- [ ] Bag 目录存在且包含 `metadata.yaml` 和 `*.db3` 或 `*.mcap`
- [ ] `metadata.yaml` 权限可写（或已用 sudo chmod 修复）
- [ ] `offered_qos_profiles: []` 已修复为 `offered_qos_profiles: ''`
- [ ] Python YAML 可正常解析 metadata
- [ ] Fast-LIVO 启动日志无错误
- [ ] 系统资源充足（检查 docker stats）
- [ ] Config 与 Bag topic 一致
- [ ] 若 pose.json 仍为空，尝试 `--bag-rate 0.5` 降速

---

## 联系与反馈

遇到问题？
1. 先运行 `bash scripts/diagnose_automap.sh`
2. 收集完整日志：`tar -czf automap_logs.tar.gz /tmp/automap_logs`
3. 检查本文档的诊断部分
4. 查看本项目 Issues 或提交新 Issue

---

**最后更新**: 2026-03-05  
**维护者**: AutoMap-Pro Team
