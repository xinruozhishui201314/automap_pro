# 离线运行异常分析与日志强化说明

## Executive Summary

- **现象**：`run_automap.sh --offline --bag-file .../street_03_ros2 --config system_config_M2DGR.yaml` 运行时，`ros2 bag play` 报 `yaml-cpp: error at line 25, column 29: bad conversion` 退出，随后 HBA 因 `pose_vec.size()=0 < WIN_SIZE(10)` 崩溃。
- **根因链**：metadata.yaml 解析失败 → bag 未播放 → 无传感器数据 → fast_livo 无输出 → pose.json 为空/未生成 → HBA 读到位姿数为 0 后抛错。
- **本次变更**：在宿主机对 metadata 做预检查与自动修复尝试、强化 [BAG]/[HBA] 日志、launch 打印 bag 路径与依赖提示、HBA 规范化 data_path 并输出 pose 文件存在性/大小，便于快速定位同类问题。

---

## 1. 根因分析

| 环节 | 现象 | 原因 |
|------|------|------|
| ros2 bag play | `Exception on parsing info file: yaml-cpp: error at line 25, column 29: bad conversion` | ROS2 Humble 的 rosbag2 用 yaml-cpp 解析 metadata.yaml 时，对 `offered_qos_profiles: []` 等格式期望与 rosbags 生成的不一致，导致转换失败；且 metadata 若由容器 root 创建则宿主机可能不可写，修复脚本无法改写。 |
| fast_livo | 无报错但无位姿输出 | bag 未播放，无 lidar/imu/camera 数据，前端无输入。 |
| HBA | `lio_pose_orig.size()=0`，随后 `pose_vec.size()=0 < WIN_SIZE(10)` 抛错 | 从 `data_path + "pose.json"` 读位姿；config 中 `data_path: "/tmp/hba_data"` 无尾部 `/` 时拼接成错误路径 `/tmp/hba_datapose.json`，且即使用正确路径，bag 未播放时 pose.json 也为空。 |

---

## 2. 变更清单（文件/模块）

| 文件 | 变更要点 |
|------|----------|
| `run_automap.sh` | 宿主机对 metadata 做 chmod 尝试、修复脚本执行、**[BAG] 预检查**（Python 解析 metadata），并输出 bag_dir/metadata 路径与可写性；预检查失败时打印解析异常并给出修复命令。 |
| `automap_pro/launch/automap_offline.launch.py` | 启动时 **打印 [BAG] 离线回放 bag_file=** 及“若 bad conversion 将无数据、HBA 依赖 pose.json”的提示。 |
| `automap_pro/src/modular/HBA-main/HBA_ROS2/src/hba.cpp` | **data_path 尾部补 `/`**，统一 pose 路径；**[HBA] [DATA]** 输出 pose 文件完整路径、位姿数量；当 size=0 时额外输出 **pose 文件存在与否与大小（字节）**；**[HBA] [FATAL]** 中增加 pose_path 并提示先确认 ros2 bag play 无 bad conversion。 |
| `scripts/fix_ros2_bag_metadata.py` | 读取/写入失败时 **stderr 输出**明确信息；无写权限时提示 `chmod u+w` 并以 exit 3 退出。 |

---

## 3. 编译与验证

- **编译**：仅 HBA 与 launch/脚本相关，无需改 CMake；若只改脚本/launch 可不重编，改 HBA 后需重编 `hba` 包。
  ```bash
  cd automap_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select hba
  ```
- **验证建议**：
  1. **修复当前 bag**（宿主机，解决 bad conversion）：
     ```bash
     chmod u+w data/automap_input/M2DGR/street_03_ros2/metadata.yaml
     python3 scripts/fix_ros2_bag_metadata.py data/automap_input/M2DGR/street_03_ros2
     ```
  2. 再次运行离线命令，确认：
     - 启动时出现 `[BAG] bag_dir=... metadata=... writable=...`、`[BAG] [PRECHECK] metadata.yaml 解析通过`（或预检查失败时有解析异常详情）；
     - launch 输出 `[automap_offline] [BAG] 离线回放 bag_file=...`；
     - 若 bag 仍失败，HBA 日志中有 `[HBA] [DATA] 读取 LIO 位姿: /tmp/hba_data/pose.json`、`存在=no` 或 `大小=0` 及 [FATAL] 中的 pose_path。

---

## 4. 风险与回滚

- **风险**：宿主机 chmod 可能在某些多用户/只读挂载场景不适用，仅做尝试并打日志。
- **回滚**：用 git 还原上述 4 个文件即可；HBA 的 data_path 尾部 `/` 为兼容性修复，建议保留。

---

## 5. 后续可选

- 在 launch 中在 `ros2 bag play` 前增加“仅解析 metadata 不播放”的预检查（容器内），失败时提前退出并打 [BAG] 日志。
- 将 [BAG] [PRECHECK] 失败时建议的修复命令写入 run_automap.sh 退出码说明或 Runbook。
