# Fast-LIVO SIGSEGV 修复说明

## 1. Executive Summary

- **现象**：`fastlivo_mapping` 在运行约第 10 帧后以 **exit code -11 (SIGSEGV)** 崩溃。
- **根因**：多处未对空容器/空指针做防护，在 `measures.empty()`、`pv_list_` 与 `world_lidar` 大小不一致、或 `feats_undistort` 空时访问导致未定义行为。
- **修复**：在 `LIVMapper.cpp` 中增加防御性检查与空指针优先判断，避免对空容器 `.back()` 和越界访问。

## 2. 修改清单（LIVMapper.cpp）

| 位置 | 问题 | 修复 |
|------|------|------|
| `handleLIO()` 入口 | `feats_undistort->empty()` 若指针为 null 会先解引用崩溃 | 改为 `(feats_undistort == nullptr) \|\| feats_undistort->empty()` |
| `handleVIO()` 入口 | 同上，`pcl_w_wait_pub` | 改为 `(pcl_w_wait_pub == nullptr) \|\| pcl_w_wait_pub->empty()` |
| `handleLIO()` 体素更新循环 | `pv_list_[i]` / `cross_mat_list_[i]` 与 `world_lidar->points.size()` 可能不一致导致越界 | 循环前检查 `pv_list_.size() == world_lidar->points.size()` 及 cross/body_cov 长度，不匹配则打 ERROR 并跳过循环与 UpdateVoxelMap |
| 发布 block `frame_ts` | `LidarMeasures.measures.back()` 在 measures 为空时未定义 | 先判断 `!measures.empty()`，否则用 `last_lio_update_time` 或 0.0 |
| save map 块 `update_time` | 同上 | 先判断 `!measures.empty()`，否则用 `last_lio_update_time` 并打 THROTTLE 告警 |
| PCD save case 1 (body frame) | 使用 `feats_undistort` 未判空 | 条件增加 `feats_undistort != nullptr && !feats_undistort->empty()` |
| `fout_lidar_pos` / `fout_visual_pos` | 写文件时使用 `measures.back()` | 仅在 `!measures.empty()` 时写入 |

## 3. 编译与验证

```bash
# 在 automap_ws 下重新编译 fast_livo（LIVMapper 属于该包）
cd /path/to/automap_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_livo
source install/setup.bash

# 复现原场景（离线 M2DGR）
bash run_automap.sh --offline \
  --bag-file "$(pwd)/data/automap_input/M2DGR/street_03_ros2" \
  --config system_config_M2DGR.yaml
```

- **预期**：`fastlivo_mapping` 不再因 SIGSEGV 退出；若出现 `pv_list_/world_lidar size mismatch`，说明曾存在潜在越界，现已避免崩溃并打日志便于后续排查。

## 4. 若仍崩溃

若修复后仍出现 -11，请用 GDB 抓 backtrace：

```bash
# 用 GDB 启动 fastlivo_mapping（需在 launch 里对该节点用 run_under_gdb 或 gdb -ex run --args）
gdb -ex run --args /path/to/install/fast_livo/lib/fast_livo/fastlivo_mapping --ros-args ...
# 崩溃后在 GDB 中执行
(gdb) bt full
```

将 `bt full` 输出与日志一并保留，便于定位到具体行号与调用栈。

## 5. 风险与回滚

- **风险**：在 size mismatch 时跳过体素更新，该帧地图不更新，可能轻微影响局部一致性；仅当 StateEstimation 与 world_lidar 长度本当一致却出现不一致时发生。
- **回滚**：用 git 还原 `automap_pro/src/modular/fast-livo2-humble/src/LIVMapper.cpp` 并重新编译 `fast_livo` 即可。
