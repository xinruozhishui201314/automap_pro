# Fast-LIVO 第 10 帧崩溃修复说明

## Executive Summary

**现象**：`bash run_automap.sh --offline --bag-file ... --config system_config_M2DGR.yaml --gdb --clean` 运行后，fastlivo_mapping 进程在发布约第 10 帧点云后崩溃（exit code -11，SIGSEGV）。

**根因与措施**：
1. **voxel_map**：`GetUpdatePlane()` 在遍历体素树时未对 `current_octo` / `plane_ptr_` 做空指针检查，在部分边界情况下可能解引用空指针导致 SIGSEGV。
2. **LIVMapper**：`publish_frame_world()` 中 “save map” 与 buffer clear 段未做异常保护，任何异常会直接导致进程退出；并增加对 clear 前的指针检查。
3. **资源/并发**：OpenMP 线程数过多可能加剧内存与调度压力，在 GDB/离线回放场景下更容易触发问题；对 fast_livo 的 OpenMP 线程数做了上限（默认 2）。

**修复状态**：✅ 已实施上述三项修改；需**重新编译 fast_livo** 后验证。

---

## 1. 日志与崩溃特征

从 `logs/full.log` / `logs/automap.log` 可见：

```text
[fast_livo][PUB] about_to_publish #10 ts=1628249878.307 pts=15859 ...
[fast_livo][PUB] published #10 done (publish() took 0.00 ms; ...)
[ERROR] [fastlivo_mapping-2]: process has died [pid 140, exit code -11, ...]
```

- **崩溃进程**：fastlivo_mapping（pid 140），**非** automap_system_node。
- **退出码 -11**：SIGSEGV（段错误）。
- **时间点**：第 10 帧点云发布日志打印完成后、下一帧处理前或同帧后续逻辑中。

---

## 2. 修改清单

| 文件 | 修改内容 |
|------|----------|
| `automap_pro/src/modular/fast-livo2-humble/src/voxel_map.cpp` | `GetUpdatePlane()` 入口增加 `current_octo == nullptr \|\| current_octo->plane_ptr_ == nullptr` 时直接 return，避免解引用空指针。 |
| `automap_pro/src/modular/fast-livo2-humble/src/LIVMapper.cpp` | 在 `publish_frame_world()` 中，将 “save map” 整段（含 pcd_save、fout 写入、img_save、buffer clear）包在 `try { ... } catch (std::exception& e) { RCLCPP_ERROR(...); } catch (...) { ... }` 中；clear 前增加对 `laserCloudWorldRGB` / `pcl_w_wait_pub` 的检查（防御性）。 |
| `automap_pro/src/modular/fast-livo2-humble/CMakeLists.txt` | 引入 `MP_PROC_NUM_MAX = 2`，将 OpenMP 实际使用的线程数上限设为 2（原逻辑在 N>4 时用 4，否则用 N）；编译输出中会提示 “Cores: X (capped at 2 for stability)”。 |

---

## 3. 编译与运行

**必须重新编译 fast_livo**（CMake 宏 `MP_PROC_NUM` 与源码均变更）：

```bash
cd /path/to/automap_ws
# 若使用 colcon
colcon build --packages-select fast_livo
# 若 fast_livo 在 automap_pro 内一起编
colcon build --packages-select automap_pro
source install/setup.bash
```

然后按原命令回放（可不带 --gdb 先验证是否还崩溃）：

```bash
bash run_automap.sh --offline \
  --bag-file "$(pwd)/data/automap_input/M2DGR/street_03_ros2" \
  --config system_config_M2DGR.yaml \
  --clean
```

确认无崩溃后再用 `--gdb` 调试（若仍需 backtrace）。

---

## 4. 诊断日志（精准定位再发崩溃）

已加强日志，便于再次崩溃时快速定位到具体步骤：

| 标签 | 含义 | 节流 |
|------|------|------|
| `[fast_livo][LIO][TRACE] frame=N step=...` | handleLIO 各步骤：before/after_publish_frame_world、before/after_publish_effect_world、before/after_pubVoxelMap、before/after_laser_map_pub、before_publish_path、after_publish_mavros | 前 25 帧 + 每 200 帧 |
| `[fast_livo][DIAG] publish_frame_world enter pub_idx=N mode=... lio_vio_flg=...` | 进入 publish_frame_world，当前模式和 buffer 大小 | 前 25 次 + 每 100 次 |
| `[fast_livo][DIAG] publish_frame_world step=save_map_enter pub_idx=N` | 即将执行 save/clear 段（若下一行未出现 save_map_done 即崩溃在此段） | 同上 |
| `[fast_livo][DIAG] publish_frame_world step=save_map_done pub_idx=N` | save/clear 段正常结束 | 同上 |
| `[fast_livo][DIAG] publish_effect_world enter effect_feat_num=N` | 进入 publish_effect_world | 前 20 次 + 每 200 次 |
| `[fast_livo][VoxelMap][DIAG] pubVoxelMap enter/GetUpdatePlane done/exit` | 体素图发布入口、平面数、出口（stderr） | 前 20 次 + 每 200 次 |
| `[fast_livo][VoxelMap][DIAG] GetUpdatePlane null guard hit=N` | 体素树空指针被防护命中（stderr） | 前 5 次 + 每 500 次 |

**用法**：崩溃后查看 `logs/automap.log` 或 `logs/full.log`，搜 `TRACE` / `DIAG`，**最后一条 `step=` 或 `enter`/`done` 即崩溃前执行到的位置**；若最后是 `save_map_enter` 且无 `save_map_done`，则崩溃在 save/clear 段内。

## 5. 验证建议

1. **无 GDB 连续跑**：同一 bag、同一 config 跑到 bag 结束或至少超过 30 帧，观察是否再出现 `process has died [exit code -11]`。
2. **带 GDB**：若仍崩溃，在 GDB 里对 fastlivo_mapping 做 `run`，崩溃时执行 `thread apply all bt`，把 backtrace 贴到后续分析。
3. **结合 TRACE/DIAG**：用上表对日志做 grep，确定最后一步，再结合 backtrace 精确定位代码行。
4. **放宽线程数（可选）**：若确认与线程数无关且需要更高并行度，可修改 `CMakeLists.txt` 中 `set(MP_PROC_NUM_MAX 2)` 为 4 或注释掉 cap 逻辑后重新编译。

---

## 6. 风险与回滚

- **风险**：OpenMP 线程数限制为 2 可能略降前端 LIO 吞吐，在极高帧率或大点云场景下可再调大或取消上限。
- **回滚**：若问题复现或需恢复原并行度，可：
  - 恢复 `voxel_map.cpp` 中 `GetUpdatePlane()` 去掉空指针检查；
  - 恢复 `LIVMapper.cpp` 中去掉 try-catch 与多余指针检查；
  - 在 `CMakeLists.txt` 中删除 `MP_PROC_NUM_MAX` 及与 cap 相关的 `if(PROC_NUM GREATER MP_PROC_NUM_MAX)` 等逻辑，并重新编译。

---

## 7. 与既有文档的关系

- 本修复在 **FAST_LIVO_SIGSEGV_V2_FIX.md**（`PointCloudXXX().swap(*ptr)` 悬空指针修复）之后实施，与之互补：V2 解决的是 swap 导致的悬空指针，本次解决的是体素空指针与异常/线程数导致的崩溃与稳定性。
