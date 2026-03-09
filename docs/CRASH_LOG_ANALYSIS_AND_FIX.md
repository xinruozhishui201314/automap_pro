# 崩溃日志分析与精准定位指南

## Executive Summary

- **现象**：`logs/automap.log` 与 `logs/full.log` 显示 **fastlivo_mapping** 进程在约第 10 帧时崩溃（exit code -11 = SIGSEGV）。
- **结论**：崩溃发生在 **fast_livo（LIVMapper）** 的 `handleLIO()` 中，最后一条成功日志为 `frame=10 step=after_laser_map_pub`，即崩溃点在 **after_laser_map_pub 与 before_publish_path 之间**（含 `publish_path` / `publish_mavros` 或其后逻辑）。
- **已做强化**：  
  1）在 LIVMapper 中增加细粒度 trace（exit_laser_map_block / after_publish_path / after_frame_num_inc），便于下次复现时精确定位到步骤；  
  2）fast_livo main 中注册 SIGSEGV 处理，崩溃时打印 backtrace 到 stderr；  
  3）launch 支持 `run_fast_livo_under_gdb:=true`（脚本 `--gdb-frontend`），用 GDB 抓取前端崩溃的完整 backtrace。  
- **建议**：用 **--gdb-frontend** 或 **ulimit -c unlimited + core 事后分析** 拿到 backtrace，结合新 trace 确定具体崩溃行并修复。

---

## 1. 日志分析结论

### 1.1 从哪里看崩溃

| 日志文件 | 内容 |
|----------|------|
| `logs/full.log` | 全 launch 合并输出（含 ros2、fast_livo、automap_system、rviz 等），带时间戳 |
| `logs/automap.log` | 通常为 automap 相关或过滤后的总览，与 full.log 末尾一致 |

### 1.2 本次崩溃关键信息

从 `logs/full.log` / `logs/automap.log` 末尾可见：

```text
[fast_livo][LIO][TRACE] frame=10 step=before_laser_map_pub accum_pts=159412
[fast_livo][LIO][TRACE] frame=10 step=after_laser_map_pub
[ERROR] [fastlivo_mapping-2]: process has died [pid 140, exit code -11, cmd '.../fastlivo_mapping ...']
```

- **崩溃进程**：`fastlivo_mapping`（laserMapping），pid 140  
- **退出码 -11**：SIGSEGV（段错误）  
- **最后成功步骤**：`frame=10 step=after_laser_map_pub`  
- **未出现**：`frame=10 step=before_publish_path` → 说明崩溃发生在 **after_laser_map_pub 与 before_publish_path 之间**，即：
  - 刚退出 laser_map 发布逻辑的 `if` 块，或
  - `publish_path(pubPath)`，或
  - `publish_mavros(...)`，或
  - 二者之间的 trace 打印

### 1.3 代码位置对应（LIVMapper.cpp handleLIO）

```text
step=after_laser_map_pub   ← 最后打出的 trace
}  // 结束 laser_map 间隔发布的 if
step=exit_laser_map_block  ← 新增：若看到此项，说明崩溃在 publish_path/publish_mavros
step=before_publish_path
publish_path(pubPath);
step=after_publish_path    ← 新增：若看到此项，说明崩溃在 publish_mavros 或之后
publish_mavros(...);
step=after_publish_mavros (handleLIO done)
frame_num++;
step=after_frame_num_inc   ← 新增
```

通过 **“最后出现的 step”** 即可把崩溃缩小到上述某一段。

---

## 2. 已做的日志与调试强化

### 2.1 细粒度 trace（LIVMapper）

在 `automap_pro/src/modular/fast-livo2-humble/src/LIVMapper.cpp` 的 `handleLIO()` 中新增：

- `step=exit_laser_map_block`：刚退出 laser_map 发布的 `if`，若下一行崩溃则 fault 在 publish_path/publish_mavros。
- `step=after_publish_path`：在 `publish_path(pubPath)` 之后。
- `step=after_frame_num_inc`：在 `frame_num++` 之后（handleLIO 尾部）。

复现后查看 **最后一条** `[fast_livo][LIO][TRACE] frame=N step=...` 即可精确定位到上述步骤之一。

### 2.2 SIGSEGV 时打印 backtrace（fast_livo main）

在 `automap_pro/src/modular/fast-livo2-humble/src/main.cpp` 中：

- 注册了 `SIGSEGV` 信号处理。
- 崩溃时向 stderr 打印：
  - 提示：最后 LIO 步骤在 log 中为最后一条 `[fast_livo][LIO][TRACE] frame=N step=...`
  - 说明：如何开启 coredump 并用 GDB 事后分析
  - （Linux/Apple）**best-effort** 的 `backtrace()` / `backtrace_symbols()` 输出，便于在不跑 GDB 时也能看到调用栈片段。

输出会进入 launch 的合并日志（如 `full.log`），便于与 trace 对照。

### 2.3 使用 GDB 抓取前端崩溃 backtrace

当前 **只有 automap_system_node** 可用 `--gdb` 以 GDB 启动；**fastlivo_mapping 是独立进程**，需单独用 GDB 才能在其崩溃时拿到完整 backtrace。

**方式一：launch 参数（推荐）**

- 离线 launch 已支持 `run_fast_livo_under_gdb:=true`，即用 GDB 包装 **fastlivo_mapping**：
  ```bash
  ros2 launch automap_pro automap_offline.launch.py ... run_fast_livo_under_gdb:=true
  ```
- 一键脚本支持 `--gdb-frontend`：
  ```bash
  bash run_automap.sh --offline --bag-file <path> --config system_config_M2DGR.yaml --gdb-frontend
  ```
- 崩溃时 GDB 会打印 `bt full`，输出在 **full.log**（或当前终端）。

**方式二：coredump 事后分析**

1. 启动前在容器/本机执行：`ulimit -c unlimited`
2. 复现崩溃后，在 core 所在目录（或 `coredumpctl` 指定）：
   ```bash
   gdb -c core.<pid> $(which fastlivo_mapping)
   (gdb) bt full
   ```
3. 将 `bt full` 与 **最后一条** `[fast_livo][LIO][TRACE] frame=N step=...` 结合定位到具体代码行。

---

## 3. 推荐排障流程

1. **重新编译**（确保包含新 trace 与 SIGSEGV 处理）  
   - 至少重新编译 fast_livo 与 automap_pro（若改过 launch/脚本则无需重编代码）。

2. **复现并保留日志**  
   - 使用与之前相同的 bag + 配置（如 M2DGR street_03 + system_config_M2DGR.yaml）。  
   - 建议同时：
     - 使用 `--gdb-frontend` 或 `run_fast_livo_under_gdb:=true` 抓 backtrace，或  
     - 在运行环境执行 `ulimit -c unlimited` 后复现，再对 core 做 `gdb -c core ...`。

3. **从日志定位“最后一步”**  
   - 在 `full.log` 或 `automap.log` 中搜索：  
     `[fast_livo][LIO][TRACE]`  
   - 看 **最后一条** 的 `frame=` 和 `step=`：
     - 若最后是 `after_laser_map_pub` → 崩溃在 exit_laser_map_block 或 publish_path 入口附近。
     - 若最后是 `exit_laser_map_block` → 崩溃在 before_publish_path 或 publish_path 内。
     - 若最后是 `before_publish_path` → 崩溃在 `publish_path(pubPath)` 内。
     - 若最后是 `after_publish_path` → 崩溃在 `publish_mavros` 或之后（含 frame_num++ 等）。

4. **结合 backtrace 定行**  
   - GDB 的 `bt full` 或 SIGSEGV 处理里打印的 backtrace，会给出崩溃栈；  
   - 与上一步的 **step** 对应到 `LIVMapper.cpp` 的 `handleLIO` / `publish_path` / `publish_mavros` 等，即可精确到行。

5. **修复与回归**  
   - 根据栈和 step 修代码（空指针、越界、错误 swap/clear 等），参考已有文档如 `docs/FAST_LIVO_SIGSEGV_V2_FIX.md`。  
   - 再次用同一 bag + 配置跑一遍，确认不再出现 exit code -11，且 trace 能走到 `after_frame_num_inc` 或后续帧。

---

## 4. 编译与运行说明

- **编译**（在容器或本机工作空间）：  
  `colcon build --packages-select fast_livo automap_pro`（或全工作空间 build）。
- **运行**（示例）：  
  ```bash
  bash run_automap.sh --offline \
    --bag-file "$(pwd)/data/automap_input/M2DGR/street_03_ros2" \
    --config system_config_M2DGR.yaml \
    --gdb-frontend
  ```
- **查看日志**：  
  - 宿主机：`logs/full.log`、`logs/automap.log`  
  - 搜索：`[fast_livo][LIO][TRACE]`、`[fast_livo][SIGSEGV]`、`process has died`

---

## 5. 风险与回滚

- **Trace 量**：仅在高帧号或每 N 帧打印（与现有 `trace_this` 一致），对性能影响可忽略。若需关闭，可将 `trace_this` 条件收紧或关闭。  
- **SIGSEGV 处理**：在 handler 中调用 `backtrace_symbols()` 不符合 async-signal-safe，仅作 best-effort 调试用；若出现异常行为可注释掉 handler 内 backtrace 打印，只保留提示与 re-raise。  
- **回滚**：若新 trace 或 GDB 启动导致问题，可去掉新增的 trace 行、恢复 main 不注册 SIGSEGV、launch 不传 `run_fast_livo_under_gdb`，脚本不传 `--gdb-frontend`。

---

## 6. 变更清单（便于 code review）

| 文件 | 变更说明 |
|------|----------|
| `automap_pro/src/modular/fast-livo2-humble/src/LIVMapper.cpp` | handleLIO 中增加 step=exit_laser_map_block, after_publish_path, after_frame_num_inc |
| `automap_pro/src/modular/fast-livo2-humble/src/main.cpp` | 注册 SIGSEGV handler，崩溃时打印说明 + best-effort backtrace |
| `automap_pro/launch/automap_offline.launch.py` | 支持 run_fast_livo_under_gdb，fast_livo 节点可带 GDB prefix |
| `run_automap.sh` | 新增 --gdb-frontend，传递 run_fast_livo_under_gdb:=true |

---

## 7. 后续可选

- 若确定崩溃在 `publish_path` / `publish_mavros` 内，可在这两个函数内部再拆更细的 trace 或对 `path`/`msg_body_pose`/`_state` 做空或合法性检查。  
- 可增加配置项关闭或放宽 LIO trace 频率，便于长期运行与日志体积控制。
