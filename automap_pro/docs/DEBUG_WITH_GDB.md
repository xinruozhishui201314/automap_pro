# 使用 GDB 精准定位崩溃（SIGSEGV）

当 `automap_system_node` 以 exit code -11 (SIGSEGV) 崩溃时，可用 GDB 获取**完整调用栈**，精确定位崩溃代码行。

---

## 方式一：Launch 内 GDB 包装（推荐，崩溃即打 backtrace）

让 `automap_system_node` 在 GDB 下启动，崩溃时 GDB 自动执行 `bt full` 并退出，backtrace 直接打在终端。

### 1. 确保容器内已安装 GDB，且 launch 目录有 run_under_gdb.sh

- **安装 GDB**（若镜像内无）：  
  `docker run -it --rm -v /path/to/automap_ws:/root/automap_ws:rw automap-env:humble bash`  
  内执行：`apt-get update && apt-get install -y gdb`
- **包装脚本**：launch 通过 `automap_pro/launch/run_under_gdb.sh` 调用 GDB；若日志出现「未找到 run_under_gdb.sh」，请确认该脚本存在且可执行：`chmod +x automap_pro/launch/run_under_gdb.sh`

### 2. 离线模式 + GDB 启动

```bash
bash run_automap.sh --offline \
  --bag-file "$(pwd)/data/automap_input/M2DGR/street_03_ros2" \
  --config system_config_M2DGR.yaml \
  --gdb
```

或直接传 launch 参数（容器内）：

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch automap_pro automap_offline.launch.py \
  config:=/root/automap_ws/src/automap_pro/config/system_config_M2DGR.yaml \
  bag_file:=/data/automap_input/M2DGR/street_03_ros2 \
  run_automap_under_gdb:=true
```

### 3. 查看输出

崩溃发生时，终端会先出现 GDB 的 `Program received signal SIGSEGV`，紧接着是 **backtrace**，例如：

```
Thread 1 "automap_syst" received signal SIGSEGV, Segmentation fault.
0x00007f8b... in pcl::VoxelGrid<...>::applyFilter(...) at ...
#0  0x... in ...
#1  0x... in automap_pro::utils::voxelDownsample(...) at .../utils.cpp:93
#2  0x... in automap_pro::utils::voxelDownsampleChunked(...) at .../utils.cpp:161
...
```

根据 **#0 #1 #2 ...** 最上层几帧即可定位到源文件与行号。

---

## 方式二：Core Dump + 事后分析

不改 launch，正常跑；崩溃后用生成的 core 文件在 GDB 里看 backtrace。

### 1. 开启 core dump

在**容器内**执行 launch 前：

```bash
ulimit -c unlimited
# 可选：把 core 写到挂载目录，便于拷到宿主机
export core_pattern_orig=$(cat /proc/sys/kernel/core_pattern 2>/dev/null)
# 若容器有权限可设（多数容器不可写）：
# echo '/data/automap_output/core.%e.%p' | sudo tee /proc/sys/kernel/core_pattern
```

然后正常启动（例如 `ros2 launch ...` 或 `bash run_automap.sh --offline ...`）。崩溃后 core 通常在**当前工作目录**（如 `/root/automap_ws`），文件名多为 `core` 或 `core.<pid>`。

### 2. 用 GDB 读 core 并打 backtrace

在容器内（或把可执行文件和 core 拷到宿主机后）：

```bash
cd /root/automap_ws
gdb -batch -ex "bt full" \
  install/automap_pro/lib/automap_pro/automap_system_node \
  ./core
```

若 core 在挂载目录（如 `/data/automap_output`）：

```bash
gdb -batch -ex "bt full" \
  /root/automap_ws/install/automap_pro/lib/automap_pro/automap_system_node \
  /data/automap_output/core.12345
```

输出格式与方式一相同，根据最顶几帧定位源码与行号。

### 3. 宿主机分析（可选）

若在宿主机用 GDB 分析，需保证：

- 可执行文件路径与容器内一致（或使用 `set sysroot` / 相同编译产物）；
- 有对应带符号的二进制（Debug 或 RelWithDebInfo 更佳）。

---

## 方式三：交互式 GDB（可单步、看变量）

需要单步、查看变量时，可单独用 GDB 启动 `automap_system_node`（其他节点和 bag 仍由 launch 启动）。

### 1. 先启动除 automap 外的节点 + bag

可临时改 launch 去掉 automap 节点，或在一台终端正常 `ros2 launch ...`，在另一台用 GDB 启动 automap（需相同 ROS_DOMAIN_ID 等环境）。更简单的方式是：用 `run_automap_under_gdb:=true` 时 GDB 已是“包装”启动，若需交互，可改为不用 launch 的 prefix，手动执行：

```bash
# 终端 1：先启动 fast_livo + rviz + bag（或修改 launch 只起这些）
# 终端 2：在容器内
source /opt/ros/humble/setup.bash && source install/setup.bash
gdb --args install/automap_pro/lib/automap_pro/automap_system_node \
  --ros-args -r __node:=automap_system -p config_file:=/root/automap_ws/src/automap_pro/config/system_config_M2DGR.yaml -p use_sim_time:=true
(gdb) run
# 崩溃后
(gdb) bt full
(gdb) info locals
(gdb) frame 2
(gdb) list
```

---

## 常见崩溃位置与对应代码

| backtrace 中出现 | 可能位置 |
|------------------|----------|
| `voxelDownsampleChunked` / `voxelDownsample` | `automap_pro/src/core/utils.cpp` 体素滤波或点云遍历 |
| `buildGlobalMap` | `automap_pro/src/submap/submap_manager.cpp` 合并/体素 |
| `publishGlobalMap` | `automap_pro/src/system/automap_system.cpp` 发布全局地图 |
| `pcl::VoxelGrid::applyFilter` | PCL 内部；多为点云过大或 leaf 过小导致索引溢出，可加大 leaf 或限制点数 |

---

## 编译类型与符号

- **Release**：有行号、少优化，backtrace 通常足够定位文件与函数，偶有内联。
- **Debug**：`colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug` 可获完整符号与行号，便于 `list`、`info locals`。

---

## 小结

| 方式 | 优点 | 适用场景 |
|------|------|----------|
| **--gdb / run_automap_under_gdb:=true** | 不改流程，崩溃即打 bt | 快速拿 backtrace |
| **Core dump + gdb -batch -ex "bt full"** | 不拖慢运行，可重复分析 | 无法现场看终端时 |
| **交互式 GDB** | 可单步、看变量 | 需要深入排查逻辑时 |

推荐优先使用 **方式一**（`--gdb`），崩溃后直接根据 backtrace 对应到源码行即可精准修 bug。
