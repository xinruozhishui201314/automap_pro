# 崩溃分析：fast_livo SIGSEGV 发生在 after_laser_map_pub 之后

## Executive Summary

- **现象**：`fastlivo_mapping` 在第 10 帧打印 `after_laser_map_pub` 后立即 SIGSEGV（exit code -11），未打印下一行 `exit_laser_map_block`。
- **根因定位**：崩溃发生在**激光地图发布 if 块结束**时，局部变量 `PointCloudXYZI::Ptr to_pub` 析构 → `shared_ptr::_M_release` → 析构 PointCloud → `free()` 时崩溃，属 **double free 或 invalid free**。
- **触发条件**：`laser_map_pub_interval_ > 0` 且 `(frame_num+1) % interval == 0`（如第 10 帧）、且进入 voxel 下采样分支且 `!to_pub->empty()` 时，执行了 `*pcl_laser_map_accum_ = *to_pub` 后，块结束时析构 `to_pub` 触发崩溃。
- **修复策略**：在复制到 `pcl_laser_map_accum_` 后对 `to_pub` 做 `clear()`，使析构时只释放空点云，避免释放可能被 PCL 内部共享或已失效的缓冲区。

---

## 1. 日志与时间线

### 1.1 关键日志（logs/full.log / automap.log）

```text
[fast_livo][LIO][TRACE] frame=10 step=before_laser_map_pub accum_pts=159412
[fast_livo][LIO][TRACE] frame=10 step=after_laser_map_pub
[fast_livo][SIGSEGV] process received SIGSEGV. Last LIO step in log = last '[fast_livo][LIO][TRACE] frame=N step=...' line.
[fast_livo][SIGSEGV] backtrace (best-effort, 11 frames):
  #0  fastlivo_mapping(+0x34743)
  #1  libc(+0x42520)
  #2  libc.so.6(free+0x1e)
  #3  fastlivo_mapping(+0x66800)
  #4  _Sp_counted_base::_M_release
  #5  fastlivo_mapping(+0x582b2)
  ...
[ERROR] [fastlivo_mapping-2]: process has died [pid 138, exit code -11, ...]
```

- 最后一条 TRACE 为 **after_laser_map_pub**，下一条应为 **exit_laser_map_block** 但未出现 → 崩溃发生在 718 行与 720 行之间。
- 该区间**唯一可能触发 free 的路径**是：`if (laser_map_pub_interval_ > 0 && ...)` 块结束，局部变量 `to_pub` 析构。

### 1.2 时间线小结

| 步骤 | 代码位置 | 日志 step | 说明 |
|------|----------|-----------|------|
| 1 | 691 | 进入 if (laser_map_interval) | 第 10 帧、accum 非空 |
| 2 | 692 | before_laser_map_pub | accum_pts=159412 |
| 3 | 693–716 | - | 创建 to_pub、voxel 滤波、赋值、toROSMsg、publish |
| 4 | 718 | after_laser_map_pub | 已打印 |
| 5 | 717 块结束 | （无） | **to_pub 析构 → shared_ptr 释放 → PointCloud 析构 → free() 崩溃** |
| 6 | 720 | exit_laser_map_block | 未执行 |

---

## 2. 代码路径与根因分析

### 2.1 相关代码（LIVMapper.cpp 689–720）

```cpp
*pcl_laser_map_accum_ += *pcl_w_wait_pub;
if (laser_map_pub_interval_ > 0 && (frame_num + 1) % laser_map_pub_interval_ == 0 && !pcl_laser_map_accum_->empty()) {
  PointCloudXYZI::Ptr to_pub(new PointCloudXYZI());
  if (laser_map_voxel_size_ > 0.0) {
    vg.setInputCloud(pcl_laser_map_accum_);
    vg.filter(*to_pub);
    if (!to_pub->empty()) {
      *pcl_laser_map_accum_ = *to_pub;  // 用下采样结果替换累积
    }
    if (to_pub->empty()) to_pub = pcl_laser_map_accum_;
    ...
  } else {
    to_pub = pcl_laser_map_accum_;
  }
  pcl::toROSMsg(*to_pub, laser_map_msg);
  pubLaserCloudMap->publish(laser_map_msg);
  if (trace_this) RCLCPP_INFO(..., "step=after_laser_map_pub", ...);
}  // <-- 此处 to_pub 析构，backtrace 指向 free + _M_release
if (trace_this) RCLCPP_INFO(..., "step=exit_laser_map_block", ...);
```

### 2.2 崩溃路径归纳

- **backtrace**：`free` ← 某处析构（+0x66800）← `_Sp_counted_base::_M_release`（shared_ptr 析构）。
- **结论**：某 `shared_ptr<PointCloudXYZI>` 析构时，对 PointCloud 内某块内存调用 `free()` 导致 SIGSEGV，即 **double free 或 invalid free**。
- **唯一在该区间析构的 shared_ptr**：局部变量 **to_pub**（块结束）。

因此根因可归纳为：

- 在 **voxel 分支且 `!to_pub->empty()`** 时：
  - 执行了 `*pcl_laser_map_accum_ = *to_pub`（PCL 的 copy/swap 语义可能使内部缓冲区存在共享或特殊释放顺序），
  - 随后 `to_pub` 仍持有原滤波结果的 PointCloud，
- 块结束时 `to_pub` 析构，释放该 PointCloud 的内部缓冲区时，与 PCL 实现或与 `pcl_laser_map_accum_` 的缓冲区产生冲突（例如同一块内存被认为由两处“拥有”），导致 **free 时崩溃**。

### 2.3 为何是第 10 帧

- `laser_map_pub_interval_` 一般为 10；条件 `(frame_num + 1) % laser_map_pub_interval_ == 0` 在 frame_num=9（第 10 帧）为真。
- 前几帧未进入该发布分支，故不会在“块结束析构 to_pub”时崩溃。

---

## 3. 修复方案

### 3.1 思路

- 在 **已用 `*to_pub` 覆盖 `pcl_laser_map_accum_` 且不再需要 to_pub 内容** 的分支中，**先清空 to_pub**，再让块结束析构。
- 这样析构时释放的是**空点云**（无大块堆内存），避免对可能已与 `pcl_laser_map_accum_` 发生共享或已失效的缓冲区做 free，从而消除 double free / invalid free。

### 3.2 具体修改（LIVMapper.cpp）

在 `*pcl_laser_map_accum_ = *to_pub;` 之后、且在该分支内“不再使用 to_pub 内容”的前提下，增加一行：

- `to_pub->clear();`

这样：

- 发布仍用 `*to_pub`（在 clear 之前已完成 toROSMsg 与 publish），逻辑不变。
- 若后面有 `if (to_pub->empty()) to_pub = pcl_laser_map_accum_;`，在 clear 之后 to_pub 为空，会改为指向 pcl_laser_map_accum_，析构时只减引用计数，不释放大块内存，也更安全。

注意：若在 **同一次 if 块内** 在 `*pcl_laser_map_accum_ = *to_pub` 之后还有对 `*to_pub` 的读操作（例如 toROSMsg），则必须在 **toROSMsg / publish 之后** 再 `to_pub->clear()`。当前代码顺序是：赋值 → toROSMsg(*to_pub) → publish → after_laser_map_pub，因此应在 **after_laser_map_pub 之前**（或块内最后、且不再使用 to_pub 内容处）调用 `to_pub->clear()`，即 **在 publish 之后、块结束前** 清空 to_pub。

---

## 4. 验证建议

1. **回放**：同一 bag + 同一 config（M2DGR street_03，laser_map 间隔 10）多次运行，确认不再出现“after_laser_map_pub 后 SIGSEGV”。
2. **GDB**：若仍崩溃，在 718 行后、720 行前设断点，确认崩溃栈是否仍为 to_pub 析构 → free。
3. **日志**：确认能稳定看到 `exit_laser_map_block` 及后续 `before_publish_path` 等 step，且进程不退出。

---

## 5. 与既有文档的关系

- **FAST_LIVO_SIGSEGV_V2_FIX.md** 已修复 4 处 `PointCloudXXX().swap(*ptr)` 导致的悬空指针；本次崩溃发生在**激光地图发布块内 to_pub 析构**，是另一处与点云生命周期 / free 相关的问题，建议与本修复一并保留。

---

## 6. 变更清单

| 文件 | 变更 |
|------|------|
| `automap_pro/src/modular/fast-livo2-humble/src/LIVMapper.cpp` | 在 laser map 发布块内，在 `*pcl_laser_map_accum_ = *to_pub` 且发布完成后、块结束前对 `to_pub` 调用 `clear()`，避免析构时对可能共享/失效的缓冲区做 free。 |
