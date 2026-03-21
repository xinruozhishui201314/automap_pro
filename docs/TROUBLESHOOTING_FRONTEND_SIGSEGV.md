# 前端 `fastlivo_mapping` SIGSEGV（exit -11）排查

## 症状

- 日志末尾出现 `[fast_livo][SIGSEGV]`，栈在 `free` / `shared_ptr` 析构路径；进程 `exit code -11`。
- 常发生在 **建图结束 / SIGINT / 保存 PCD**（如 `all_raw_points.pcd`）之后，属于前端进程独立生命周期内的 **析构或重复保存** 问题（use-after-free / double-free 类），**不是** `automap_system` 后端 GDB 包装能直接捕获的。

## 推荐复现（宿主机）

使用一键脚本对 **前端节点** 启用 GDB（与仅包装 `automap_system` 的 `--gdb` 不同）：

```bash
bash run_automap.sh --offline --bag-file <path> --config system_config_M2DGR.yaml \
  --gdb-frontend --clean --log-dir <path>
```

- `--gdb`：仅对 **automap_system** 节点包装 GDB。
- `--gdb-frontend`：对 **fastlivo_mapping**（Fast-LIVO2 前端）包装 GDB，用于定位前端 SIGSEGV。

## 修复方向（独立包）

根因修复应在 **fast_livo / Fast-LIVO2** 包内：保存 PCD、shutdown、重复触发保存等路径上避免 **double-save**、**重复释放** 或 **析构顺序** 问题。本仓库 `automap_pro` 侧以 **可观测性**（`--gdb-frontend`）与运行文档为主。

## 相关日志

```bash
grep -E 'fast_livo\]\[SIGSEGV|exit code -11' full.log
```
