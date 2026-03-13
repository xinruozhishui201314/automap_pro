# 建图日志验证步骤（Mapping Log Verification）

建图结束后，可用下列命令在 `full.log` 或终端输出中验证配置与回环/HBA 是否按预期生效。对应分析见「建图日志深度分析报告」类文档。

## 1. HBA 配置与执行

- **启动时确认 `enable_gtsam_fallback` 已加载**（配置为 true 时 HBA 才会在无 hba_api 时运行）：
  ```bash
  grep "enable_gtsam_fallback" full.log
  grep "HBA CONFIG" full.log
  ```
  应看到 `backend.hba.enable_gtsam_fallback=true` 或 `[HBA][CONFIG] ... enable_gtsam_fallback=1`。

- **HBA 是否真正执行**（GTSAM fallback 或 hba_api 成功）：
  ```bash
  grep "optimization done success=1" full.log
  grep "HBA done: MME=" full.log
  ```
  至少应出现一条，表示某次 HBA 优化成功。

## 2. 回环约束

- **是否有回环被接受并加入因子图**：
  ```bash
  grep "LOOP_ACCEPTED" full.log
  grep "\[AutoMapSystem\]\[LOOP\] detected" full.log
  ```
  若轨迹有闭环且回环检测正常，应出现若干条；0 条表示本 run 无回环约束加入。

## 3. 流程顺序（可选）

- **确认「先后端 flush 再 HBA」**：
  ```bash
  grep "backend_flushed_before_hba\|ensureBackend_exit\|triggerAsync" full.log
  ```
  顺序应为：`forceUpdate`/commitAndUpdate 完成 → `backend_flushed_before_hba` → `enqueue submaps` / `triggerAsync`。

---

以上 grep 均在 `logs/run_YYYYMMDD_HHMMSS/full.log` 或实时终端输出上执行即可。
