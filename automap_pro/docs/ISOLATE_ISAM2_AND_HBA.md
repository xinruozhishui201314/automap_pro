# 隔离 ISAM2+GPS 与 HBA（双路 GTSAM 崩溃排查）

## 目的

同一进程内 **IncrementalOptimizer（ISAM2）** 与 **HBA GTSAM fallback（LevenbergMarquardtOptimizer）** 共用 libgtsam，若出现 `double free or corruption`，可通过“只跑一路”的配置区分根因：

- 仅跑 **ISAM2 + GPS**：若崩溃消失 → 更可能是双路共用或 HBA 路径导致。
- 仅跑 **HBA 用 GPS、ISAM2 不加 GPS**：若仍崩溃 → 更可能是 HBA/GTSAM 单路内某因子或库 bug。

## 配置方式

### 1. 仅 ISAM2 + GPS（关闭 HBA）

在 `system_config*.yaml` 的 `backend.hba` 下设置：

```yaml
backend:
  hba:
    enabled: false   # 不触发 HBA（周期 / GPS 对齐 / 建图结束均不跑）
```

效果：

- 子图冻结时不再调用 `hba_optimizer_.triggerAsync()`。
- GPS 对齐后仍会调用 `addBatchGPSFactors()`（ISAM2 加 GPS），但 **不会** 触发 HBA。
- 传感器空闲 / `finish_mapping` 时不再执行最终 HBA。

手动触发 HBA 的服务 `TriggerHBA` 仍可用（若需要可单独测 HBA）。

### 2. 仅 HBA 用 GPS（ISAM2 不批量加 GPS）

在 `system_config*.yaml` 的 `gps` 下设置：

```yaml
gps:
  add_constraints_on_align: false   # 对齐后不向 ISAM2 批量添加 GPS 因子
```

效果：

- GPS 对齐后 **不会** 调用 `addBatchGPSFactors()` 向 iSAM2 添加 GPS 因子（ISAM2 仅有 prior + odom，无 GPS）。
- HBA 仍会在周期/对齐/结束建图时触发，且 **带 GPS**（`backend.hba.enable_gps_factor: true` 时）。

适合验证：只有 HBA 使用 GTSAM+GPS 时是否仍崩溃。

### 3. 组合示例

| 目标                     | backend.hba.enabled | gps.add_constraints_on_align |
|--------------------------|---------------------|------------------------------|
| 仅 ISAM2+GPS（关 HBA）   | **false**           | true（默认）                 |
| 仅 HBA 用 GPS            | true（默认）        | **false**                    |
| 双路都跑（默认）         | true                | true                         |

## 编译与验证

```bash
colcon build --packages-select automap_pro --cmake-args -DCMAKE_BUILD_TYPE=Release
```

修改对应 YAML 后按原流程启动（如 M2DGR 回放），观察：

- `[HBA][STATE]` 是否仍出现（enabled=false 时不应再出现周期/对齐/结束触发的 HBA）。
- `[GPS_BATCH]`：`add_constraints_on_align=false` 时应出现 `skipped (gps.add_constraints_on_align=false, ISAM2 no GPS)`。

## 参考

- 双路 GTSAM 与崩溃假设：`GTSAM_MULTI_USE_AND_LOGGING.md` 第 5 节。
- HBA fallback double free 与日志：`FIX_GPS_BATCH_SIGSEGV_20260310.md` 第 7.2 节。
