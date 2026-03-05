# AutoMap-Pro 快速启动指南

## 故障诊断 (< 1 分钟)

```bash
# 快速诊断 Bag 问题
python3 scripts/diagnose_and_fix_bag.py data/automap_input/M2DGR/street_03_ros2 --verbose

# 一键修复 + 启动
bash scripts/quick_fix_and_run.sh --offline \
  --bag-file data/automap_input/M2DGR/street_03_ros2 \
  --config system_config_M2DGR.yaml
```

## 常见命令

```bash
# 【标准】离线回放建图（降速 50%）
bash run_automap.sh --offline \
  --bag-file data/automap_input/M2DGR/street_03_ros2 \
  --config system_config_M2DGR.yaml \
  --bag-rate 0.5

# 【无 UI】不启动 RViz（资源节省）
bash run_automap.sh --offline \
  --bag-file data/automap_input/M2DGR/street_03_ros2 \
  --config system_config_M2DGR.yaml \
  --no-rviz

# 【仅编译】
bash run_automap.sh --build-only

# 【清理重编】
bash run_automap.sh --clean --offline --bag-file <path>
```

## 错误排查速查表

| 错误信息 | 原因 | 解决 |
|---------|------|------|
| `yaml-cpp: bad conversion (line 25)` | metadata.yaml 格式问题 | `python3 scripts/fix_ros2_bag_metadata.py <bag_dir>` |
| `HBA: insufficient poses` | LIO 无位姿输出 | 1) 检查 Bag 回放; 2) 降速 `--bag-rate 0.5`; 3) 检查配置与 topic |
| `pose.json` 为空 | fast_livo 无输入 | 同上（级联故障） |
| `Camera model not specified` | fast_livo 参数错误 | 检查 config 中 `parameter_blackboard.model` |

## 详细文档

查看 `automap_pro/docs/TROUBLESHOOTING.md` 获得完整故障排查指南。

---

**注**：若问题仍存，请收集完整日志：
```bash
tar -czf automap_logs_$(date +%s).tar.gz /tmp/automap_logs
```
并提交 Issue。
