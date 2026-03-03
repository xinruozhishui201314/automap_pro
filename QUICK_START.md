# AutoMap-Pro 快速验证清单

## ✅ 修复完成

原始报错命令现在可以正常工作：
```bash
./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag
```

---

## 🚀 快速验证（推荐）

```bash
# 一键验证修复
./verify_fix.sh
```

**预期结果**：所有测试通过 ✅

---

## 📋 完整测试清单

### 1. 语法检查
```bash
bash -n run_full_mapping_docker.sh
```
**预期**：无输出（语法正确）

### 2. 原始报错场景测试
```bash
echo "n" | timeout 15 ./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag 2>&1 | head -20
```
**预期**：
- 显示 "找到匹配文件: data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag"
- 不再报 "Bag 文件不存在"

### 3. 其他路径格式测试
```bash
# 短路径不带 @
echo "n" | timeout 10 ./run_full_mapping_docker.sh -b data/automap_input/nya_02.bag 2>&1 | grep -E "(找到匹配|Bag 文件)"

# 仅文件名
echo "n" | timeout 10 ./run_full_mapping_docker.sh -b nya_02.bag 2>&1 | grep -E "(找到匹配|Bag 文件)"

# 默认参数
echo "n" | timeout 10 ./run_full_mapping_docker.sh 2>&1 | grep -E "(找到匹配|Bag 文件)"
```

---

## ✅ 实际运行建图

确认验证通过后，可以运行实际建图：

```bash
# 前台运行（推荐用于测试）
./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag

# 后台运行（推荐用于生产）
./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag --detach

# 进入容器 shell（调试用）
./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag --shell
```

---

## 📚 相关文档

- 完整修复文档：`FIX_DOCUMENTATION.md`
- 主脚本：`run_full_mapping_docker.sh`
- 验证脚本：`verify_fix.sh`

---

## 🐛 问题排查

如果验证失败，检查以下内容：

1. **脚本权限**
   ```bash
   chmod +x run_full_mapping_docker.sh verify_fix.sh
   ```

2. **Docker 状态**
   ```bash
   docker ps  # 检查容器运行状态
   ```

3. **Bag 文件存在性**
   ```bash
   ls -lh data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
   ```

4. **查看详细日志**
   ```bash
   ./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag 2>&1 | tee mapping.log
   ```

---

## 🎯 支持的路径格式

| 格式 | 示例 | 说明 |
|------|------|------|
| @ 前缀短路径 | `@data/automap_input/nya_02.bag` | 原始报错的格式，现已支持 ✅ |
| 短路径 | `data/automap_input/nya_02.bag` | 不带 @ 的相对路径 |
| 仅文件名 | `nya_02.bag` | 在 data 目录下搜索 |
| 完整路径 | `data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag` | 精确匹配 |
| 绝对路径 | `/home/.../nya_02.bag` | 绝对路径 |
| 默认参数 | （不指定 -b） | 使用默认 bag 文件 |

---

## 🔍 验证成功标志

运行 `./verify_fix.sh` 后，应该看到：

```
========================================
AutoMap-Pro 路径修复验证
========================================

测试 1: 原始报错命令 -b @data/automap_input/nya_02.bag
----------------------------------------
✓ PASS 成功找到匹配文件
[INFO] ✓ 找到匹配文件: data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
✓ PASS 脚本成功运行到确认步骤

========================================
✓ 验证通过！修复成功！
========================================
```

如果看到上述输出，说明修复成功！🎉
