# AutoMap-Pro 路径解析修复文档

## Executive Summary

**问题**：`run_full_mapping_docker.sh` 脚本在处理 `@data/automap_input/nya_02.bag` 时报错 "Bag 文件不存在"

**根因**：用户输入的短路径与实际文件位置不匹配，脚本缺乏智能匹配机制

**修复**：添加智能路径解析功能，支持模糊匹配和自动搜索

**验证状态**：✅ 所有测试通过

---

## 1. 问题分析

### 1.1 原始报错场景

```bash
./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag
```

**错误信息**：
```
[ERROR] Bag 文件不存在: /home/wqs/Documents/github/automap_pro/@data/automap_input/nya_02.bag
[INFO] 可用的 bag 文件:
/home/wqs/Documents/github/automap_pro/data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
```

### 1.2 根本原因

| 原因 | 说明 |
|------|------|
| 路径不精确 | 用户输入：`data/automap_input/nya_02.bag` |
| 实际位置 | `data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag` |
| 脚本逻辑 | 脚本仅做精确匹配，无模糊搜索能力 |

---

## 2. 解决方案设计

### 2.1 路径解析流程

```mermaid
flowchart TD
    A[输入: @data/automap_input/nya_02.bag] --> B[去掉@前缀]
    B --> C[转换为绝对路径]
    C --> D{文件存在?}
    D -->|是| E[使用精确路径]
    D -->|否| F[提取文件名: nya_02]
    F --> G[搜索策略1: 精确文件名匹配]
    G --> H{找到?}
    H -->|是| I[返回匹配路径]
    H -->|否| J[搜索策略2: 模糊匹配 *nya_02*.bag]
    J --> K{找到?}
    K -->|是| I
    K -->|否| L[报错: 未找到匹配文件]

    style I fill:#90EE90
    style L fill:#FFB6C1
```

### 2.2 修复内容

#### 文件修改：`run_full_mapping_docker.sh`

**新增函数：`resolve_bag_file()`**

- 精确路径检查
- 绝对路径/相对路径支持
- 智能搜索（两级策略）
- 友好的错误提示

**修改点**：
1. 第 141-184 行：新增 `resolve_bag_file()` 函数
2. 第 357-381 行：在 `main()` 函数中调用路径解析
3. 第 222-249 行：保持路径检查作为双重保险

---

## 3. 编译/部署/运行说明

### 3.1 环境要求

- OS: Linux (Ubuntu 22.04+)
- Docker: 已安装并运行
- Bash: 4.0+

### 3.2 无需编译

本次修复仅为 Shell 脚本逻辑优化，无需编译。

### 3.3 验证修复

```bash
# 快速验证
./verify_fix.sh
```

**预期输出**：
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

### 3.4 运行建图

支持的所有路径格式：

```bash
# 方式1: 使用 @ 前缀（原始报错的命令，现已修复）
./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag

# 方式2: 不带 @ 的短路径
./run_full_mapping_docker.sh -b data/automap_input/nya_02.bag

# 方式3: 仅文件名
./run_full_mapping_docker.sh -b nya_02.bag

# 方式4: 完整路径
./run_full_mapping_docker.sh -b data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag

# 方式5: 默认参数（不指定 bag，使用默认值）
./run_full_mapping_docker.sh

# 方式6: 后台运行
./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag --detach
```

---

## 4. 验证与回归测试

### 4.1 测试覆盖

| 测试场景 | 输入 | 预期结果 | 状态 |
|---------|------|---------|------|
| 原始报错命令 | `-b @data/automap_input/nya_02.bag` | 自动匹配到正确路径 | ✅ PASS |
| 短路径不带@ | `-b data/automap_input/nya_02.bag` | 自动匹配到正确路径 | ✅ PASS |
| 仅文件名 | `-b nya_02.bag` | 自动匹配到正确路径 | ✅ PASS |
| 完整路径 | `-b data/.../nya_02_slam_imu_to_lidar/nya_02.bag` | 精确匹配 | ✅ PASS |
| 默认参数 | （不指定 -b） | 使用默认路径 | ✅ PASS |

### 4.2 自动化验证脚本

**快速验证**：`./verify_fix.sh`
- 测试原始报错场景
- 验证模糊匹配功能
- 确认脚本正常流程

**完整测试**：`./test_bag_path_resolution.sh`
- 测试所有路径格式
- 生成详细报告

---

## 5. 风险与回滚方案

### 5.1 风险评估

| 风险项 | 影响 | 概率 | 缓解措施 |
|--------|------|------|---------|
| 模糊匹配错误路径 | 中 | 低 | 优先精确匹配，模糊匹配时显示匹配结果 |
| 性能影响 | 低 | 低 | 搜索限制在 data 目录，find 使用 head -1 |
| 向后兼容 | 无 | - | 保持精确路径优先，不影响现有用法 |

### 5.2 回滚策略

如需回滚，执行以下步骤：

```bash
# 1. 备份当前版本
cp run_full_mapping_docker.sh run_full_mapping_docker.sh.fixed

# 2. 使用 Git 回滚
git checkout HEAD -- run_full_mapping_docker.sh

# 3. 验证回滚
bash -n run_full_mapping_docker.sh
```

---

## 6. 观测性与运维

### 6.1 日志输出

修复后的脚本提供清晰的日志：

```
[WARN] 文件不存在: /path/to/input.bag
[INFO] 正在搜索匹配的 bag 文件...
[INFO] ✓ 找到匹配文件: data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
```

### 6.2 故障排查

| 问题 | 可能原因 | 解决方法 |
|------|---------|---------|
| 仍然报错找不到 bag | 文件不在 data 目录 | 检查文件位置，使用绝对路径 |
| 找到错误的 bag | 多个相似文件 | 使用完整路径或更精确的文件名 |
| 搜索慢 | data 目录文件过多 | 使用完整路径跳过搜索 |

---

## 7. 后续演进路线图

### MVP（已完成）
- ✅ 基础路径解析
- ✅ 模糊匹配
- ✅ 错误提示

### V1（规划中）
- [ ] 支持多个 bag 文件批量处理
- [ ] 路径缓存机制（避免重复搜索）
- [ ] 交互式选择（多个匹配时询问用户）

### V2（未来）
- [ ] 支持正则表达式匹配
- [ ] 自动扫描常用 bag 目录
- [ ] 集成到 IDE 工具（如 VS Code）

---

## 8. 代码参考

### 关键代码片段

**路径解析函数**（第 141-184 行）：

```141:184:run_full_mapping_docker.sh
resolve_bag_file() {
    local input_path="$1"
    local resolved_path=""

    # 如果是绝对路径，直接检查
    if [[ "$input_path" == /* ]]; then
        if [ -f "$input_path" ]; then
            echo "$input_path"
            return 0
        else
            echo ""
            return 1
        fi
    fi

    # 转换为绝对路径
    local abs_path="$SCRIPT_DIR/$input_path"

    # 精确匹配
    if [ -f "$abs_path" ]; then
        echo "$input_path"
        return 0
    fi

    # 如果文件不存在，尝试智能搜索
    local bag_name=$(basename "$input_path" .bag)
    log_warn "文件不存在: $abs_path"
    log_info "正在搜索匹配的 bag 文件..."

    # 搜索策略1: 精确文件名匹配
    local found=$(find "$SCRIPT_DIR/data" -name "${bag_name}.bag" 2>/dev/null | head -1)
    if [ -n "$found" ]; then
        resolved_path="${found#$SCRIPT_DIR/}"
        log_info "✓ 找到匹配文件: $resolved_path"
        echo "$resolved_path"
        return 0
    fi

    # 搜索策略2: 模糊匹配（包含 bag_name）
    found=$(find "$SCRIPT_DIR/data" -name "*${bag_name}*.bag" 2>/dev/null | head -1)
    if [ -n "$found" ]; then
        resolved_path="${found#$SCRIPT_DIR/}"
        log_info "✓ 找到相似文件: $resolved_path"
        echo "$resolved_path"
        return 0
    fi

    # 搜索失败，返回空
    log_error "未找到匹配的 bag 文件"
    echo ""
    return 1
}
```

---

## 9. 总结

| 项目 | 说明 |
|------|------|
| **修复状态** | ✅ 完成 |
| **测试状态** | ✅ 所有测试通过 |
| **向后兼容** | ✅ 完全兼容 |
| **性能影响** | ✅ 可忽略 |
| **可回滚** | ✅ Git 支持 |
| **文档完整性** | ✅ 包含使用说明、验证方法、故障排查 |

**下一步行动**：
1. 运行 `./verify_fix.sh` 确认修复
2. 尝试使用 `./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag` 实际运行建图
3. 根据需要扩展其他场景的路径解析

---

**修复完成时间**：2026-03-01
**修复版本**：v1.0
**验证者**：自动化验证脚本 + 手工验证
