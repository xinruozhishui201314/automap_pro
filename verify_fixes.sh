#!/bin/bash
# 回环检测系统集成验证脚本
# 检查所有修复是否正确应用，并验证功能

set -e

REPO_ROOT="/home/wqs/Documents/github/automap_pro"
cd "$REPO_ROOT"

echo "=================================="
echo "回环检测系统集成验证"
echo "=================================="
echo ""

# 检查 1: 配置参数
echo "[Check 1/5] 配置参数验证..."
echo "  检查 max_range 是否为 80m..."
grep -q "max_range.*80" automap_pro/config/system_config_M2DGR.yaml && echo "  ✅ M2DGR config: max_range=80m" || echo "  ❌ M2DGR config: max_range NOT 80m"
grep -q "max_range.*80" automap_pro/config/system_config.yaml && echo "  ✅ Default config: max_range=80m" || echo "  ❌ Default config: max_range NOT 80m"

echo "  检查 overlap_threshold 是否为 0.25..."
grep -q "overlap_threshold.*0.25" automap_pro/config/system_config_M2DGR.yaml && echo "  ✅ M2DGR config: overlap_threshold=0.25" || echo "  ❌ M2DGR config: overlap_threshold NOT 0.25"

echo "  检查 top_k 是否为 8..."
grep -q "top_k.*8" automap_pro/config/system_config_M2DGR.yaml && echo "  ✅ M2DGR config: top_k=8" || echo "  ❌ M2DGR config: top_k NOT 8"
echo ""

# 检查 2: Python 侧增强
echo "[Check 2/5] Python descriptor_server 增强检查..."
echo "  检查模型权重验证代码..."
grep -q "Weight Check" automap_pro/src/modular/overlap_transformer_ros2/overlap_transformer_ros2/descriptor_server.py && echo "  ✅ 已添加权重形状验证" || echo "  ❌ 权重验证代码缺失"

echo "  检查范围图补齐代码..."
grep -q "maximum_filter" automap_pro/src/modular/overlap_transformer_ros2/overlap_transformer_ros2/descriptor_server.py && echo "  ✅ 已添加稀疏像素补齐" || echo "  ❌ 范围图优化缺失"

echo "  检查输出维度严格检查..."
grep -q "np.pad" automap_pro/src/modular/overlap_transformer_ros2/overlap_transformer_ros2/descriptor_server.py && echo "  ✅ 已添加维度检查与补齐" || echo "  ❌ 维度检查缺失"
echo ""

# 检查 3: C++ 侧增强
echo "[Check 3/5] C++ 代码增强检查..."
echo "  检查 SubMap 的 norm 缓存字段..."
grep -q "overlap_descriptor_norm" automap_pro/include/automap_pro/core/data_types.h && echo "  ✅ 已添加 norm 缓存字段" || echo "  ❌ norm 缓存字段缺失"

echo "  检查范围图补齐代码（C++）..."
grep -q "valid_pixel" automap_pro/src/loop_closure/overlap_transformer_infer.cpp && echo "  ✅ 已添加 C++ 侧稀疏像素补齐" || echo "  ❌ C++ 范围图优化缺失"

echo "  检查相似度计算优化..."
grep -q "query_norm_cache" automap_pro/src/loop_closure/overlap_transformer_infer.cpp && echo "  ✅ 已添加 norm 缓存优化" || echo "  ❌ 相似度优化缺失"

echo "  检查诊断日志..."
grep -q "METRIC" automap_pro/src/loop_closure/loop_detector.cpp && echo "  ✅ 已添加诊断指标日志" || echo "  ❌ 诊断日志缺失"
echo ""

# 检查 4: 验证脚本
echo "[Check 4/5] 验证脚本状态..."
if [ -f verify_ot_model.py ]; then
    echo "  ✅ 模型验证脚本存在"
    if python3 verify_ot_model.py 2>&1 | tail -1 | grep -q "✅"; then
        echo "  ✅ 模型验证通过"
    else
        echo "  ⚠️  模型验证需要运行检查"
    fi
else
    echo "  ❌ 验证脚本不存在"
fi
echo ""

# 检查 5: 文档
echo "[Check 5/5] 文档状态..."
if [ -f OVERLAP_TRANSFORMER_DEEP_ANALYSIS.md ]; then
    echo "  ✅ 深度分析文档存在"
else
    echo "  ⚠️  深度分析文档不存在"
fi
echo ""

echo "=================================="
echo "验证完成"
echo "=================================="
echo ""
echo "下一步："
echo "1. 等待编译完成（colcon build）"
echo "2. 运行验证脚本: python3 verify_ot_model.py"
echo "3. 离线回放测试: bash run_automap.sh --offline --bag-file <path> --config system_config_M2DGR.yaml"
echo "4. 观察日志中 [METRIC] 行查看回环检测统计"
echo ""
