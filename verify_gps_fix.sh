#!/bin/bash

# GPS-Odom偏差修复验证脚本
set -e

echo "========================================"
echo "GPS-Odom偏差修复验证"
echo "========================================"
echo ""

# 1. 检查修改的文件
echo "[步骤1] 检查修改的文件..."
FILES=(
    "automap_pro/src/frontend/gps_manager.cpp"
    "automap_pro/include/automap_pro/frontend/gps_manager.h"
    "automap_pro/config/system_config_M2DGR.yaml"
)

for file in "${FILES[@]}"; do
    if [ -f "$file" ]; then
        echo " ✓ 文件存在: $file"
    else
        echo " ✗ 文件缺失: $file"
        exit 1
    fi
done
echo ""

# 2. 验证代码修改
echo "[步骤2] 验证代码修改..."

if grep -q "hdop <= 10.0" automap_pro/src/frontend/gps_manager.cpp; then
    echo " ✓ HDOP阈值已放宽到10.0"
else
    echo " ✗ HDOP阈值未修改"
    exit 1
fi

if grep -q "quality >= GPSQuality::MEDIUM" automap_pro/src/frontend/gps_manager.cpp; then
    echo " ✓ 对齐条件已放宽到MEDIUM+"
else
    echo " ✗ 对齐条件未修改"
    exit 1
fi

if grep -q "estimateGpsPositionByOdom" automap_pro/src/frontend/gps_manager.cpp; then
    echo " ✓ 新增里程计插值估计方法"
else
    echo " ✗ 里程计插值估计方法缺失"
    exit 1
fi

if grep -q "onlineCalibrate" automap_pro/src/frontend/gps_manager.cpp; then
    echo " ✓ 新增在线校正方法"
else
    echo " ✗ 在线校正方法缺失"
    exit 1
fi

echo ""

# 3. 验证配置修改
echo "[步骤3] 验证配置修改..."

if grep -q "quality_threshold_hdop: 12.0" automap_pro/config/system_config_M2DGR.yaml; then
    echo " ✓ M2DGR配置: quality_threshold_hdop已调整为12.0"
else
    echo " ✗ quality_threshold_hdop未修改"
    exit 1
fi

if grep -q "align_rmse_threshold_m: 5.0" automap_pro/config/system_config_M2DGR.yaml; then
    echo " ✓ M2DGR配置: align_rmse_threshold_m已调整为5.0"
else
    echo " ✗ align_rmse_threshold_m未修改"
    exit 1
fi

echo ""
echo "========================================"
echo " ✓ 所有验证通过！"
echo "========================================"
echo ""
echo "后续步骤："
echo "  1. 运行建图: make run_offline_docker"
echo "  2. 检查日志: grep -E 'GPS_DIAG|GPS_STATE|ALIGNED' logs/full.log"
echo "  3. 分析偏差: python3 scripts/plot_trajectory_compare.py automap_ws/logs/trajectory_odom_*.csv --stats"
echo ""
