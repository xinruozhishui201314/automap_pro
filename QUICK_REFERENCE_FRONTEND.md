# AutoMap-Pro 前端验证快速参考

> 更新时间: 2026-03-03
> 目的: 在Docker容器内快速验证Fast-LIVO2前端里程计

---

## 一键执行（推荐）

### 方法1：使用自动验证脚本

```bash
# 1. 进入Docker容器
docker run -it --rm \
  --gpus all \
  -v $(pwd):/workspace/automap_pro \
  -v $(pwd)/data:/workspace/data \
  automap-env:humble \
  bash

# 2. 在容器内执行验证脚本
cd /workspace/automap_pro
bash test_frontend_docker.sh
```

### 方法2：手动逐步验证

```bash
# 1. 进入容器
docker run -it --rm \
  --gpus all \
  -v $(pwd):/workspace/automap_pro \
  -v $(pwd)/data:/workspace/data \
  automap-env:humble \
  bash

# 2. 设置环境
cd /workspace/automap_pro
export WORKSPACE=/workspace/automap_ws

# 3. 编译项目
cd /workspace/automap_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_livo --cmake-args -DCMAKE_BUILD_TYPE=Release

# 4. 测试参数生成
cd /workspace/automap_pro/launch
python3 -c "
from params_from_system_config import load_system_config, get_fast_livo2_params
config = load_system_config('/workspace/automap_pro/config/system_config.yaml')
params = get_fast_livo2_params(config)
print('参数生成成功')
print('LiDAR话题:', params.get('common', {}).get('lid_topic'))
print('IMU话题:', params.get('common', {}).get('imu_topic'))
print('体素大小:', params.get('lio', {}).get('voxel_size'))
"

# 5. 测试fastlivo_mapping
source /workspace/automap_ws/install/setup.bash
ros2 run fast_livo fastlivo_mapping --help
```

---

## 关键步骤说明

### 步骤1：进入容器

```bash
docker run -it --rm \
  --gpus all \
  -v $(pwd):/workspace/automap_pro \
  -v $(pwd)/data:/workspace/data \
  automap-env:humble \
  bash
```

**参数说明**:
- `--gpus all`: 启用GPU支持
- `-v $(pwd):/workspace/automap_pro`: 挂载项目代码
- `-v $(pwd)/data:/workspace/data`: 挂载数据目录
- `automap-env:humble`: 使用Humble镜像

### 步骤2：设置工作空间

```bash
cd /workspace/automap_pro
export WORKSPACE=/workspace/automap_ws

# 创建软链
ln -sfn /workspace/automap_pro /workspace/automap_ws/src/automap_pro
ln -sfn /workspace/automap_pro/fast-livo2-humble /workspace/automap_ws/src/fast_livo2-humble
```

### 步骤3：编译项目

```bash
cd /workspace/automap_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 编译fast_livo
colcon build --packages-select fast_livo --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 步骤4：验证参数生成

```bash
cd /workspace/automap_pro/launch
python3 << 'EOF'
from params_from_system_config import load_system_config, get_fast_livo2_params
config = load_system_config('/workspace/automap_pro/config/system_config.yaml')
params = get_fast_livo2_params(config)

print('=== 参数生成验证 ===')
print('✓ 参数生成成功')
print('\n关键参数:')
print(f'  LiDAR话题: {params.get("common", {}).get("lid_topic")}')
print(f'  IMU话题: {params.get("common", {}).get("imu_topic")}')
print(f'  图像话题: {params.get("common", {}).get("img_topic")}')
print(f'  体素大小: {params.get("lio", {}).get("voxel_size")}')
print(f'  最大迭代: {params.get("lio", {}).get("max_iterations")}')
print(f'  IMU积分帧: {params.get("imu", {}).get("imu_int_frame")}')
EOF
```

### 步骤5：测试节点执行

```bash
# Source环境
source /workspace/automap_ws/install/setup.bash

# 测试--help
ros2 run fast_livo fastlivo_mapping --help

# 检查可执行文件
ls -lh /workspace/automap_ws/install/fast_livo/lib/fast_livo/fastlivo_mapping
```

### 步骤6：短时间建图测试（可选）

```bash
# 启动ros2 bag play（10秒）
mkdir -p /tmp/test_output
ros2 bag play /workspace/data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2 \
  --rate 1.0 \
  --clock &
BAG_PID=$!

sleep 2

# 启动fastlivo_mapping（30秒）
timeout 35 ros2 run fast_livo fastlivo_mapping \
  --ros-args \
  -r __node:=laserMapping \
  -p use_sim_time:=true &
FAST_LIVO_PID=$!

# 监控进程
echo "运行30秒..."
for i in {1..30}; do
    echo -n "."
    sleep 1
done
echo ""

# 停止进程
kill $BAG_PID 2>/dev/null || true
kill $FAST_LIVO_PID 2>/dev/null || true
wait $BAG_PID 2>/dev/null || true
wait $FAST_LIVO_PID 2>/dev/null || true

echo "✓ 短时间建图测试完成"
```

---

## 验证检查清单

### 环境检查

- [ ] Docker容器可启动
- [ ] ROS2 Humble已安装
- [ ] Python3可用
- [ ] 工作空间已创建

### 配置检查

- [ ] system_config.yaml存在
- [ ] YAML语法正确
- [ ] LiDAR话题匹配: `/os1_cloud_node1/points`
- [ ] IMU话题匹配: `/imu/imu`

### 编译检查

- [ ] fast_livo已编译
- [ ] 可执行文件存在: `install/fast_livo/lib/fast_livo/fastlivo_mapping`
- [ ] 可执行权限正确

### 参数检查

- [ ] 参数生成脚本可用
- [ ] 参数可正确生成
- [ ] 关键参数正确:
  - [ ] lid_topic: `/os1_cloud_node1/points`
  - [ ] imu_topic: `/imu/imu`
  - [ ] voxel_size: 0.5
  - [ ] max_iterations: 5

### 节点检查

- [ ] fastlivo_mapping可执行
- [ ] `--help`参数正常
- [ ] 可正常启动（不报parameter ''错误）

---

## 常见问题

### 问题1：ImportError: No module named 'yaml'

```bash
# 解决：安装PyYAML
pip install pyyaml
```

### 问题2：parameter '' has invalid type

```bash
# 原因：fast_livo未设置automatically_declare_parameters_from_overrides(false)
# 解决：重新编译fast_livo
cd /workspace/automap_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_livo --cmake-args -DCMAKE_BUILD_TYPE=Release

# 验证修复
strings /workspace/automap_ws/install/fast_livo/lib/fast_livo/fastlivo_mapping | \
  grep automatically_declare_parameters_from_overrides
```

### 问题3：ros2 run失败

```bash
# 检查是否正确source
source /workspace/automap_ws/install/setup.bash

# 检查可执行文件
ls -lh /workspace/automap_ws/install/fast_livo/lib/fast_livo/fastlivo_mapping

# 检查权限
chmod +x /workspace/automap_ws/install/fast_livo/lib/fast_livo/fastlivo_mapping
```

### 问题4：找不到配置文件

```bash
# 检查配置文件路径
ls -lh /workspace/automap_pro/config/system_config.yaml

# 检查当前工作目录
pwd

# 使用绝对路径
python3 -c "
from params_from_system_config import load_system_config, get_fast_livo2_params
config = load_system_config('/workspace/automap_pro/config/system_config.yaml')
params = get_fast_livo2_params(config)
print('参数生成成功')
"
```

---

## 日志分析

### 查看验证日志

```bash
# 自动脚本日志
cat test_frontend_logs/frontend_test_*.log

# 查找错误
grep ERROR test_frontend_logs/frontend_test_*.log

# 查找警告
grep WARN test_frontend_logs/frontend_test_*.log

# 查找parameter ''错误
grep "parameter ''" test_frontend_logs/frontend_test_*.log
```

### 查看ROS2日志

```bash
# 查看节点日志
ros2 node list

# 查看话题列表
ros2 topic list

# 查看话题频率
ros2 topic hz /aft_mapped_to_init

# 查看话题内容
ros2 topic echo /aft_mapped_to_init --once
```

---

## 成功标准

### 环境验证成功

```
[STEP] 环境检查
[✓] 在Docker容器内运行
[✓] ROS2 Humble 已安装
[✓] Python3 已安装
```

### 配置验证成功

```
[STEP] 配置验证
[✓] 配置文件存在
[✓] YAML语法正确
[✓] LiDAR话题匹配
[✓] IMU话题匹配
```

### 编译验证成功

```
[STEP] 编译验证
[✓] Fast-LIVO2已编译
[✓] 可执行文件存在
[✓] 可执行权限正常
```

### 参数验证成功

```
[STEP] 参数生成验证
[✓] 参数生成脚本可用
[✓] 参数生成成功
[✓] 关键参数正确
```

### 节点验证成功

```
[STEP] 节点验证
[✓] fastlivo_mapping可执行
[✓] --help正常
[✓] 无parameter ''错误
```

---

## 下一步操作

### 如果前端验证全部通过

1. **执行完整建图**:
```bash
cd /workspace/automap_pro
bash run_full_mapping_enhanced.sh \
  --verbose \
  --no-convert \
  --no-ui
```

2. **验证后端优化**:
   - 检查HBA优化是否正常触发
   - 检查位姿图是否正确构建

3. **验证回环检测**:
   - 检查OverlapTransformer是否正常工作
   - 检查TEASER++配准是否成功

### 如果前端验证失败

1. **查看详细日志**:
```bash
cat test_frontend_logs/frontend_test_*.log
```

2. **检查错误信息**:
```bash
grep ERROR test_frontend_logs/frontend_test_*.log
```

3. **参考故障排查指南**:
   - 查看 TEST_FRONTEND_DOCKER_GUIDE.md
   - 查看 VERIFICATION_SUMMARY.md

---

## 参考文档

- [TEST_FRONTEND_DOCKER_GUIDE.md](TEST_FRONTEND_DOCKER_GUIDE.md) - 详细前端验证指南
- [VERIFICATION_SUMMARY.md](VERIFICATION_SUMMARY.md) - 验证总结报告
- [MAPPING_ANALYSIS_REPORT.md](MAPPING_ANALYSIS_REPORT.md) - 深度分析报告
- [FINAL_ANALYSIS.md](FINAL_ANALYSIS.md) - 最终分析总结

---

## 快速命令参考

```bash
# 进入容器
docker run -it --rm --gpus all -v $(pwd):/workspace/automap_pro -v $(pwd)/data:/workspace/data automap-env:humble bash

# 设置环境
cd /workspace/automap_pro && export WORKSPACE=/workspace/automap_ws

# 编译
cd $WORKSPACE && source /opt/ros/humble/setup.bash && colcon build --packages-select fast_livo --cmake-args -DCMAKE_BUILD_TYPE=Release

# 测试参数
cd /workspace/automap_pro/launch && python3 -c "from params_from_system_config import load_system_config, get_fast_livo2_params; config = load_system_config('/workspace/automap_pro/config/system_config.yaml'); print('✓ 参数生成成功')"

# 测试节点
source $WORKSPACE/install/setup.bash && ros2 run fast_livo fastlivo_mapping --help

# 验证脚本
bash test_frontend_docker.sh
```

---

**快速参考结束**

> 提示：如需详细说明，请参考 TEST_FRONTEND_DOCKER_GUIDE.md
