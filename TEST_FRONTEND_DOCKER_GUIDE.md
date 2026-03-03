# AutoMap-Pro 前端验证指南 (Docker容器内)

> 目的：在ROS2 Humble Docker容器内测试验证Fast-LIVO2前端里程计
> 日期：2026-03-03

---

## 0. Executive Summary

本指南提供在Docker容器内测试验证Fast-LIVO2前端里程计的完整流程，包括：
- 环境检查
- 配置验证
- 参数生成测试
- 节点可执行性测试
- 短时间建图测试

---

## 1. 前置条件

### 1.1 Docker镜像

确保Docker镜像已构建并存在：

```bash
# 检查镜像
docker images | grep automap-env

# 预期输出：
# automap-env:humble    9b192a834b7d    22.3GB    7.08GB
```

### 1.2 数据文件

确保nya_02_ros2数据文件存在：

```bash
# 检查数据
ls -lh data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2/metadata.yaml

# 预期输出：
# -rw-r--r-- 1 root root 5.9K Mar 1 22:00 .../metadata.yaml
```

### 1.3 配置文件

确保system_config.yaml存在：

```bash
# 检查配置
ls -lh automap_pro/config/system_config.yaml

# 预期输出：
# -rw-rw-r-- 1 wqs wqs 11K Mar 1 16:57 automap_pro/config/system_config.yaml
```

---

## 2. 进入Docker容器

### 2.1 启动容器并进入

```bash
# 进入容器
docker run -it --rm \
  --gpus all \
  -v $(pwd):/workspace/automap_pro \
  -v $(pwd)/data:/workspace/data \
  automap-env:humble \
  bash
```

### 2.2 容器内配置

进入容器后，设置环境变量：

```bash
# 设置环境
cd /workspace/automap_pro
export WORKSPACE=/workspace/automap_ws

# 检查环境
echo "当前目录: $(pwd)"
echo "工作空间: $WORKSPACE"
```

---

## 3. 执行前端验证

### 3.1 方式1：使用自动验证脚本

```bash
# 运行前端验证脚本
bash test_frontend_docker.sh
```

该脚本将自动执行以下步骤：
1. ✅ 检查Docker环境
2. ✅ 检查ROS2环境
3. ✅ 检查工作空间
4. ✅ 检查配置文件
5. ✅ 检查Bag数据
6. ✅ 检查Fast-LIVO2编译
7. ✅ 测试参数生成
8. ✅ 测试节点可执行性
9. ✅ 测试Launch文件
10. ✅ 检查ROS2话题
11. ✅ 测试短时间建图

### 3.2 方式2：手动逐步验证

如果自动脚本执行失败，可以手动逐步验证。

#### 步骤1：检查ROS2环境

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# 检查版本
ros2 --version
# 预期输出: ros2 2.x.x
```

#### 步骤2：设置工作空间

```bash
# 创建工作空间
mkdir -p /workspace/automap_ws/src

# 软链源码
ln -sfn /workspace/automap_pro /workspace/automap_ws/src/automap_pro
ln -sfn /workspace/automap_pro/fast-livo2-humble /workspace/automap_ws/src/fast-livo2-humble 2>/dev/null || true

# 安装依赖
cd /workspace/automap_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

#### 步骤3：编译项目

```bash
# 编译automap_pro
colcon build --packages-select automap_pro --cmake-args -DCMAKE_BUILD_TYPE=Release

# 编译fast_livo
colcon build --packages-select fast_livo --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### 步骤4：检查编译产物

```bash
# 检查automap_pro
ls -lh /workspace/automap_ws/install/automap_pro/lib/automap_pro/

# 检查fast_livo
ls -lh /workspace/automap_ws/install/fast_livo/lib/fast_livo/

# 预期输出:
# fastlivo_mapping
```

#### 步骤5：测试参数生成

```bash
# Source工作空间
cd /workspace/automap_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 测试参数生成
cd /workspace/automap_pro/launch

python3 -c "
from params_from_system_config import load_system_config, get_fast_livo2_params

config = load_system_config('/workspace/automap_pro/automap_pro/config/system_config.yaml')
params = get_fast_livo2_params(config)

print('✓ 参数生成成功')
print('关键参数:')
print('  lid_topic:', params.get('common', {}).get('lid_topic'))
print('  imu_topic:', params.get('common', {}).get('imu_topic'))
print('  voxel_size:', params.get('lio', {}).get('voxel_size'))
print('  max_iterations:', params.get('lio', {}).get('max_iterations'))
"
```

#### 步骤6：生成参数文件

```bash
# 生成参数文件
python3 -c "
from params_from_system_config import load_system_config, get_fast_livo2_params, write_fast_livo_params_file

config = load_system_config('/workspace/automap_pro/automap_pro/config/system_config.yaml')
write_fast_livo_params_file(config, '/tmp/test_fast_livo_params.yaml')

print('✓ 参数文件生成: /tmp/test_fast_livo_params.yaml')
"

# 检查参数文件
ls -lh /tmp/test_fast_livo_params.yaml
head -30 /tmp/test_fast_livo_params.yaml
```

#### 步骤7：测试fastlivo_mapping可执行性

```bash
# 测试--help
ros2 run fast_livo fastlivo_mapping --help

# 预期输出:
# Usage: fastlivo_mapping [OPTIONS]
# ...
```

#### 步骤8：测试短时间建图

```bash
# 启动ros2 bag play（只播放10秒）
mkdir -p /tmp/test_output
ros2 bag play /workspace/data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2 \
  --rate 1.0 \
  --clock \
  &
BAG_PID=$!

sleep 2

# 启动fastlivo_mapping（使用参数文件）
ros2 run fast_livo fastlivo_mapping \
  --ros-args \
  -r __node:=laserMapping \
  -p use_sim_time:=true \
  --params-file /tmp/test_fast_livo_params.yaml \
  &
FAST_LIVO_PID=$!

# 运行10秒
echo "运行10秒..."
for i in {1..10}; do
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

## 4. 检查验证结果

### 4.1 检查日志

```bash
# 如果使用自动脚本
cat test_frontend_logs/frontend_test_*.log

# 查找错误
grep ERROR test_frontend_logs/frontend_test_*.log

# 查找警告
grep WARN test_frontend_logs/frontend_test_*.log
```

### 4.2 检查ROS2话题

```bash
# 检查话题列表
ros2 topic list

# 检查fastlivo输出话题
ros2 topic list | grep -E "aft_mapped|cloud_registered"

# 预期输出:
# /aft_mapped_to_init
# /cloud_registered
```

### 4.3 检查话题频率

```bash
# 检查位姿话题频率
ros2 topic hz /aft_mapped_to_init

# 预期输出:
# average rate: 10.0 (或接近LiDAR频率)
```

### 4.4 检查节点状态

```bash
# 检查节点列表
ros2 node list

# 预期输出:
# /laserMapping
# (如果有ros2 bag play，还会有rosbag相关节点)
```

---

## 5. 常见问题排查

### 5.1 问题：parameter '' has invalid type

**原因**: fast_livo未设置`automatically_declare_parameters_from_overrides(false)`

**解决**:
```bash
# 重新编译fast_livo（确保main.cpp中已设置该参数）
cd /workspace/automap_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_livo --cmake-args -DCMAKE_BUILD_TYPE=Release

# 验证修复
strings /workspace/automap_ws/install/fast_livo/lib/fast_livo/fastlivo_mapping | \
  grep automatically_declare_parameters_from_overrides
```

### 5.2 问题：ImportError: No module named 'yaml'

**原因**: Python缺少yaml模块

**解决**:
```bash
# 安装yaml
pip install pyyaml
```

### 5.3 问题：ros2 run失败

**原因**: 工作空间未source或编译不完整

**解决**:
```bash
# Source工作空间
cd /workspace/automap_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 重新编译
colcon build --packages-select fast_livo --cmake-args -DCMAKE_BUILD_TYPE=Release

# 检查可执行文件
ls -lh install/fast_livo/lib/fast_livo/fastlivo_mapping
```

### 5.4 问题：ros2 bag play失败

**原因**: Bag文件路径错误或格式不支持

**解决**:
```bash
# 检查Bag文件
ls -lh /workspace/data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2/metadata.yaml

# 检查Bag信息
ros2 bag info /workspace/data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2
```

---

## 6. 完整建图流程

如果前端验证通过，可以执行完整建图流程：

```bash
# 在容器内执行
cd /workspace/automap_pro

# 方式1：使用完整脚本
bash run_full_mapping_enhanced.sh \
  --verbose \
  --no-convert \
  --no-ui

# 方式2：手动执行
ros2 launch automap_pro automap_offline.launch.py \
  config:=automap_pro/config/system_config.yaml \
  bag_file:=data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2 \
  rate:=1.0 \
  use_rviz:=false \
  use_external_frontend:=true
```

---

## 7. 验证清单

### 7.1 环境验证

- [ ] Docker容器可启动
- [ ] ROS2 Humble已安装
- [ ] Python3可用
- [ ] 工作空间已创建

### 7.2 配置验证

- [ ] system_config.yaml存在
- [ ] YAML语法正确
- [ ] 话题与数据匹配

### 7.3 编译验证

- [ ] automap_pro已编译
- [ ] fast_livo已编译
- [ ] 可执行文件存在

### 7.4 参数验证

- [ ] 参数生成脚本可用
- [ ] 参数可正确生成
- [ ] 参数文件可写入

### 7.5 节点验证

- [ ] automap_system_node可执行
- [ ] fastlivo_mapping可执行
- [ ] Launch文件可导入

### 7.6 运行验证

- [ ] ros2 bag play可执行
- [ ] fastlivo_mapping可启动
- [ ] 话题正常发布
- [ ] 位姿话题有输出

---

## 8. 日志分析

### 8.1 快速检查

```bash
# 查看日志文件
ls -lh test_frontend_logs/frontend_test_*.log

# 查看最新日志
cat test_frontend_logs/frontend_test_*.log | tail -50
```

### 8.2 错误检查

```bash
# 查找所有ERROR
grep -n "ERROR" test_frontend_logs/frontend_test_*.log

# 查找parameter ''错误
grep -n "parameter ''" test_frontend_logs/frontend_test_*.log
```

### 8.3 关键词搜索

```bash
# 搜索"完成"或"成功"
grep -n "✓\|完成\|成功" test_frontend_logs/frontend_test_*.log

# 搜索"失败"或"错误"
grep -n "✗\|失败\|错误" test_frontend_logs/frontend_test_*.log
```

---

## 9. 总结

### 9.1 验证目标

- ✅ 验证Fast-LIVO2可以在Docker容器内正确编译
- ✅ 验证参数可以从system_config.yaml正确生成
- ✅ 验证fastlivo_mapping节点可以正常启动
- ✅ 验证可以正确处理nya_02_ros2数据

### 9.2 验证方法

- 自动验证脚本：`bash test_frontend_docker.sh`
- 手动逐步验证：按照步骤1-8逐一验证

### 9.3 成功标准

- 所有环境检查通过
- 所有配置检查通过
- 所有编译检查通过
- 参数生成成功
- 节点可以启动
- 话题正常发布

### 9.4 下一步

如果前端验证全部通过，可以：
1. 执行完整建图流程
2. 验证后端优化功能
3. 验证回环检测功能
4. 验证地图输出功能

---

**文档结束**
