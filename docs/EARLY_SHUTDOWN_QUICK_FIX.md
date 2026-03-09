# AutoMap-Pro 过早关闭快速修复方案

## Executive Summary

**问题**：系统在 rosbag2 数据未完全播放时就触发了传感器空闲超时（10秒），导致约 40-50% 的数据未被处理。

**快速修复方案**：将离线模式下的传感器空闲超时从 10 秒增加到 7200 秒（2 小时），确保 bag 数据完全播放完。

**预期效果**：✅ 立即生效，简单可靠

---

## 1. 快速修复方案（5 分钟）

### 1.1 修改配置文件

编辑 `/root/automap_ws/src/automap_pro/config/system_config_M2DGR.yaml`：

```yaml
# 找到以下配置项（大约在第 80-100 行）
sensor:
  # 在线模式传感器空闲超时（秒）
  idle_timeout_sec: 10.0  # 在线模式：10 秒
  
# 新增或修改以下配置
mode:
  # 运行模式：online | offline
  type: offline
  
  # 在线模式配置
  online:
    # 传感器空闲超时（秒）
    sensor_idle_timeout_sec: 10.0
  
  # 离线模式配置
  offline:
    # 传感器空闲超时（秒）- 改为 7200 秒（2 小时）
    sensor_idle_timeout_sec: 7200.0
```

### 1.2 验证配置

运行以下命令验证配置：

```bash
# 检查配置文件
grep -A 10 "mode:" /root/automap_ws/src/automap_pro/config/system_config_M2DGR.yaml
```

预期输出：
```yaml
mode:
  # 运行模式：online | offline
  type: offline
  # ...
  offline:
    # 传感器空闲超时（秒）
    sensor_idle_timeout_sec: 7200.0
```

### 1.3 代码修改（最小改动）

修改 `automap_pro/src/core/config_manager.cpp`，添加离线模式超时支持：

```cpp
// 在 ConfigManager::load() 函数中添加（大约在第 50-60 行）

// ... 现有代码 ...

// 新增：读取运行模式和离线模式超时配置
std::string mode_type = "offline";
double online_timeout = 10.0;
double offline_timeout = 7200.0;

try {
    mode_type = config_file["mode"]["type"].as<std::string>();
} catch (...) {
    RCLCPP_WARN(get_logger(), "[CONFIG] mode.type not set, defaulting to offline");
}

try {
    online_timeout = config_file["mode"]["online"]["sensor_idle_timeout_sec"].as<double>();
} catch (...) {
    RCLCPP_WARN(get_logger(), "[CONFIG] mode.online.sensor_idle_timeout_sec not set, defaulting to 10.0");
}

try {
    offline_timeout = config_file["mode"]["offline"]["sensor_idle_timeout_sec"].as<double>();
} catch (...) {
    RCLCPP_WARN(get_logger(), "[CONFIG] mode.offline.sensor_idle_timeout_sec not set, defaulting to 7200.0");
}

// 根据模式设置超时
if (mode_type == "offline") {
    sensor_idle_timeout_sec_ = offline_timeout;
    RCLCPP_INFO(get_logger(), "[CONFIG] Offline mode, sensor_idle_timeout = %.1fs", offline_timeout);
} else {
    sensor_idle_timeout_sec_ = online_timeout;
    RCLCPP_INFO(get_logger(), "[CONFIG] Online mode, sensor_idle_timeout = %.1fs", online_timeout);
}

// ... 继续现有代码 ...
```

修改 `automap_pro/include/automap_pro/core/config_manager.h`：

```cpp
// 在类定义中添加成员变量（如果还没有的话）
class ConfigManager {
private:
    // ... 现有成员 ...
    
    double sensor_idle_timeout_sec_{10.0};
    
    // ... 其他成员 ...
    
public:
    // 新增：获取传感器空闲超时
    double getSensorIdleTimeoutSec() const {
        return sensor_idle_timeout_sec_;
    }
};
```

### 1.4 重新编译

```bash
cd /root/automap_ws
source install/setup.bash
colcon build --packages-select automap_pro --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 2. 验证修复

### 2.1 运行测试

```bash
# 清理旧的日志
rm -rf logs/*.log logs/*.csv

# 运行系统
ros2 launch automap_pro automap_offline.launch.py \
    config:=/root/automap_ws/src/automap_pro/config/system_config_M2DGR.yaml
```

### 2.2 检查日志

```bash
# 1. 确认使用了离线模式超时
grep "sensor_idle_timeout" logs/automap.log
# 预期输出：[CONFIG] Offline mode, sensor_idle_timeout = 7200.0s

# 2. 检查处理的数据帧数量
grep "cloud #" logs/full.log | wc -l
# 预期：应该显著增加（从约 1941 到 bag 的总帧数，可能 5000+）

# 3. 检查关键帧数量
grep "created kf_id=" logs/full.log | wc -l
# 预期：应该显著增加（从 201 到更多，可能 500+）

# 4. 检查是否正确等待 bag 完成
grep "context shutdown" logs/full.log
# 预期：应该在 log 的最后出现

# 5. 检查地图大小
ls -lh /data/automap_output/global_map.pcd
# 预期：文件大小应该显著增加（点数应该更多）
```

### 2.3 对比前后

| 指标 | 修复前 | 修复后（预期） |
|------|--------|---------------|
| 处理的帧数 | ~1941 | ~5000+ (所有数据) |
| 关键帧数 | 201 | ~500+ |
| 地图点数 | 535,004 | ~1,000,000+ |
| 运行时长 | ~7 分钟 | ~14 分钟（因为 0.5x 播放） |

---

## 3. 代码修改详细说明

### 3.1 config_manager.cpp 完整修改

找到 `automap_pro/src/core/config_manager.cpp` 中的 `ConfigManager::load()` 函数，在适当位置插入以下代码：

```cpp
// ========== 快速修复：离线模式超时配置 ==========
// 读取运行模式和离线模式超时配置
std::string mode_type = "offline";
double online_timeout = 10.0;
double offline_timeout = 7200.0;

// 读取模式类型
try {
    if (config_file["mode"]) {
        if (config_file["mode"]["type"]) {
            mode_type = config_file["mode"]["type"].as<std::string>();
        }
    }
} catch (...) {
    RCLCPP_WARN(get_logger(), "[CONFIG] mode.type not set, defaulting to offline");
}

// 读取在线模式超时
try {
    online_timeout = config_file["mode"]["online"]["sensor_idle_timeout_sec"].as<double>();
} catch (...) {
    RCLCPP_WARN(get_logger(), "[CONFIG] mode.online.sensor_idle_timeout_sec not set, defaulting to 10.0");
}

// 读取离线模式超时
try {
    offline_timeout = config_file["mode"]["offline"]["sensor_idle_timeout_sec"].as<double>();
} catch (...) {
    RCLCPP_WARN(get_logger(), "[CONFIG] mode.offline.sensor_idle_timeout_sec not set, defaulting to 7200.0");
}

// 根据模式设置超时
if (mode_type == "offline") {
    sensor_idle_timeout_sec_ = offline_timeout;
    RCLCPP_INFO(get_logger(), "[CONFIG] Offline mode, sensor_idle_timeout = %.1fs", offline_timeout);
} else {
    sensor_idle_timeout_sec_ = online_timeout;
    RCLCPP_INFO(get_logger(), "[CONFIG] Online mode, sensor_idle_timeout = %.1fs", online_timeout);
}
// ========== 快速修复结束 ==========

// 确保在类定义中添加了成员变量 sensor_idle_timeout_sec_
```

### 3.2 config_manager.h 完整修改

在 `automap_pro/include/automap_pro/core/config_manager.h` 的 `ConfigManager` 类中添加：

```cpp
class ConfigManager {
private:
    // ... 现有成员 ...
    
    // ========== 快速修复：传感器空闲超时配置 ==========
    double sensor_idle_timeout_sec_{10.0};
    // ========== 快速修复结束 ==========
    
    // ... 其他成员 ...
    
public:
    // 新增：获取传感器空闲超时
    double getSensorIdleTimeoutSec() const {
        return sensor_idle_timeout_sec_;
    }
    
    // ... 其他现有方法 ...
};
```

### 3.3 automap_system.cpp 修改

在 `automap_pro/src/system/automap_system.cpp` 中，找到使用 `sensor_idle_timeout_sec_` 的地方，修改为从 ConfigManager 获取：

```cpp
// 找到类似这样的代码：
// if (current_time - last_sensor_timestamp_ > sensor_idle_timeout_sec_) {

// 修改为：
double timeout_sec = config_manager_->getSensorIdleTimeoutSec();
if (current_time - last_sensor_timestamp_ > timeout_sec) {
    // ...
}

// 确保在类定义中添加了成员变量：
// std::shared_ptr<ConfigManager> config_manager_;
```

---

## 4. 故障排查

### 4.1 问题：编译失败

**现象**：`colcon build` 报错

**原因**：
- 缺少 YAML 配置项
- 类型不匹配

**解决**：
```bash
# 1. 检查配置文件语法
python3 -c "import yaml; yaml.safe_load(open('/root/automap_ws/src/automap_pro/config/system_config_M2DGR.yaml'))"

# 2. 查看完整编译错误
colcon build --packages-select automap_pro 2>&1 | grep -A 5 "error:"
```

### 4.2 问题：运行时超时仍然太短

**现象**：日志显示 `sensor_idle_timeout = 10.0s`

**原因**：
- 配置文件未正确加载
- 使用了旧的配置

**解决**：
```bash
# 1. 确认配置文件内容
grep -A 15 "mode:" /root/automap_ws/src/automap_pro/config/system_config_M2DGR.yaml

# 2. 重新编译
colcon build --packages-select automap_pro --symlink-install
source install/setup.bash

# 3. 确认使用了正确的配置
grep "sensor_idle_timeout" logs/automap.log
```

### 4.3 问题：系统运行时间过长

**现象**：系统运行超过 2 小时仍未退出

**原因**：
- 离线模式超时设置过长
- Rosbag2 播放卡住

**解决**：
```bash
# 1. 检查 rosbag2 是否还在播放
ps aux | grep "ros2 bag play"

# 2. 检查系统是否在处理数据
tail -f logs/full.log | grep "cloud #"

# 3. 如果确实卡住，可以手动终止（Ctrl+C）
# 或者修改配置将超时减少到合理值
```

---

## 5. 回滚方案

如果快速修复方案出现问题，可以通过以下方式回滚：

### 5.1 配置回滚

修改配置文件，移除 `mode` 配置段：

```yaml
# system_config_M2DGR.yaml

# 删除或注释掉 mode 配置段
# mode:
#   type: offline
#   online:
#     sensor_idle_timeout_sec: 10.0
#   offline:
#     sensor_idle_timeout_sec: 7200.0

# 使用原有的配置
sensor:
  idle_timeout_sec: 10.0
```

### 5.2 代码回滚

恢复原始的代码版本，使用 git：

```bash
cd /root/automap_ws/src/automap_pro
git diff HEAD -- src/core/config_manager.cpp include/automap_pro/core/config_manager.h src/system/automap_system.cpp

# 查看修改内容后，决定是否回滚
git checkout -- src/core/config_manager.cpp include/automap_pro/core/config_manager.h src/system/automap_system.cpp
```

或者手动删除快速修复添加的代码。

---

## 6. 验证清单

- [ ] 配置文件已修改并验证
- [ ] 代码已修改并编译成功
- [ ] 运行日志显示使用 7200 秒超时
- [ ] 处理的数据帧数显著增加
- [ ] 关键帧数显著增加
- [ ] 地图点数显著增加
- [ ] 系统在 bag 播放完成后正常退出
- [ ] 生成的地图文件大小合理

---

## 7. 总结

**快速修复方案**：
- ✅ 修改配置文件，添加离线模式超时配置
- ✅ 修改代码，支持读取模式相关的超时配置
- ✅ 重新编译并运行
- ✅ 验证数据完整性

**预期效果**：
- ✅ 所有 bag 数据被正确处理
- ✅ 系统在 bag 播放完成后自动退出
- ✅ 地图完整性和准确性显著提升

**后续步骤**：
- 完成快速修复后，可以考虑实施完整的解决方案（见 `EARLY_SHUTDOWN_ROOT_CAUSE_ANALYSIS.md`）
- 添加 bag 数据扫描和 rosbag2 状态监控
- 实现更智能的退出机制

---

**文档版本**：v1.0  
**创建日期**：2026-03-09  
**作者**：AutoMap-Pro Team  
**预计实施时间**：30 分钟（包括测试验证）
