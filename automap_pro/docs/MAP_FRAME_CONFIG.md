# 地图坐标系 .cfg 读取接口与调用位置建议

## 1. 函数签名

```cpp
#include "automap_pro/core/map_frame_config.h"

namespace automap_pro::MapFrameConfig {

// 读取结果：供其他模块使用的 ENU 原点
struct ENUOriginResult {
    double latitude_deg;   // 纬度（度）
    double longitude_deg;  // 经度（度）
    double altitude_m;     // 高度（米）
    bool   valid;          // 解析成功且范围合法为 true
};

// 从 .cfg 读取 ENU 原点
std::optional<ENUOriginResult> read(const std::string& cfg_path);

// 写入 .cfg（GPS 首次设原点时调用，此处仅列签名）
bool write(const std::string& cfg_path,
           double latitude_deg, double longitude_deg, double altitude_m);
}
```

- **read**：解析 `coordinate_system`、`latitude`、`longitude`、`altitude`，校验范围后返回；文件不存在或格式错误返回 `std::nullopt`。
- 路径建议统一从 **ConfigManager::mapFrameConfigPath()** 获取，与写入端一致。

---

## 2. 调用位置建议

| 模块 | 调用时机 | 用途 |
|------|----------|------|
| **AutoMapSystem** | 节点 init 或首次需要地图原点时 | 预读 ENU 原点，注入 MapExporter / 其他需要 WGS84 原点的模块，避免等 GPS 才导出。 |
| **MapExporter** | export 前若自身无 origin（如未配 YAML origin） | 用 `read(mapFrameConfigPath())` 得到 `origin_latitude/longitude/altitude`，用于 ENU→WGS84、KML 等。 |
| **GPSManager（可选）** | 构造或 init 时 | 若存在 .cfg 且有效，可把读到的 lat/lon/alt 作为初始 ENU 原点，无 GPS 时也能有一致坐标系。 |
| **RVizPublisher / 可视化** | 需要在地图上标 WGS84 或显示经纬度时 | 读 .cfg 做 ENU↔WGS84 转换。 |
| **离线/回放或多会话** | 加载历史会话前 | 读 .cfg 确认或设置当前会话的地图坐标系，保证与后端一致。 |

---

## 3. 使用示例

```cpp
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/map_frame_config.h"

void useMapFrameOrigin() {
    std::string path = ConfigManager::instance().mapFrameConfigPath();
    auto origin = MapFrameConfig::read(path);
    if (!origin || !origin->valid)
        return;  // 文件不存在或尚未写入（如尚未收到 GPS）
    double lat = origin->latitude_deg;
    double lon = origin->longitude_deg;
    double alt = origin->altitude_m;
    // 用于 ENU↔WGS84、导出、可视化等
}
```

---

## 4. 数据流简述

```
写入：首个有效 GPS → GPSManager::addGPSMeasurement (call_once)
        → MapFrameConfig::write(cfg_path, lat, lon, alt)

读取：任意模块在需要 ENU 原点时
        → path = ConfigManager::instance().mapFrameConfigPath()
        → MapFrameConfig::read(path) → optional<ENUOriginResult>
```

同一进程内写入后立即可读；跨进程/重启后依赖该 .cfg 文件存在且未被删改。
