# TEASER-plusplus 使用本地依赖库修改说明

## 0. Executive Summary

本次修改解决了 TEASER-plusplus 编译时因网络问题无法下载依赖库(tinyply, pmc)的问题,通过配置 CMake 使用工程内已有的本地源码,实现了离线编译。

## 1. 背景&目标

### 问题
- TEASER-plusplus 使用 `FetchContent` 从 GitHub 下载依赖库 pmc 和 tinyply
- 网络连接超时导致构建失败
- 工程中已有这些依赖库在 `thrid_party/` 目录

### 目标
- 修改 TEASER-plusplus 和 run_automap.sh,使其使用本地依赖源码
- 支持离线编译
- 提高构建可靠性和速度

## 2. 方案设计

### 方案对比

| 方案 | 优点 | 缺点 | 选择 |
|------|------|------|------|
| 1. 硬编码本地路径 | 简单直接 | 灵活性差,路径固定 | ❌ |
| 2. 使用 FETCHCONTENT_SOURCE_DIR | CMake 标准做法,灵活 | 需要修改 CMakeLists | ✅ |
| 3. 完全禁用 FetchContent | 最简单 | 需要手动管理依赖 | ❌ |

### 选用方案
使用 CMake `FetchContent_SOURCE_DIR_<name>` 变量,允许在配置时指定本地源码路径,保持与 FetchContent 机制的兼容性。

## 3. 变更清单

### 3.1 文件修改

| 文件 | 变更类型 | 说明 |
|------|---------|------|
| `TEASER-plusplus-master/teaser/CMakeLists.txt` | 修改 | 恢复 FetchContent 声明,支持通过参数指定本地源码 |
| `run_automap.sh` | 修改 | 添加 pmc 和 tinyply 本地源码检查和 CMAKE 参数传递 |
| `thrid_party/tinyply/` | 无变更 | 已存在的 tinyply 源码 |
| `thrid_party/pmc-master/` | 无变更 | 已存在的 pmc 源码 |

## 4. 代码与配置

### 4.1 TEASER-plusplus-master/teaser/CMakeLists.txt

修改内容:恢复标准的 FetchContent 声明,CMake 会自动检查 `FETCHCONTENT_SOURCE_DIR_<name>` 变量

```cmake
include(FetchContent)

# 使用 FetchContent 声明 pmc 和 tinyply
# 如果通过 CMAKE 参数指定了本地源码路径,使用本地路径
FetchContent_Declare(pmc
    GIT_REPOSITORY https://github.com/jingnanshi/pmc.git
)

FetchContent_Declare(tinyply
    GIT_REPOSITORY https://github.com/ddiakopoulos/tinyply.git
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
)

# Populat pmc
if (NOT pmc_POPULATED)
  FetchContent_Populate(pmc)
  set(PMC_BUILD_SHARED OFF CACHE INTERNAL "")
  add_subdirectory(${pmc_SOURCE_DIR} ${pmc_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

# Populat tinyply (仅头文件库,无需编译)
if (NOT tinyply_POPULATED)
  FetchContent_Populate(tinyply)
endif()
```

### 4.2 run_automap.sh

在 TEASER++ 编译部分添加本地源码检查和 CMAKE 参数:

```bash
# 从源码编译 TEASER++，安装到工作空间
# 使用本地的 pmc 和 tinyply 源码
PMC_SRC=/root/mapping/thrid_party/pmc-master
TINYPLY_SRC=/root/mapping/thrid_party/tinyply
if [ ! -d "${PMC_SRC}" ]; then
  echo "✗ 错误: PMC 源码不存在于 ${PMC_SRC}"
  exit 1
fi
if [ ! -d "${TINYPLY_SRC}" ]; then
  echo "✗ 错误: tinyply 源码不存在于 ${TINYPLY_SRC}"
  exit 1
fi
echo "使用本地 PMC 源码: ${PMC_SRC}"
echo "使用本地 tinyply 源码: ${TINYPLY_SRC}"
# 使用 FETCHCONTENT_SOURCE_DIR 指向本地源码
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/root/automap_ws/install/teaserpp \
      -DBUILD_TEASER_FPFH=ON \
      -DBUILD_TESTS=OFF \
      -DBUILD_PYTHON_BINDINGS=OFF \
      -DFETCHCONTENT_SOURCE_DIR_PMC="${PMC_SRC}" \
      -DFETCHCONTENT_SOURCE_DIR_TINYPLY="${TINYPLY_SRC}" \
      -DCMAKE_PREFIX_PATH="" \
      ..
```

## 5. 编译/部署/运行说明

### 5.1 环境要求

- Docker 镜像: `automap-env:humble`
- CMake 版本: 3.24+
- 依赖库源码路径:
  - `thrid_party/pmc-master/`
  - `thrid_party/tinyply/`

### 5.2 编译步骤

```bash
# 1. 进入工作空间
cd /home/wqs/Documents/github/mapping

# 2. 运行编译脚本
./run_automap.sh --build

# 或仅清理构建产物
./run_automap.sh --clean
```

### 5.3 手动编译 TEASER++

```bash
# 在容器内执行
cd /root/automap_ws
mkdir -p build_teaser
cd build_teaser

cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/root/automap_ws/install/teaserpp \
      -DBUILD_TEASER_FPFH=ON \
      -DBUILD_TESTS=OFF \
      -DBUILD_PYTHON_BINDINGS=OFF \
      -DFETCHCONTENT_SOURCE_DIR_PMC="/root/mapping/thrid_party/pmc-master" \
      -DFETCHCONTENT_SOURCE_DIR_TINYPLY="/root/mapping/thrid_party/tinyply" \
      /root/mapping/TEASER-plusplus-master

make -j$(nproc)
make install
```

## 6. 验证与回归测试

### 6.1 验证清单

- [ ] TEASER++ 编译成功,无网络错误
- [ ] CMake 输出显示 "Using local PMC 源码" 和 "使用本地 tinyply 源码"
- [ ] 编译产物正确安装到 `/root/automap_ws/install/teaserpp/`
- [ ] automap_pro 能正确链接 TEASER++ 库
- [ ] 最终程序能正常运行

### 6.2 预期输出

```
使用本地 PMC 源码: /root/mapping/thrid_party/pmc-master
使用本地 tinyply 源码: /root/mapping/thrid_party/tinyply
...
-- Found tinyply: /root/mapping/thrid_party/tinyply
...
Scanning dependencies of target teaser_io
[ 10%] Building CXX object teaser/CMakeFiles/teaser_io.dir/src/ply_io.cc.o
...
[100%] Built target teaser_io
✓ TEASER++ 源码编译完成，安装到工作空间
```

## 7. 风险与回滚方案

### 7.1 风险清单

| 风险 | 影响 | 概率 | 缓解策略 |
|------|------|------|---------|
| 本地源码路径不存在 | 编译失败 | 低 | 脚本中添加存在性检查 |
| 依赖库版本不匹配 | 运行时错误 | 低 | 使用原始上游版本 |
| CMake 版本不兼容 | FetchContent 失败 | 低 | 镜像中已安装 CMake 3.24 |

### 7.2 回滚方案

如果修改导致问题,可以:
1. 恢复备份: `cp run_automap.sh.backup run_automap.sh`
2. 恢复 CMakeLists: 使用 git 或手动回退
3. 移除 CMAKE 参数,恢复网络下载

## 8. 后续演进路线图

### MVP (当前)
- ✅ 使用本地 pmc 和 tinyply 源码
- ✅ 支持 `FETCHCONTENT_SOURCE_DIR` 参数

### V1 (短期优化)
- 将所有依赖库(third-party)统一管理
- 支持依赖库版本锁定和更新机制
- 添加依赖库完整性校验(SHA256)

### V2 (长期演进)
- 考虑使用 CPM.cmake 或 vcpkg 等现代依赖管理工具
- 实现依赖库的自动下载和缓存机制
- 支持依赖库的预编译二进制包

## 9. 参考资料

- CMake FetchContent 文档: https://cmake.org/cmake/help/latest/module/FetchContent.html
- FETCHCONTENT_SOURCE_DIR 用法: https://cmake.org/cmake/help/latest/variable/FETCHCONTENT_SOURCE_DIR_UPPERCASE_NAME.html
- TEASER-plusplus GitHub: https://github.com/MIT-SPARK/TEASER-plusplus
