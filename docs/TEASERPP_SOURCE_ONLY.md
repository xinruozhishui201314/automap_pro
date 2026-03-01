# TEASER++ 源码优先配置说明

## 配置概述

**配置目标：** 确保 AutoMap-Pro 始终使用从 `TEASER-plusplus-master` 源码编译的 TEASER++ 库，而非系统安装的版本。

**配置日期：** 2026-02-28

---

## 1. 为什么使用源码优先？

### 1.1 优势

| 优势 | 说明 |
|------|------|
| **版本控制** | 可以追踪和回退到特定提交版本 |
| **调试便利** | 可以直接修改源码进行调试和优化 |
| **避免版本冲突** | 避免系统版本过旧或依赖不兼容 |
| **最新特性** | 可以使用上游最新的特性和 bug 修复 |
| **一致性保证** | 确保所有环境使用相同的 TEASER++ 版本 |

### 1.2 适用场景

- 需要修改 TEASER++ 源码进行定制
- 系统安装的 TEASER++ 版本过旧或依赖不兼容
- 需要保证开发、测试、生产环境的 TEASER++ 版本一致
- 需要复现和调试 TEASER++ 相关的问题

---

## 2. 修改文件清单

### 2.1 CMakeLists.txt

#### automap_pro/CMakeLists.txt

```cmake
# 修改前
find_package(teaserpp QUIET)
if(teaserpp_FOUND)
  add_definitions(-DUSE_TEASER)
  message(STATUS "TEASER++ found")
endif()

# 修改后
# TEASER++: 优先使用工作空间源码编译版本，而非系统安装版本
# 工作空间版本通过 CMAKE_PREFIX_PATH 指定（由 run_automap.sh 设置）
# 这确保始终使用最新的源码版本，而非旧版本系统包
find_package(teaserpp QUIET NO_DEFAULT_PATH)
if(teaserpp_FOUND)
  add_definitions(-DUSE_TEASER)
  message(STATUS "TEASER++ found: ${teaserpp_INCLUDE_DIRS}")
else()
  # 如果工作空间版本不存在，尝试查找系统版本（可选）
  message(WARNING "TEASER++ not found in workspace, searching system...")
  find_package(teaserpp QUIET)
  if(teaserpp_FOUND)
    add_definitions(-DUSE_TEASER)
    message(WARNING "TEASER++ found from system (not recommended)")
  else()
    message(STATUS "TEASER++ not found, building without TEASER++ support")
  endif()
endif()
```

**关键变化：**
1. 使用 `NO_DEFAULT_PATH` 选项，禁止查找系统默认路径
2. 优先查找 `CMAKE_PREFIX_PATH` 指定的工作空间版本
3. 如果工作空间版本不存在，才回退到系统版本（并给出警告）
4. 增强日志输出，显示 TEASER++ 的安装路径

#### automap_loop_closure 库链接

```cmake
# 修改前
if(teaserpp_FOUND)
  target_link_libraries(automap_loop_closure teaserpp::teaser_registration)
endif()

# 修改后
if(teaserpp_FOUND)
  # 如果工作空间版本有提供 target，使用之；否则使用变量
  if(TARGET teaserpp::teaser_registration)
    target_link_libraries(automap_loop_closure teaserpp::teaser_registration)
  elseif(teaserpp_LIBRARIES)
    target_link_libraries(automap_loop_closure ${teaserpp_LIBRARIES})
  else()
    target_link_libraries(automap_loop_closure teaserpp)
  endif()
  message(STATUS "Automap loop closure linking TEASER++: ${teaserpp_INCLUDE_DIRS}")
endif()
```

**关键变化：**
1. 支持多种链接方式（target、变量、库名）
2. 增强日志输出，显示实际使用的 TEASER++ 路径

### 2.2 run_automap.sh

#### 编译流程

```bash
# 修改前
if [ -d /root/mapping/TEASER-plusplus-master ]; then
  echo '========================================'
  echo '编译 TEASER++ (上游)'
  echo '========================================'
  mkdir -p /root/automap_ws/install/teaserpp
  cd /root/mapping/TEASER-plusplus-master && mkdir -p build && cd build
  cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/root/automap_ws/install/teaserpp -DBUILD_TEASER_FPFH=ON ..
  make -j$(nproc) && make install
  cd /root/automap_ws
fi

# 修改后
if [ -d /root/mapping/TEASER-plusplus-master ]; then
  echo '========================================'
  echo '编译 TEASER++ (上游源码，优先于系统版本）'
  echo '========================================'
  mkdir -p /root/automap_ws/install/teaserpp
  cd /root/mapping/TEASER-plusplus-master && mkdir -p build && cd build
  # 从源码编译 TEASER++，安装到工作空间
  # 使用 NO_DEFAULT_PATH 确保不使用系统版本
  cmake -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/root/automap_ws/install/teaserpp \
        -DBUILD_TEASER_FPFH=ON \
        -DBUILD_TESTS=OFF \
        -DBUILD_PYTHON_BINDINGS=OFF \
        -DCMAKE_PREFIX_PATH="" \
        ..
  make -j\$(nproc) && make install
  cd /root/automap_ws
  # 设置 CMAKE_PREFIX_PATH，确保工作空间版本优先
  export CMAKE_PREFIX_PATH=/root/automap_ws/install/teaserpp:\$CMAKE_PREFIX_PATH
  echo "✓ TEASER++ 源码编译完成，安装到工作空间"
else
  echo "========================================"
  echo "TEASER-plusplus-master 未找到，将使用系统安装的 TEASER++（如果存在）"
  echo "========================================"
fi
```

**关键变化：**
1. 增加日志，明确说明使用源码编译
2. 添加 `-DBUILD_TESTS=OFF` 和 `-DBUILD_PYTHON_BINDINGS=OFF` 加快编译
3. 添加 `-DCMAKE_PREFIX_PATH=""` 避免意外使用系统版本
4. 显式设置 `CMAKE_PREFIX_PATH`，确保后续查找工作空间版本
5. 如果源码不存在，给出明确提示

---

## 3. CMake 查找优先级

### 3.1 默认查找顺序（修改前）

```
CMAKE_PREFIX_PATH
CMAKE_SYSTEM_PREFIX_PATH
/usr/local
/usr/lib
/usr/lib/x86_64-linux-gnu
...
```

### 3.2 源码优先查找顺序（修改后）

```
CMAKE_PREFIX_PATH (/root/automap_ws/install/teaserpp)
(跳过 CMAKE_SYSTEM_PREFIX_PATH 等系统路径)
(仅在 CMAKE_PREFIX_PATH 未找到时回退到系统路径)
```

### 3.3 环境变量说明

```bash
# 工作空间 TEASER++ 安装路径
CMAKE_PREFIX_PATH=/root/automap_ws/install/teaserpp

# CMake 会在此路径查找：
# /root/automap_ws/install/teaserpp/lib/cmake/teaserpp/teaserppConfig.cmake
# 或
# /root/automap_ws/install/teaserpp/lib/cmake/teaserpp/teaserpp-config.cmake
```

---

## 4. 验证方法

### 4.1 编译时验证

```bash
# 运行编译
bash run_automap.sh --build-only

# 查看编译日志，查找以下信息：
# "TEASER++ found: /root/automap_ws/install/teaserpp"
# "✓ TEASER++ 源码编译完成，安装到工作空间"
# "Automap loop closure linking TEASER++: /root/automap_ws/install/teaserpp/include"
```

**预期输出（使用源码）：**
```
========================================
编译 TEASER++ (上游源码，优先于系统版本）
========================================
开始编译 TEASER++...
✓ TEASER++ 源码编译完成，安装到工作空间
========================================
编译 automap_pro
================================--------
-- TEASER++ found: /root/automap_ws/install/teaserpp
-- Automap loop closure linking TEASER++: /root/automap_ws/install/teaserpp/include
```

**警告输出（使用系统版本）：**
```
========================================
TEASER-plusplus-master 未找到，将使用系统安装的 TEASER++（如果存在）
========================================
-- TEASER++ not found in workspace, searching system...
-- TEASER++ found: /usr/local
-- WARNING: TEASER++ found from system (not recommended)
```

### 4.2 运行时验证

```bash
# 进入容器
docker exec -it $(docker ps -q) /bin/bash

# 查看链接的 TEASER++ 库
ldd /root/automap_ws/install/automap_pro/lib/libautomap_loop_closure.so | grep teaser

# 预期输出：
# libteaserpp.so => /root/automap_ws/install/teaserpp/lib/libteaserpp.so (0x...)
# (而非 /usr/local/lib/libteaserpp.so 或 /usr/lib/libteaserpp.so)

# 查看 CMake 缓存
cd /root/automap_ws/build/automap_pro
cat CMakeCache.txt | grep teaserpp

# 预期输出：
# teaserpp_INCLUDE_DIRS:INTERNAL=/root/automap_ws/install/teaserpp/include
# teaserpp_LIBRARIES:INTERNAL=/root/automap_ws/install/teaserpp/lib/libteaserpp.so
```

### 4.3 功能验证

```bash
# 启动系统
bash run_automap.sh

# 触发回环检测
# 在另一个终端
docker exec -it $(docker ps -q) /bin/bash
source install/setup.bash
ros2 service call /automap/trigger_optimize automap_pro/srv/TriggerOptimize \
  "{full_optimization: true, max_iterations: 100}"

# 查看日志，确认 TEASER++ 正常工作
ros2 node info /automap_system

# 预期：回环检测正常，无 TEASER++ 相关错误
```

---

## 5. 故障排查

### 5.1 TEASER++ 未找到

**症状：**
```
-- TEASER++ not found, building without TEASER++ support
```

**可能原因：**
- `TEASER-plusplus-master` 目录不存在
- TEASER++ 编译失败
- 系统中也未安装 TEASER++

**解决方法：**
```bash
# 检查源码目录
ls -la TEASER-plusplus-master/

# 如果不存在，克隆或下载
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git TEASER-plusplus-master

# 重新编译
bash run_automap.sh --clean
```

### 5.2 链接到系统版本

**症状：**
```bash
ldd ... | grep teaser
# libteaserpp.so => /usr/local/lib/libteaserpp.so
```

**可能原因：**
- 工作空间编译失败，回退到系统版本
- `CMAKE_PREFIX_PATH` 未正确设置

**解决方法：**
```bash
# 清理编译产物
bash scripts/clean.sh --workspace

# 重新编译
bash run_automap.sh --build-only

# 检查 CMAKE 缓存
cd automap_ws/build/automap_pro
cat CMakeCache.txt | grep teaserpp

# 确保 teaserpp_INCLUDE_DIRS 指向工作空间
```

### 5.3 TEASER++ 编译失败

**症状：**
```
-- TEASER++ 源码编译完成，安装到工作空间
-- TEASER++ not found in workspace, searching system...
```

**可能原因：**
- TEASER++ 编译过程有错误
- `make install` 失败

**解决方法：**
```bash
# 手动进入容器编译
docker run -it --rm --gpus all --net=host \
    -v $(pwd)/automap_ws:/root/automap_ws \
    -v $(pwd)/TEASER-plusplus-master:/root/TEASER-plusplus-master \
    automap-env:humble /bin/bash

# 手动编译 TEASER++
cd /root/TEASER-plusplus-master
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/root/automap_ws/install/teaserpp \
      -DBUILD_TEASER_FPFH=ON \
      -DBUILD_TESTS=OFF \
      -DBUILD_PYTHON_BINDINGS=OFF \
      ..
make -j$(nproc)
make install

# 检查安装结果
ls -la /root/automap_ws/install/teaserpp/
```

---

## 6. 版本管理

### 6.1 查看当前版本

```bash
# 查看工作空间 TEASER++ 版本
cd TEASER-plusplus-master
git log -1 --oneline

# 查看编译安装的版本
cat automap_ws/install/teaserpp/include/teaser/registration.h | grep "TEASER"
```

### 6.2 切换到特定版本

```bash
# 切换到指定 tag 或 commit
cd TEASER-plusplus-master
git fetch --tags
git checkout v1.1.0  # 或特定 commit hash

# 清理并重新编译
rm -rf build automap_ws/install/teaserpp
bash run_automap.sh --build-only
```

### 6.3 更新到最新版本

```bash
# 拉取最新代码
cd TEASER-plusplus-master
git pull origin master

# 清理并重新编译
rm -rf build automap_ws/install/teaserpp
bash run_automap.sh --build-only
```

---

## 7. 性能优化

### 7.1 加速编译

```bash
# 使用 ccache 加速
export CC="ccache gcc"
export CXX="ccache g++"

# 增加 -j 参数
make -j$(nproc)  # 使用所有 CPU 核心

# 跳过测试和 Python binding
cmake ... -DBUILD_TESTS=OFF -DBUILD_PYTHON_BINDINGS=OFF
```

### 7.2 减小镜像大小

```bash
# 编译完成后清理中间文件
cd TEASER-plusplus-master/build
rm -rf _deps CMakeFiles

# Dockerfile 中可以添加
RUN rm -rf /tmp/TEASER-plusplus
```

---

## 8. Docker 镜像说明

### 8.1 镜像中的 TEASER++

Docker 镜像（`automap-env:humble`）中已包含：
- **系统安装的 TEASER++**：安装到 `/usr/local`
- **预编译的源码**：`docker/deps/TEASER-plusplus`

### 8.2 工作空间 vs 系统

```
优先级：
1. 工作空间源码编译版本 (/root/automap_ws/install/teaserpp)
2. 系统安装版本 (/usr/local) [仅在源码不存在时]
```

**注意：** 即使系统中有 TEASER++，只要 `TEASER-plusplus-master` 存在，就会使用源码编译的版本。

---

## 9. 与其他配置的集成

### 9.1 与 Docker 镜像的集成

Docker 镜像构建脚本已包含 TEASER++ 系统版本安装：
```dockerfile
# docker/dockerfile
COPY ./deps/TEASER-plusplus /tmp/TEASER-plusplus
RUN cd /tmp/TEASER-plusplus && ... && make install
```

**工作空间源码编译会覆盖系统版本：**
```bash
# run_automap.sh
cmake ... -DCMAKE_INSTALL_PREFIX=/root/automap_ws/install/teaserpp ...
export CMAKE_PREFIX_PATH=/root/automap_ws/install/teaserpp:$CMAKE_PREFIX_PATH
```

### 9.2 与 OverlapTransformer 的集成

OverlapTransformer 和 TEASER++ 独立工作：
- **OverlapTransformer**：Python 服务，提供描述子计算
- **TEASER++**：C++ 库，提供回环精匹配

两者可以独立切换：
```bash
# 使用源码 TEASER++ + 自研描述子
bash run_automap.sh  # 默认

# 使用源码 TEASER++ + OverlapTransformer 描述子
bash run_automap.sh --external-overlap

# 使用系统 TEASER++（如果源码不存在）+ OverlapTransformer
rm -rf TEASER-plusplus-master
bash run_automap.sh --external-overlap
```

---

## 10. 最佳实践

### 10.1 开发环境

```bash
# 1. 使用源码 TEASER++
bash run_automap.sh --build-only

# 2. 修改 TEASER++ 源码
vim TEASER-plusplus-master/teaser/registration.cpp

# 3. 清理并重新编译
rm -rf automap_ws/install/teaserpp
bash run_automap.sh --build-only

# 4. 运行测试
bash run_automap.sh
```

### 10.2 生产环境

```bash
# 1. 固定 TEASER++ 版本
cd TEASER-plusplus-master
git checkout v1.1.0

# 2. 编译并验证
bash run_automap.sh --build-only

# 3. 保存编译产物
tar -czf automap_ws_install.tar.gz automap_ws/install/

# 4. 部署时直接使用（跳过编译）
tar -xzf automap_ws_install.tar.gz
bash run_automap.sh --run-only
```

### 10.3 CI/CD 环境

```yaml
# .github/workflows/build.yml
- name: Build TEASER++ from source
  run: |
    cd TEASER-plusplus-master
    mkdir -p build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX=${{ github.workspace }}/automap_ws/install/teaserpp \
          -DBUILD_TEASER_FPFH=ON \
          -DBUILD_TESTS=OFF \
          -DBUILD_PYTHON_BINDINGS=OFF \
          ..
    make -j$(nproc)
    make install

- name: Build AutoMap-Pro
  run: |
    export CMAKE_PREFIX_PATH=${{ github.workspace }}/automap_ws/install/teaserpp
    bash run_automap.sh --build-only
```

---

## 11. 常见问题

### Q1：为什么使用源码而不是系统版本？

**A：** 源码版本提供：
- 更好的版本控制
- 调试和定制能力
- 避免系统版本冲突
- 确保环境一致性

### Q2：是否可以强制使用系统版本？

**A：** 可以，删除或重命名 `TEASER-plusplus-master`：
```bash
mv TEASER-plusplus-master TEASER-plusplus-master.disabled
bash run_automap.sh --build-only
```

### Q3：如何验证实际使用的 TEASER++ 版本？

**A：** 使用 `ldd` 查看：
```bash
ldd automap_ws/install/automap_pro/lib/libautomap_loop_closure.so | grep teaser
```

### Q4：修改 TEASER++ 源码后需要重新编译吗？

**A：** 需要重新编译 TEASER++ 和 AutoMap-Pro：
```bash
cd TEASER-plusplus-master/build
make install
cd ../../automap_ws
colcon build --packages-select automap_pro
```

### Q5：TEASER++ 编译很慢怎么办？

**A：** 可以：
- 使用 `ccache` 加速
- 跳过测试和 Python binding
- 增加并行编译参数

---

## 12. 相关文档

- [四项目融合设计文档](./FOUR_PROJECTS_FUSION_DESIGN.md)
- [前端默认变更说明](./DEFAULT_FRONTEND_CHANGE.md)
- [TEASER++ 官方文档](https://github.com/MIT-SPARK/TEASER-plusplus)

---

## 13. 联系方式

如有问题或建议，请联系：
- 项目地址：[GitHub Repository]
- 文档版本：v1.0
- 更新日期：2026-02-28

---

**Made with ❤️ by AutoMap-Pro Team**
