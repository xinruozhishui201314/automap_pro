# Open3D Docker 构建失败分析：HTTP/2 Stream Error

## 0. Executive Summary

| 项目 | 说明 |
|------|------|
| **现象** | Docker 构建 Open3D 时，CMake ExternalProject 从 GitHub 下载多个第三方依赖失败 |
| **错误码** | curl status_code: **92**，status_string: **"Stream error in the HTTP/2 framing layer"** |
| **根因** | 容器内 curl/libcurl 使用 **HTTP/2** 连接 GitHub，在部分网络/代理/Docker 环境下流未正常关闭，触发已知兼容性问题 |
| **推荐修复** | 方案 A：构建阶段强制使用 **HTTP/1.1**（curl 包装）；方案 B：**预下载** 3rdparty 到 `3rdparty_downloads` 并在 CMake 中优先使用 `file://`（离线可靠） |
| **风险** | 仅改构建环境，不影响运行时；回滚即去掉 curl 包装或恢复未打补丁的 Open3D 源码 |

---

## 1. 背景与现象

构建命令（节选）：

```bash
RUN cd /tmp/Open3D && mkdir build && cd build && \
    cmake .. -D... && make -j$(nproc) && make install
```

失败发生在 **make** 阶段：CMake 生成的 ExternalProject 目标在 **download** 步骤中通过 `file(DOWNLOAD ...)` 拉取 GitHub 上的 tarball 时全部失败。

### 1.1 失败依赖列表（来自日志）

| 依赖 | 下载 URL（摘要） | 错误 |
|------|------------------|------|
| ext_qhull | qhull/archive/refs/tags/v8.0.2.tar.gz | 92, HTTP/2 stream not closed cleanly |
| ext_nanoflann | nanoflann/archive/refs/tags/v1.5.0.tar.gz | 同上 |
| ext_stdgpu | stdgpu/archive/e10f6f3...e6.tar.gz | 同上 |
| ext_embree | embree/archive/refs/tags/v3.13.3.tar.gz | 同上 |
| ext_parallelstl | oneDPL/archive/refs/tags/20190522.tar.gz | 同上 |
| ext_msgpack-c | msgpack-c/releases/.../msgpack-3.3.0.tar.gz | 同上 |
| ext_poisson | Open3D-PoissonRecon/archive/90f3f06...0.tar.gz | 同上 |
| ext_tinygltf | tinygltf/archive/72f4a55...07.tar.gz | 同上 |
| ext_tinyobjloader | tinyobjloader/archive/refs/tags/v2.0.0rc8.tar.gz | 同上 |

日志中还有：`SSL certificate verify result: unable to get local issuer certificate (20), continuing anyway`，说明证书链在容器内不完整，但与本次 92 错误的直接原因主要是 **HTTP/2 流关闭异常**。

---

## 2. 根因分析

### 2.1 技术链路

- **ALPN: server accepted h2** → 连接使用 **HTTP/2**。
- **HTTP/2 stream 1 was not closed cleanly before end of the underlying stream** → 服务端（或中间代理）关闭流的方式与当前 **libcurl/curl 7.83.1** 的 HTTP/2 实现不兼容，导致 libcurl 报错 92（CURLE_HTTP2_STREAM）。
- CMake 的 `file(DOWNLOAD)` 使用 **libcurl**（非系统 `curl` 可执行文件），因此：
  - 仅替换 PATH 中的 `curl` 为带 `--http1.1` 的包装脚本，**不能**解决由 libcurl 发起的下载；
  - 若构建过程中有脚本或子进程显式调用 `curl`，则 curl 包装对这部分有效。

### 2.2 为何在 Docker 里易现

- 网络路径更长（NAT、代理、企业防火墙等），容易放大 HTTP/2 帧/流关闭时序问题。
- 基础镜像中 curl/libcurl 版本较旧，与当前 GitHub 边缘节点行为组合后易触发已知 bug。

---

## 3. 方案对比与取舍

| 方案 | 做法 | 优点 | 缺点 |
|------|------|------|------|
| **A. 强制 HTTP/1.1** | 构建 Open3D 时用 curl 包装（`curl --http1.1`）并确保该 curl 在 PATH 优先 | 改 Dockerfile 即可，无需改 Open3D 源码 | 对 CMake `file(DOWNLOAD)` 无效（用 libcurl）；仅对显式调用 `curl` 的步骤有效 |
| **B. 预下载 + file:// 优先** | 宿主机预下载 9 个 tarball 到 `deps/Open3D/3rdparty_downloads/<name>/`，并在各 3rdparty 的 CMake 中在现有 URL 前增加 `file://${OPEN3D_THIRD_PARTY_DOWNLOAD_DIR}/<name>/<file>` | 构建完全不依赖外网，可复现、可离线 | 需维护下载脚本和 9 处 CMake 小补丁 |
| **C. 升级 libcurl** | 在镜像中安装更新版本 libcurl/curl（PPA/源码） | 从根上缓解 HTTP/2 兼容性 | 需改基础镜像或增加构建步骤，影响面大 |
| **D. 仅预下载不补丁** | 只把文件放到 3rdparty_downloads，不改 CMake | 无 CMake 改动 | ExternalProject 仍会执行 download 步骤并覆盖/重下，无法规避 92 |

**已实现**：采用 **方案 B**（预下载 + 在 9 个 3rdparty cmake 中增加 `file://` 优先 URL）。构建前在宿主机执行 `./scripts/download_deps.sh` 可预下载上述 9 个 tarball 到 `deps/Open3D/3rdparty_downloads/`，Docker 构建时 CMake 会优先使用本地文件，避免 HTTP/2 92。

---

## 4. 变更清单（已实现）

| 文件/位置 | 变更类型 | 说明 |
|-----------|----------|------|
| `docker/dockerfile` | 修改 | 注释中说明：若遇 HTTP/2 92，请先执行 `./scripts/download_deps.sh` |
| `docker/scripts/download_deps.sh` | 修改 | 增加 `download_open3d_3rdparty()`，将 9 个 tarball 下载到 `deps/Open3D/3rdparty_downloads/<name>/`（使用 `--http1.1`） |
| `docker/deps/Open3D/3rdparty/{qhull,nanoflann,stdgpu,embree,parallelstl,msgpack,possionrecon,tinygltf,tinyobjloader}/*.cmake` | 修改 | 在对应 `ExternalProject_Add` 的 URL 前增加一条 `file://${OPEN3D_THIRD_PARTY_DOWNLOAD_DIR}/<name>/<file>`，CMake 会优先使用本地文件 |

---

## 5. 验证与回滚

- **验证**：在同一构建环境重新执行 `docker build`，确认 9 个 ext_* 的 download 步骤均成功，Open3D 编译安装完成。
- **回滚**：去掉 curl 包装逻辑；若采用方案 B，恢复 Open3D 3rdparty 的 CMake 原始 URL 并删除预下载文件。

---

## 6. 参考资料

- curl error 92: `CURLE_HTTP2_STREAM` — HTTP/2 framing layer 错误。
- 日志中 user-agent: `curl/7.83.1`，ALPN `h2` → 确认为 HTTP/2 连接。
- Open3D 3rdparty 使用 `OPEN3D_THIRD_PARTY_DOWNLOAD_DIR` 与 `DOWNLOAD_DIR`，支持预下载后通过 `file://` 优先使用本地文件。
