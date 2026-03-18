# LibTorch + OverlapTransformer 模型加载与运行说明

**目标**：在已安装本地 LibTorch、已有 `pretrained_overlap_transformer.pth.tar` 的前提下，正确加载并运行 OverlapTransformer，使回环模块能使用 OT 描述子。

---

## 0. Executive Summary

| 项目 | 说明 |
|------|------|
| **收益** | 回环可选用 OverlapTransformer 描述子，与 ScanContext 互补，提高召回。 |
| **前提** | LibTorch 已安装到本地；拥有 `pretrained_overlap_transformer.pth.tar`。 |
| **必须步骤** | ① 编译前设置 `LIBTORCH_HOME`（或 `Torch_DIR`）；② 将 `.pth.tar` 转为 TorchScript `.pt`；③ 配置中 `model_path` 指向该 `.pt`。 |
| **验证** | 运行后日志出现 “Model loaded” 且无 “LibTorch not available”。 |

---

## 1. 编译前：指定本地 LibTorch

C++ 回环模块通过 `find_package(Torch)` 使用 LibTorch。**支持将 LibTorch 放在工程本地文件夹**，CMake 会自动按以下顺序查找（无需设环境变量即可用本地路径）：

1. **环境变量** `LIBTORCH_HOME`（若已设置且其下存在 `share/cmake/Torch`）
2. **CMake 参数** `-DTorch_DIR=/path/to/libtorch/share/cmake/Torch`
3. **本地目录**（按顺序尝试，第一个存在 `share/cmake/Torch` 的即用）：
   - 仓库根目录下的 `libtorch/`（与 `automap_pro` 同级）
   - `automap_pro/thrid_party/libtorch/`
   - 工作空间或包内的 `libtorch/`
   - `/opt/libtorch`、`/usr/local/libtorch`

**推荐：直接使用本地文件夹**（解压官方 LibTorch 到仓库内即可）：

```bash
# 将下载的 libtorch 解压到仓库根目录或 automap_pro/thrid_party/
# 例如: 仓库根/libtorch/ 或 仓库根/automap_pro/thrid_party/libtorch/
cd /path/to/automap_ws && colcon build --packages-select automap_pro
```

编译时可在 **build.log** 中看到 LibTorch 状态，例如：
- `========== [automap_pro] LibTorch (OverlapTransformer) ==========`
- `[automap_pro] LibTorch source: local path = /path/to/libtorch`
- `[automap_pro] LibTorch BUILD STATUS: FOUND -> USE_TORCH=1` 或 `NOT FOUND -> USE_TORCH=0`

**方式二**：环境变量（适合未把 libtorch 放在工程内时）：

```bash
export LIBTORCH_HOME=/path/to/libtorch   # 解压后的 libtorch 目录（其下有 share/cmake/Torch）
cd /path/to/automap_ws && colcon build --packages-select automap_pro
```

**方式三**：直接传 CMake 变量：

```bash
colcon build --packages-select automap_pro --cmake-args -DTorch_DIR=/path/to/libtorch/share/cmake/Torch
```

**方式四（推荐，与 GTSAM 一致）**：使用 **install_deps**，一次安装、工程编译不重复下载/编译 LibTorch：

- 在容器内执行 `scripts/build_inside_container.sh` 时，若 `install_deps/libtorch` 不存在，会**自动下载**预编译 LibTorch（默认 CPU 2.1.2）并解压到 `install_deps/libtorch`；已存在则跳过。
- 编译时通过 `LIBTORCH_HOME` 或 CMake 候选路径 `install_deps/libtorch` 自动找到，无需每次设置。
- 运行时 `run_automap.sh` 会自动把 `install_deps/libtorch/lib` 加入 `LD_LIBRARY_PATH`。
- 可选环境变量：
  - `LIBTORCH_SKIP_DOWNLOAD=1`：不自动下载，仅使用已有 `install_deps/libtorch`（需自行放置或解压）。
  - `LIBTORCH_URL=<url>`：覆盖默认下载地址（例如 CUDA 版本：`https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.1.2%2Bcu118.zip`）。

---

## 2. 运行前：将 .pth.tar 转为 TorchScript .pt

C++ 端使用 `torch::jit::load(model_path)`，**只支持 TorchScript 格式（.pt）**，不能直接加载 PyTorch 检查点 `.pth.tar`。必须先做一次转换。

**依赖**：转换脚本需 Python 3 与 PyTorch（`pip install torch`）。无 GPU 也可用 CPU 做 trace。**宿主机未装 PyTorch 时**，可用 Docker 在容器内执行转换（见 2.2）。

### 2.1 使用仓库内脚本（宿主机已装 PyTorch）

在**源码根目录**或 **automap_pro** 下执行：

```bash
# 从 automap_pro 目录
cd /path/to/automap_pro
python3 scripts/convert_ot_pth_to_torchscript.py \
  models/pretrained_overlap_transformer.pth.tar \
  models/overlapTransformer.pt
```

- 第一个参数：现有的 `.pth.tar` 路径。  
- 第二个参数（可选）：输出的 `.pt` 路径；不写则默认 `automap_pro/models/overlapTransformer.pt`。

脚本会加载 `featureExtracter(64, 900, 1)`（KITTI 尺寸），trace 后保存为 TorchScript。若为 Haomo 等 32 高输入，需在脚本内把 `height=64` 改为 `32` 并重新转换。

### 2.2 在 Docker 容器内转换（宿主机无 PyTorch 时推荐）

宿主机未安装 PyTorch 时，可用工程 Docker 镜像在容器内执行转换（镜像内已具备 PyTorch 等环境）：

```bash
# 在仓库根目录执行（与 run_full_mapping_docker.sh 同目录）
cd /path/to/automap_pro   # 仓库根，不是 automap_pro 子目录
./convert_ot_model_in_docker.sh
```

- 默认：读 `automap_pro/models/pretrained_overlap_transformer.pth.tar`，写 `automap_pro/models/overlapTransformer.pt`。  
- 自定义路径：`./convert_ot_model_in_docker.sh /path/to/xxx.pth.tar /path/to/out.pt`（路径为宿主机路径，脚本会挂载仓库并映射到容器内）。  
- 使用镜像 `automap-env:humble`（可通过 `IMAGE_NAME=xxx ./convert_ot_model_in_docker.sh` 覆盖）。若镜像不存在，需先加载或构建（如 `docker load -i docker/automap-env_humble.tar` 或 `run_full_mapping_docker.sh --build`）。
- **镜像内无 PyTorch 时**：脚本会先用**国内 pip 镜像**在容器内安装 `torch`、`pyyaml` 再转换（默认清华源）。使用其他镜像可设置环境变量后执行，例如：
  - 清华（默认）：`./convert_ot_model_in_docker.sh`
  - 阿里云：`PIP_INDEX="-i https://mirrors.aliyun.com/pypi/simple/" ./convert_ot_model_in_docker.sh`
  - 豆瓣：`PIP_INDEX="-i https://pypi.douban.com/simple" ./convert_ot_model_in_docker.sh`
 若环境完全无网络，请用含 PyTorch 的 Dockerfile 重新构建镜像（如 `./run_full_mapping_docker.sh --build`）。

转换完成后，`.pt` 会出现在宿主机 `automap_pro/models/` 下，无需再拷文件。

### 2.2 使用 OverlapTransformer 原仓脚本

也可在子模块内用其自带脚本（需先改 `../config/config.yml` 里的 `test_weights` 指向你的 `.pth.tar`）：

```bash
cd automap_pro/src/modular/OverlapTransformer-master/OT_libtorch
python3 gen_libtorch_model.py
# 会生成 ./overlapTransformer.pt，再拷贝到 automap_pro/models/
cp overlapTransformer.pt ../../../../models/
```

---

## 3. 配置 model_path 指向 .pt

主配置中 **必须** 使用转换后的 `.pt` 路径，例如在 `system_config_M2DGR.yaml` 中：

```yaml
loop_closure:
  overlap_transformer:
    model_path: "${CMAKE_CURRENT_SOURCE_DIR}/models/overlapTransformer.pt"
```

`${CMAKE_CURRENT_SOURCE_DIR}` 在加载时会被替换为**配置所在包根目录**（开发时为 automap_pro，安装后为 share/automap_pro），因此：

- **开发/未 install**：请把 `overlapTransformer.pt` 放在 `automap_pro/models/` 下。  
- **install 后**：CMake 会安装 `models/*.pt` 到 `share/automap_pro/models/`，只要用上述配置即可。

若使用绝对路径，可直接写：

```yaml
model_path: "/abs/path/to/overlapTransformer.pt"
```

---

## 4. 安装时带上模型（可选）

若希望 `colcon build` + `install` 后直接运行，无需再拷文件，可保证：

1. 转换得到的 `overlapTransformer.pt` 已放在 **`automap_pro/models/`** 下；  
2. CMake 已包含对 `models/` 的安装（仅 `*.pt` 与 `*.pth.tar`），例如：

```cmake
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models")
  install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.pt" PATTERN "*.pth.tar")
endif()
```

这样安装后 `share/automap_pro/models/overlapTransformer.pt` 存在，与配置中的展开路径一致。

---

## 5. 验证是否加载成功

- **编译状态**：查看 **build.log**，搜索 `[automap_pro] LibTorch`，应看到 `BUILD STATUS: FOUND -> USE_TORCH=1` 或 `NOT FOUND -> USE_TORCH=0`。
- **运行状态**：启动建图/回环节点后查看日志：
  - `[LoopDetector][LibTorch] compile-time USE_TORCH=1` 表示本二进制带 LibTorch 编译，可加载 .pt；`USE_TORCH=0` 表示仅 fallback。
  - 出现 **“Model loaded”**（或等价成功日志）且 **没有** “LibTorch not available” → LibTorch 可用且 OT 模型已加载。
  - 若编译时未链接 LibTorch 或未启用，会打印 “LibTorch not compiled (USE_TORCH off)”，此时回环不会使用 OT 描述子。

- 可选：对回环相关 topic/日志做一次短时运行，确认有 OT 参与的回环候选或描述子计算（视当前日志与代码而定）。

---

## 6. 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| 编译报错找不到 Torch | LibTorch 未找到 | 设置 `LIBTORCH_HOME` 或 `Torch_DIR`，见第 1 节。 |
| 运行时报错加载模型失败 | 仍指向 .pth.tar 或路径错误 | 必须用 .pt；检查 model_path 是否指向转换后的 .pt 及路径是否可读。 |
| 转换脚本报错 No module 'modules' | 未在正确环境运行 | 使用仓库内 `scripts/convert_ot_pth_to_torchscript.py`，它会自动把 OverlapTransformer-master 加入 path。 |
| 安装后找不到模型 | 未安装 models 或路径不对 | 确认 CMake 安装了 models/，且配置中用的是 `${CMAKE_CURRENT_SOURCE_DIR}/models/overlapTransformer.pt`（会展开到 share/automap_pro）。 |

---

## 7. 待替换项清单（占位符）

- `<path/to/libtorch>`：本地 LibTorch 解压目录。  
- `<path/to/automap_ws>`：你的 ROS2 工作空间路径。  
- `/path/to/automap_pro`：automap_pro 源码包路径（或 install 后 share 下的 automap_pro）。  

按上述步骤完成后，即可保证 OverlapTransformer 模型被正常加载并参与回环计算。
