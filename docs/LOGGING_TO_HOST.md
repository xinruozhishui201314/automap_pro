# 工程日志统一写入宿主机 `logs/`

## 默认行为：全部日志写入项目根下的 `logs/`

**宿主机**上的日志目录默认为 **`<项目根>/logs`**。工程**编译与运行的所有日志**都会写入该目录，便于精准分析问题：

| 文件 | 内容 | 何时产生 |
|------|------|----------|
| **automap.log** | 本次运行的**全程总日志**（脚本 + 编译 + 运行） | 每次执行脚本都会写入 |
| **build.log** | 编译阶段完整输出 | 执行到编译时 |
| **full.log** | 建图运行阶段完整输出 | 执行到运行时 |
| **clean.log** | 清理编译产物输出 | 仅在使用 `--clean` 时 |
| **image.log** | 镜像加载/构建输出 | 仅当镜像需加载或构建时 |

无需加任何参数，直接执行即可：

```bash
# 在宿主机项目根目录执行；编译与运行日志自动写入 logs/
bash run_automap.sh --build-only --clean && \
bash run_automap.sh --offline \
  --bag-file "$(pwd)/data/automap_input/M2DGR/street_03_ros2" \
  --config system_config_M2DGR.yaml \
  --gdb
```

- **完整时间线**：`cat logs/automap.log`
- **编译问题**：`cat logs/build.log` 或搜索 `error:`、`FAILED`
- **运行问题**：`cat logs/full.log` 或 `./diagnose_global_map.sh logs/full.log`
- 更多说明见 **`logs/README.md`**

---

## 覆盖默认日志目录：`--log-dir`

若希望把日志写到其他宿主机目录，可使用 `--log-dir`：

```bash
# 保存到当前目录下的 my_logs
bash run_automap.sh --offline --bag-file /data/xxx --log-dir "$(pwd)/my_logs"

# 保存到任意绝对路径
bash run_automap.sh --offline --bag-file /data/xxx --log-dir /home/wqs/automap_logs
```

- 编译仍会写入 `<log-dir>/build.log`，运行写入 `<log-dir>/full.log`（与默认 `logs/` 行为一致，只是目录不同）。

---

## 目录与文件说明

- **logs/** 在**宿主机**上，由脚本在首次运行时创建（`mkdir -p`）。
- **automap.log**：脚本启动后通过 `exec > >(tee logs/automap.log)` 将本次运行的全部 stdout/stderr 同时写入该文件。
- **build.log**：编译阶段宿主机执行 `docker run ... 2>&1 | tee logs/build.log`。
- **full.log**：运行阶段容器内挂载 `logs` → `/root/run_logs`，`ros2 launch ... 2>&1 | tee /root/run_logs/full.log`。
- **clean.log** / **image.log**：清理与镜像步骤的输出分别 tee 到对应文件。
- 诊断运行问题：在宿主机执行 `./diagnose_global_map.sh logs/full.log`。
