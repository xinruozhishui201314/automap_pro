# sloam_rec（随 automap_pro 包分发）

本目录为 **SemanticProcessor** 编译与运行所需的 `sloam_rec` 拷贝，路径固定为：

`automap_pro/thrid_party/sloam_rec/`

CMake **仅** 使用 `${CMAKE_CURRENT_SOURCE_DIR}/thrid_party/sloam_rec/sloam/include`，不再引用仓库外 `../thrid_party`。

## 更新上游

若需同步上游版本，在仓库根目录执行（按需调整源路径）：

```bash
rsync -a --delete /path/to/upstream/sloam_rec/ automap_pro/thrid_party/sloam_rec/
```

## ONNX 模型

语义模型路径由运行时参数 `semantic.model_path` 指定，建议使用 `share` 或安装前缀下的绝对路径，勿写死其他工作区路径。
