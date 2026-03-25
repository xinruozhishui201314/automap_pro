#!/usr/bin/env python3
"""
在 automap-env:humble 容器内将 PyTorch .pt 导出为 ONNX（通用辅助脚本）。

适用前提
- 输入为 TorchScript（torch.jit.save / torch.jit.trace 得到的 .pt），且 forward 接受
  单个浮点张量输入；此时用本脚本 + --torchscript 与匹配的 --shapes。
- 若为「含 spconv 等稀疏算子」的网络（如 LSK3DNet），标准 ONNX 常无法导出；
  请在 LSK3DNet 仓库内编写与模型 forward 一致的 export 脚本，仍通过
  run_export_onnx_in_docker.sh 进入同一容器执行。

依赖：镜像内已有 python3-torch（与 AutoMap 构建环境一致）。

示例
  python3 /root/scripts/export_pt_to_onnx_generic.py \\
    --input /data/models/model.ts \\
    --output /data/models/model.onnx \\
    --torchscript \\
    --shapes 1,4,64,2048 \\
    --opset 17
"""
from __future__ import annotations

import argparse
import os
import sys


def _parse_shapes(s: str) -> list[int]:
    parts = [p.strip() for p in s.split(",") if p.strip()]
    if len(parts) < 2:
        raise ValueError("--shapes 至少需要 2 维，例如 1,4,64,2048")
    return [int(p) for p in parts]


def main() -> int:
    p = argparse.ArgumentParser(description="Export .pt (TorchScript) to ONNX (single Tensor I/O helper).")
    p.add_argument("--input", "-i", required=True, help="输入 .pt 路径（TorchScript）")
    p.add_argument("--output", "-o", required=True, help="输出 .onnx 路径")
    p.add_argument("--torchscript", action="store_true", help="使用 torch.jit.load（推荐）")
    p.add_argument(
        "--shapes",
        default="1,4,64,2048",
        help="单个 dummy 输入形状，逗号分隔，如 1,4,64,2048（B,C,H,W）",
    )
    p.add_argument("--opset", type=int, default=17)
    p.add_argument("--input-name", default="input", help="ONNX 输入名")
    p.add_argument("--output-name", default="output", help="ONNX 输出名（多输出时 ONNX 仍可能只有一个合并名，视模型而定）")
    args = p.parse_args()

    if not os.path.isfile(args.input):
        print(f"Error: input not found: {args.input}", file=sys.stderr)
        return 2

    try:
        import torch
    except ImportError:
        print("Error: torch not installed in this environment.", file=sys.stderr)
        return 3

    shapes = _parse_shapes(args.shapes)
    dummy = torch.randn(*shapes, dtype=torch.float32)

    if args.torchscript:
        model = torch.jit.load(args.input, map_location="cpu")
    else:
        print(
            "Error: 非 --torchscript 模式需要你在业务代码里构建 nn.Module 并加载 state_dict。\n"
            "请使用 LSK3DNet 官方 builder 写专用导出脚本，或对本脚本加扩展。",
            file=sys.stderr,
        )
        return 4

    model.eval()
    os.makedirs(os.path.dirname(os.path.abspath(args.output)) or ".", exist_ok=True)

    torch.onnx.export(
        model,
        dummy,
        args.output,
        export_params=True,
        opset_version=args.opset,
        do_constant_folding=True,
        input_names=[args.input_name],
        output_names=[args.output_name],
    )
    print(f"OK: wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
