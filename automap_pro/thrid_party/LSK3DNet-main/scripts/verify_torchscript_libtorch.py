#!/usr/bin/env python3
"""
LSK3DNet：LibTorch 部署预检 — 步骤 1→2（Python）+ 可选步骤 3（C++ smoke 由 shell 调用）

步骤 1: 加载 config + checkpoint，构造与 collate 一致的 synthetic batch，torch.jit.trace 包装模块并保存 .pt
步骤 2: torch.jit.load 再前向，与 trace 当次输出数值对比
步骤 3: 在宿主机/容器内用 LibTorch C++ 仅 load（及可选一次 forward）— 见同目录 build_cpp_jit_smoke.sh

注意:
- 本网络使用 spconv / torch_scatter / torch.unique，TorchScript 可能失败；失败时脚本以非零退出并打印原因。
- checkpoint 需与 train 一致：含 ['checkpoint'] 键的 dict（load_checkpoint_old），或整包 state_dict。

用法（在 LSK3DNet-main 根目录，且已安装与训练一致的 torch / spconv / torch_scatter 等）:
  python3 scripts/verify_torchscript_libtorch.py \\
    --config config/lk-semantickitti_erk_finetune.yaml \\
    --checkpoint /path/to/model.pt \\
    --output /tmp/lsk3dnet_traced.pt \\
    --num-points 4096 \\
    --device cuda:0

仅 synthetic、无权重（验证图能否 trace）:
  python3 scripts/verify_torchscript_libtorch.py --config ... --skip-checkpoint ...
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile

import torch
import torch.nn as nn


def _repo_root() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def _prepend_path():
    root = _repo_root()
    if root not in sys.path:
        sys.path.insert(0, root)
    os.chdir(root)


def load_config(path: str):
    from easydict import EasyDict
    from utils.load_util import load_yaml

    cfg = load_yaml(path)
    return EasyDict(cfg)


def load_weights(model, ckpt_path: str, device: torch.device):
    raw = torch.load(ckpt_path, map_location=device)
    if isinstance(raw, dict) and "checkpoint" in raw:
        from utils.load_save_util import load_checkpoint_old

        load_checkpoint_old(ckpt_path, model)
        return
    if isinstance(raw, dict):
        # 纯 state_dict 或 key 为 model
        state = raw.get("state_dict", raw)
        missing, unexpected = model.load_state_dict(state, strict=False)
        print(f"[load] strict=False missing={len(missing)} unexpected={len(unexpected)}")
        return
    raise ValueError(f"Unknown checkpoint format: {type(raw)}")


def synthetic_batch(configs, num_points: int, device: torch.device):
    mp = configs.model_params
    dp = configs.dataset_params
    idim = int(mp.input_dims)
    min_v = torch.tensor(dp.min_volume_space, dtype=torch.float32, device=device)
    max_v = torch.tensor(dp.max_volume_space, dtype=torch.float32, device=device)
    u = torch.rand(num_points, 3, device=device)
    xyz = u * (max_v - min_v) + min_v
    rest = idim - 3
    if rest > 0:
        sig = torch.randn(num_points, rest, device=device, dtype=torch.float32) * 0.1
        points = torch.cat([xyz, sig], dim=1)
    else:
        points = xyz[:, :idim]
    normal = torch.randn(num_points, 3, device=device, dtype=torch.float32) * 0.01
    batch_idx = torch.zeros(num_points, dtype=torch.long, device=device)
    labels = torch.zeros(num_points, dtype=torch.long, device=device)
    return points, normal, batch_idx, labels


class TraceWrapper(nn.Module):
    """将 dict forward 转为张量 tuple，便于 jit.trace。"""

    def __init__(self, core: nn.Module, batch_size: int = 1):
        super().__init__()
        self.core = core
        self._bs = int(batch_size)

    def forward(self, points, normal, batch_idx, labels):
        data_dict = {
            "points": points,
            "normal": normal,
            "batch_idx": batch_idx,
            "labels": labels,
            "batch_size": self._bs,
        }
        out = self.core(data_dict)
        return out["logits"]


def main() -> int:
    _prepend_path()

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="YAML，如 config/lk-semantickitti_erk_finetune.yaml")
    parser.add_argument("--checkpoint", default="", help="权重 .pt；空且未 --skip-checkpoint 则用 config 内 model_load_path")
    parser.add_argument("--skip-checkpoint", action="store_true", help="不加载权重（仅测随机初始化能否 trace）")
    parser.add_argument("--output", default="", help="保存 traced .pt；默认临时目录")
    parser.add_argument("--num-points", type=int, default=4096)
    parser.add_argument("--device", default="cuda:0" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--report", default="", help="写入 JSON 报告路径")
    args = parser.parse_args()

    device = torch.device(args.device)
    configs = load_config(args.config)

    from network.largekernel_model import get_model_class

    model = get_model_class(configs.model_params.model_architecture)(configs)
    model.to(device)

    ckpt = args.checkpoint.strip()
    if not args.skip_checkpoint:
        if not ckpt:
            ckpt = configs.model_params.get("model_load_path", "")
        if ckpt and os.path.isfile(ckpt):
            load_weights(model, ckpt, device)
            print(f"[ok] loaded checkpoint: {ckpt}")
        elif ckpt:
            print(f"[warn] checkpoint not found: {ckpt}, using random init")
        else:
            print("[warn] no checkpoint path, using random init")

    model.eval()
    pts, nor, bidx, labs = synthetic_batch(configs, args.num_points, device)

    wrap = TraceWrapper(model, batch_size=1).to(device)
    wrap.eval()

    report = {
        "step1_trace": None,
        "step2_reload": None,
        "step3_note": "Run scripts/build_cpp_jit_smoke.sh with same LibTorch as PyTorch major.minor",
    }

    with torch.no_grad():
        eager_logits = wrap(pts, nor, bidx, labs)

    out_path = args.output
    if not out_path:
        out_path = os.path.join(tempfile.gettempdir(), "lsk3dnet_traced.pt")

    try:
        with torch.no_grad():
            traced = torch.jit.trace(wrap, (pts, nor, bidx, labs), strict=False)
        traced.eval()
        with torch.no_grad():
            traced_logits = traced(pts, nor, bidx, labs)
        max_diff_trace = (eager_logits - traced_logits).abs().max().item()
        os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
        traced.save(out_path)
        report["step1_trace"] = {
            "ok": True,
            "saved": os.path.abspath(out_path),
            "max_abs_diff_eager_vs_traced": max_diff_trace,
        }
        print(f"[step1] trace OK, max |eager-traced| = {max_diff_trace}")
    except Exception as e:
        report["step1_trace"] = {"ok": False, "error": repr(e)}
        print(f"[step1] FAILED: {e}", file=sys.stderr)
        if args.report:
            with open(args.report, "w") as f:
                json.dump(report, f, indent=2)
        return 1

    # Step 2: load saved jit and compare
    try:
        loaded = torch.jit.load(out_path, map_location=device)
        loaded.eval()
        with torch.no_grad():
            loaded_logits = loaded(pts, nor, bidx, labs)
        max_diff_reload = (traced_logits - loaded_logits).abs().max().item()
        report["step2_reload"] = {
            "ok": True,
            "max_abs_diff_traced_vs_loaded": max_diff_reload,
        }
        print(f"[step2] load OK, max |traced-loaded| = {max_diff_reload}")
    except Exception as e:
        report["step2_reload"] = {"ok": False, "error": repr(e)}
        print(f"[step2] FAILED: {e}", file=sys.stderr)
        if args.report:
            with open(args.report, "w") as f:
                json.dump(report, f, indent=2)
        return 2

    if args.report:
        with open(args.report, "w") as f:
            json.dump(report, f, indent=2)
        print(f"[report] wrote {args.report}")

    print(f"[step3] 在容器或宿主机执行（需 g++ 与 LibTorch）:")
    print(f"  bash {_repo_root()}/scripts/build_cpp_jit_smoke.sh {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
