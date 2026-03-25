#!/usr/bin/env python3
"""
可验证性：检查分类头 TorchScript 与 .meta.json 是否与 C++ 启动时契约一致。

  python3 verify_lsk3dnet_hybrid_classifier.py --classifier /path/to/lsk_classifier.pt

依赖: pip install torch（与 LibTorch 主版本尽量一致）
退出码: 0=通过，非0=失败
"""
from __future__ import annotations

import argparse
import json
import os
import sys


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--classifier", required=True, help="export_lsk3dnet_classifier_torchscript.py 输出的 .pt")
    args = ap.parse_args()
    pt = os.path.abspath(args.classifier)
    meta_path = pt + ".meta.json"
    if not os.path.isfile(pt):
        print(f"[fail] missing classifier: {pt}", file=sys.stderr)
        return 1
    if not os.path.isfile(meta_path):
        print(f"[warn] missing {meta_path} — C++ 将无法校验 feat_dim/SHA256（生产环境应随导出生成）", file=sys.stderr)

    try:
        import torch
    except ImportError:
        print("[fail] need: pip install torch", file=sys.stderr)
        return 2

    meta = {}
    if os.path.isfile(meta_path):
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = json.load(f)

    m = torch.jit.load(pt, map_location="cpu")
    m.eval()
    fd = int(meta.get("feat_dim", 0))
    nc = int(meta.get("num_classes", 0))
    if fd <= 0 or nc <= 0:
        print("[fail] meta.json missing feat_dim/num_classes", file=sys.stderr)
        return 3
    x = torch.zeros(1, fd)
    with torch.no_grad():
        y = m(x)
    if y.shape != (1, nc):
        print(f"[fail] forward shape {tuple(y.shape)} expected (1, {nc})", file=sys.stderr)
        return 4
    print(f"[ok] classifier={pt} feat_dim={fd} num_classes={nc} torch={torch.__version__}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
