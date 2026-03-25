#!/usr/bin/env python3
"""
从 LSK3DNet 权重导出分类头为 TorchScript，供 C++ LibTorch 与 Python spconv 骨干配合使用。
同时写入 {output}.meta.json（feat_dim、num_classes、checkpoint SHA256），供运行时契约校验。

  python3 scripts/export_lsk3dnet_classifier_torchscript.py \\
    --config config/lk-semantickitti_erk_finetune.yaml \\
    --checkpoint /path/to/model.pt \\
    --output /path/to/lsk_classifier.pt \\
    --device cpu
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys


def _sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    if root not in sys.path:
        sys.path.insert(0, root)
    os.chdir(root)

    import torch
    from easydict import EasyDict
    from network.largekernel_model import get_model_class
    from utils.load_util import load_yaml
    from utils.load_save_util import load_checkpoint_old

    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--device", default="cpu")
    args = ap.parse_args()

    ck_abs = os.path.abspath(args.checkpoint)
    if not os.path.isfile(ck_abs):
        print(f"[err] checkpoint not found: {ck_abs}", file=sys.stderr)
        return 1

    cfg = EasyDict(load_yaml(args.config))
    model = get_model_class(cfg.model_params.model_architecture)(cfg)
    dev = torch.device(args.device)
    model.to(dev)
    load_checkpoint_old(args.checkpoint, model)
    model.eval()

    feat_dim = int(model.hiden_size * model.num_scales)
    dummy = torch.randn(1, feat_dim, device=dev)
    clf = model.classifier
    clf.eval()
    with torch.no_grad():
        traced = torch.jit.trace(clf, (dummy,), strict=False)
    out_abs = os.path.abspath(args.output)
    os.makedirs(os.path.dirname(out_abs) or ".", exist_ok=True)
    traced.save(out_abs)

    ck_sha = _sha256_file(ck_abs)
    meta = {
        "schema_version": 1,
        "feat_dim": feat_dim,
        "num_classes": int(model.num_classes),
        "input_dims": int(model.input_dims),
        "checkpoint_sha256_hex": ck_sha,
        "checkpoint_path_export": ck_abs,
        "config_yaml_export": os.path.abspath(args.config),
        "torch_version_export": torch.__version__,
    }
    meta_path = out_abs + ".meta.json"
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)
    print(f"[ok] feat_dim={feat_dim} num_classes={model.num_classes} -> {out_abs}")
    print(f"[ok] meta -> {meta_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
