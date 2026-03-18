#!/usr/bin/env python3
"""
将 OverlapTransformer 的 .pth.tar 检查点转为 LibTorch 可加载的 TorchScript .pt。
用法:
  python3 convert_ot_pth_to_torchscript.py /path/to/pretrained_overlap_transformer.pth.tar [output.pt]
若未指定 output.pt，默认输出到 当前目录/models/overlapTransformer.pt（或脚本所在包根目录/models/）。
需安装: PyTorch (pip install torch)
"""
from __future__ import print_function
import os
import sys

def _find_ot_root():
    # 脚本在 automap_pro/scripts/ 或 install/share/automap_pro/scripts/
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # 包根 = 上一级（automap_pro）
    pkg_root = os.path.dirname(script_dir)
    ot_root = os.path.join(pkg_root, "src", "modular", "OverlapTransformer-master")
    if not os.path.isdir(ot_root):
        ot_root = os.path.join(script_dir, "..", "src", "modular", "OverlapTransformer-master")
    ot_root = os.path.abspath(ot_root)
    if not os.path.isdir(ot_root):
        return None
    return ot_root

def main():
    if len(sys.argv) < 2:
        print("Usage: %s <path/to/pretrained_overlap_transformer.pth.tar> [output.pt]" % sys.argv[0], file=sys.stderr)
        sys.exit(1)
    pth_path = os.path.abspath(sys.argv[1])
    if not os.path.isfile(pth_path):
        print("Error: not a file: %s" % pth_path, file=sys.stderr)
        sys.exit(2)

    ot_root = _find_ot_root()
    if not ot_root:
        print("Error: OverlapTransformer-master not found (expected under automap_pro/src/modular/).", file=sys.stderr)
        sys.exit(3)
    if ot_root not in sys.path:
        sys.path.insert(0, ot_root)
    # tools 目录加入 path，使 tools/read_samples.py 里的 "from utils.utils" 能找到 tools/utils/
    ot_tools = os.path.join(ot_root, "tools")
    if ot_tools not in sys.path:
        sys.path.insert(0, ot_tools)

    import torch
    from modules.overlap_transformer import featureExtracter

    out_path = sys.argv[2] if len(sys.argv) > 2 else None
    if not out_path:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_root = os.path.dirname(script_dir)
        models_dir = os.path.join(pkg_root, "models")
        os.makedirs(models_dir, exist_ok=True)
        out_path = os.path.join(models_dir, "overlapTransformer.pt")
    out_path = os.path.abspath(out_path)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    checkpoint = torch.load(pth_path, map_location=device)
    if isinstance(checkpoint, dict) and "state_dict" in checkpoint:
        state = checkpoint["state_dict"]
    else:
        state = checkpoint

    # KITTI 64x900；Haomo 为 32x900 需改此处
    amodel = featureExtracter(height=64, width=900, channels=1, use_transformer=True)
    amodel.load_state_dict(state, strict=True)
    amodel.to(device)
    amodel.eval()

    example = torch.rand(1, 1, 64, 900, device=device)
    with torch.no_grad():
        traced = torch.jit.trace(amodel, example)
    traced.save(out_path)
    print("Saved TorchScript model to:", out_path)
    return 0

if __name__ == "__main__":
    sys.exit(main())
