#!/usr/bin/env python3
"""
LSK3DNet 混合推理子进程：在 Python 中执行含 spconv 的骨干，输出拼接后的点级特征张量；
分类头由 C++ LibTorch 加载的 TorchScript 执行。

INIT JSON（UTF-8）扩展字段（生产）:
  normal_mode: "range" | "zeros" — range 与 dataset2 一致，使用距离图法线（需 c_gen_normal_map）
  normal_fov_up_deg, normal_fov_down_deg, normal_proj_h, normal_proj_w — 法线距离图参数（默认 3,-25,64,900 与训练默认一致）
  expected_checkpoint_sha256: 可选，与导出 .meta.json 中 checkpoint_sha256_hex 一致时强制校验权重文件

协议帧：见仓库内同文件历史注释（LSK1 魔数、CMD_INIT / INFER / SHUTDOWN）。
"""
from __future__ import annotations

import hashlib
import json
import os
import struct
import sys
import traceback

# 🏛️ [架构加固] 立即重定向标准输出到标准错误，防止后续所有 import 或 print 污染二进制协议通道
# 必须在任何三方库（如 torch, spconv）加载之前执行
_real_stdout_buffer = sys.stdout.buffer
sys.stdout = sys.stderr

MAGIC = b"LSK1"
VERSION = 1
CMD_INIT = 1
CMD_INFER = 2
CMD_SHUTDOWN = 3
ERR_RESPONSE = 0xFFFFFFFF


def read_exact(f, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = f.read(n - len(buf))
        if not chunk:
            raise EOFError("short read")
        buf += chunk
    return buf


def write_exact(f, data: bytes) -> None:
    f.write(data)
    f.flush()


def read_frame(f):
    m = read_exact(f, 4)
    if m != MAGIC:
        raise ValueError(f"bad magic {m!r}")
    ver, cmd, plen = struct.unpack("<IIQ", read_exact(f, 16))
    if ver != VERSION:
        raise ValueError(f"bad version {ver}")
    payload = read_exact(f, plen) if plen else b""
    return cmd, payload


def write_error(f, msg: str) -> None:
    b = msg.encode("utf-8")
    write_exact(f, MAGIC + struct.pack("<IIQ", VERSION, ERR_RESPONSE, len(b)) + b)


def write_ok_init(f, input_dims: int, feat_dim: int, num_classes: int) -> None:
    body = struct.pack("<III", int(input_dims), int(feat_dim), int(num_classes))
    write_exact(f, MAGIC + struct.pack("<IIQ", VERSION, CMD_INIT, len(body)) + body)


def write_ok_infer(f, n: int, feat_dim: int, feat_f32: bytes) -> None:
    head = struct.pack("<QI", int(n), int(feat_dim))
    body = head + feat_f32
    write_exact(f, MAGIC + struct.pack("<IIQ", VERSION, CMD_INFER, len(body)) + body)


def _sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fp:
        for chunk in iter(lambda: fp.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def verify_checkpoint_sha256(checkpoint_path: str, expected_hex: str) -> None:
    if not expected_hex:
        return
    exp = expected_hex.strip().lower()
    got = _sha256_file(checkpoint_path).lower()
    if got != exp:
        raise ValueError(f"checkpoint SHA256 mismatch: file={got[:16]}... expected={exp[:16]}...")


def load_model_bundle(config_yaml: str, checkpoint: str, device_s: str):
    import torch
    from easydict import EasyDict
    from network.largekernel_model import get_model_class
    from utils.load_util import load_yaml
    from utils.load_save_util import load_checkpoint_old

    print(f"[lsk3dnet_hybrid_worker] loading config from {config_yaml}", file=sys.stderr)
    cfg = EasyDict(load_yaml(config_yaml))
    model = get_model_class(cfg.model_params.model_architecture)(cfg)
    dev = torch.device(device_s)
    model.to(dev)
    if checkpoint and os.path.isfile(checkpoint):
        print(f"[lsk3dnet_hybrid_worker] loading checkpoint from {checkpoint} to {dev}", file=sys.stderr)
        raw = torch.load(checkpoint, map_location=dev, weights_only=False)
        if isinstance(raw, dict) and "checkpoint" in raw:
            load_checkpoint_old(checkpoint, model)
        elif isinstance(raw, dict):
            st = raw.get("state_dict", raw)
            miss, unexp = model.load_state_dict(st, strict=False)
            print(
                f"[lsk3dnet_hybrid_worker] load_state_dict strict=False missing={len(miss)} unexpected={len(unexp)}",
                file=sys.stderr,
            )
        else:
            raise ValueError("unsupported checkpoint type")
    else:
        print(f"[lsk3dnet_hybrid_worker] WARNING: checkpoint not found or not specified: {checkpoint}", file=sys.stderr)
    
    model.eval()
    feat_dim = int(model.hiden_size * model.num_scales)
    packed_input_dims = int(model.input_dims)
    # voxel_3d_generator.prepare_input() always builds:
    #   cat(point, nor_pc(3), center_to_point(3), normal(3))
    # so the protocol must carry RAW point dims = packed_input_dims - 9.
    if packed_input_dims <= 9:
        raise ValueError(
            f"invalid model.input_dims={packed_input_dims}; expected packed dims > 9 "
            f"(point + nor_pc + center + normal)"
        )
    raw_input_dims = packed_input_dims - 9
    num_classes = int(model.num_classes)
    print(
        f"[lsk3dnet_hybrid_worker] model initialized: feat_dim={feat_dim} "
        f"packed_input_dims={packed_input_dims} raw_input_dims={raw_input_dims} num_classes={num_classes}",
        file=sys.stderr,
    )
    return model, dev, feat_dim, raw_input_dims, num_classes


def compute_normals_like_training(
    feat_np: "np.ndarray",
    mode: str,
    fov_up_deg: float,
    fov_down_deg: float,
    proj_h: int,
    proj_w: int,
) -> "np.ndarray":
    import numpy as np

    if mode == "zeros":
        return np.zeros((feat_np.shape[0], 3), dtype=np.float32)
    if mode != "range":
        raise ValueError(f"unknown normal_mode {mode!r}")

    from utils.normalmap import range_projection
    import utils.depth_map_utils as depth_map_utils

    try:
        from c_gen_normal_map import gen_normal_map
    except ImportError as e:
        raise RuntimeError(
            "normal_mode=range requires LSK native `c_gen_normal_map` (see LSK3DNet README). "
            "Use hybrid_normal_mode=zeros only for debugging."
        ) from e

    proj_range, proj_vertex, _, _, from_proj_x, from_proj_y = range_projection(
        feat_np, fov_up=fov_up_deg, fov_down=fov_down_deg, proj_H=proj_h, proj_W=proj_w
    )
    # 🏛️ [Ideal State Fix] Check for invalid projections before normal generation
    # Points falling outside the specified normal_fov will have proj_range == -1
    proj_range = depth_map_utils.fill_in_fast(proj_range, extrapolate=True, blur_type="gaussian")
    normal_data = gen_normal_map(proj_range, proj_vertex, proj_h, proj_w)
    
    # 🏛️ [Robustness Fix] Mask out points that are far outside the normal estimation window
    # to prevent backbone from receiving 'garbage' normal features.
    mask_invalid = (from_proj_y < 0) | (from_proj_y >= proj_h) | (from_proj_x < 0) | (from_proj_x >= proj_w)
    normals = normal_data[np.clip(from_proj_y, 0, proj_h-1), np.clip(from_proj_x, 0, proj_w-1)]
    normals[mask_invalid] = 0
    return normals


def forward_point_features(model, device, points: "torch.Tensor", normals: "torch.Tensor"):
    import torch

    model.eval()
    n = int(points.shape[0])
    batch_idx = torch.zeros(n, dtype=torch.long, device=device)
    labels = torch.zeros(n, dtype=torch.long, device=device)
    data_dict = {
        "points": points.to(device),
        "normal": normals.to(device),
        "batch_idx": batch_idx,
        "labels": labels,
        "batch_size": 1,
    }
    with torch.inference_mode():
        with torch.no_grad():
            data_dict = model.voxelizer(data_dict)
        data_dict = model.voxel_3d_generator(data_dict)
        enc_feats = []
        for i in range(model.num_scales):
            enc_feats.append(model.spv_enc[i](data_dict))
        feat = torch.cat(enc_feats, dim=1)
    feat = feat.detach().float().cpu().contiguous()
    return feat.numpy().tobytes()


def main_stdio():
    import numpy as np
    import torch

    # 🏛️ [协议通道解耦] 优先尝试使用专门的文件描述符 3 进行二进制通信
    # 如果 FD 3 不可用（如旧版或手动启动），回退到原始 stdout 缓冲
    try:
        # 检查 FD 3 是否有效且可写
        os.fstat(3)
        proto_out = os.fdopen(3, "wb", buffering=0)
        print("[lsk3dnet_hybrid_worker] Using FD 3 for binary protocol", file=sys.stderr)
    except (OSError, ValueError):
        # 🏛️ [架构加固] 极端情况下 FD 3 不可用，回退到原始 stdout 缓冲（此时 FD 1 应已被 C++ 父进程重定向到 stderr）
        # 除非明确传参要求 stdio 模式，否则 FD 3 不可用应该是致命错误以防止隐式故障。
        if "--stdio" in sys.argv:
            proto_out = _real_stdout_buffer
            print("[lsk3dnet_hybrid_worker] FD 3 not available, falling back to real stdout buffer", file=sys.stderr)
        else:
            print("[lsk3dnet_hybrid_worker] CRITICAL: FD 3 not available and --stdio not specified. ABORT.", file=sys.stderr)
            sys.exit(124)

    fin = sys.stdin.buffer
    fout = proto_out
    model = None
    device = None
    feat_dim = 0
    input_dims = 0
    num_classes = 0
    normal_mode = "range"
    normal_fov_up_deg = 3.0
    normal_fov_down_deg = -25.0
    normal_proj_h = 64
    normal_proj_w = 900

    while True:
        try:
            cmd, payload = read_frame(fin)
        except EOFError:
            return 0
        except Exception as e:
            print(f"[lsk3dnet_hybrid_worker] frame read error: {e}", file=sys.stderr)
            return 1

        if cmd == CMD_SHUTDOWN:
            return 0

        if cmd == CMD_INIT:
            try:
                spec = json.loads(payload.decode("utf-8"))
                cy = spec["config_yaml"]
                ck = spec.get("checkpoint", "")
                device_s = spec.get("device", "cpu")
                exp_sha = spec.get("expected_checkpoint_sha256", "") or ""
                if ck and exp_sha:
                    verify_checkpoint_sha256(ck, exp_sha)
                normal_mode = str(spec.get("normal_mode", "range")).lower()
                normal_fov_up_deg = float(spec.get("normal_fov_up_deg", 3.0))
                normal_fov_down_deg = float(spec.get("normal_fov_down_deg", -25.0))
                normal_proj_h = int(spec.get("normal_proj_h", 64))
                normal_proj_w = int(spec.get("normal_proj_w", 900))
                model, device, feat_dim, input_dims, num_classes = load_model_bundle(cy, ck, device_s)
                write_ok_init(fout, input_dims, feat_dim, num_classes)
            except Exception:
                write_error(fout, traceback.format_exc())
            continue

        if cmd == CMD_INFER:
            if model is None:
                write_error(fout, "INFER before INIT")
                continue
            try:
                if len(payload) < 12:
                    write_error(fout, "INFER payload too short")
                    continue
                n, c = struct.unpack_from("<QI", payload, 0)
                n = int(n)
                c = int(c)
                expected = 12 + n * c * 4
                if len(payload) != expected:
                    write_error(fout, f"INFER size mismatch want {expected} got {len(payload)}")
                    continue
                if c != input_dims:
                    write_error(fout, f"channel mismatch: got {c} want {input_dims}")
                    continue
                
                # Periodic logging for inference
                if n > 0:
                    print(f"[lsk3dnet_hybrid_worker] inferring {n} points...", file=sys.stderr)

                arr = np.frombuffer(payload, dtype=np.float32, offset=12, count=n * c).reshape(n, c)
                points = torch.from_numpy(arr.copy())
                nor_np = compute_normals_like_training(
                    arr, normal_mode, normal_fov_up_deg, normal_fov_down_deg, normal_proj_h, normal_proj_w
                )
                normals = torch.from_numpy(nor_np.copy())
                blob = forward_point_features(model, device, points, normals)
                if len(blob) != n * feat_dim * 4:
                    write_error(fout, "internal feature size mismatch")
                    continue
                write_ok_infer(fout, n, feat_dim, blob)
            except Exception:
                write_error(fout, traceback.format_exc())
            continue

        write_error(fout, f"unknown cmd {cmd}")
    return 0


def main():
    if len(sys.argv) >= 2 and sys.argv[1] == "--stdio":
        return main_stdio()
    print("usage: lsk3dnet_hybrid_worker.py --stdio", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
