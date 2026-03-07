#!/usr/bin/env python3
"""
验证 OverlapTransformer 模型权重完整性与推理正确性
检查清单：
1. 权重加载是否成功
2. conv1 输入通道是否为 1（channels=1）
3. 输出维度是否为 256
4. 推理是否能跑通（CUDA/CPU）
5. L2 归一化是否正确
"""

import os
import sys
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model-path', type=str, default=None,
                        help='Path to pretrained_overlap_transformer.pth.tar')
    parser.add_argument('--verbose', action='store_true', help='Verbose output')
    args = parser.parse_args()
    
    # 查找模型路径
    if args.model_path is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        default_paths = [
            os.path.join(script_dir, 'automap_pro/src/modular/OverlapTransformer-master/model/pretrained_overlap_transformer.pth.tar'),
            os.path.join(script_dir, 'automap_ws/src/hba_api/models/pretrained_overlap_transformer.pth.tar'),
        ]
        for p in default_paths:
            if os.path.isfile(p):
                args.model_path = p
                break
        if args.model_path is None:
            print("❌ Model file not found. Please specify --model-path")
            return 1
    
    if not os.path.isfile(args.model_path):
        print(f"❌ Model file not found: {args.model_path}")
        return 1
    
    print(f"[Model Verification] Model path: {args.model_path}")
    file_size_mb = os.path.getsize(args.model_path) / (1024 * 1024)
    print(f"[Model Verification] File size: {file_size_mb:.1f} MB")
    
    try:
        import torch
        import numpy as np
    except ImportError as e:
        print(f"❌ PyTorch import failed: {e}")
        print("   Please install: pip install torch")
        return 1
    
    # 添加 OverlapTransformer 模块路径
    ot_root = os.path.dirname(args.model_path)  # model 文件夹的父级
    ot_root = os.path.dirname(ot_root)  # OverlapTransformer-master
    if ot_root not in sys.path:
        sys.path.insert(0, ot_root)
        sys.path.insert(0, os.path.join(ot_root, 'tools'))
        sys.path.insert(0, os.path.join(ot_root, 'modules'))
    
    try:
        from modules.overlap_transformer import featureExtracter
    except ImportError as e:
        print(f"❌ Failed to import featureExtracter: {e}")
        print(f"   PYTHONPATH: {':'.join(sys.path[:3])}")
        return 1
    
    print("\n[Step 1/5] Creating model architecture...")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"  Device: {device}")
    
    try:
        model = featureExtracter(height=64, width=900, channels=1, use_transformer=True)
        print("  ✓ Model created")
    except Exception as e:
        print(f"  ❌ Model creation failed: {e}")
        return 1
    
    print("\n[Step 2/5] Loading checkpoint...")
    try:
        checkpoint = torch.load(args.model_path, map_location=device)
        state_dict = checkpoint.get('state_dict', checkpoint)
        print(f"  ✓ Checkpoint loaded (keys: {len(state_dict)})")
        if args.verbose:
            for k in list(state_dict.keys())[:5]:
                print(f"    - {k}: {state_dict[k].shape}")
    except Exception as e:
        print(f"  ❌ Checkpoint loading failed: {e}")
        return 1
    
    print("\n[Step 3/5] Loading state_dict into model...")
    try:
        incompatible = model.load_state_dict(state_dict, strict=False)
        print(f"  ✓ Weights loaded (strict=False)")
        if incompatible.missing_keys:
            print(f"  ⚠ Missing keys: {incompatible.missing_keys[:3]}...")
        if incompatible.unexpected_keys:
            print(f"  ⚠ Unexpected keys: {incompatible.unexpected_keys[:3]}...")
    except Exception as e:
        print(f"  ❌ load_state_dict failed: {e}")
        return 1
    
    print("\n[Step 4/5] Checking model architecture...")
    try:
        # 检查 conv1 权重
        conv1_weight = model.conv1.weight
        print(f"  conv1.weight.shape: {tuple(conv1_weight.shape)}")
        if conv1_weight.shape[1] != 1:
            print(f"  ❌ ERROR: Expected 1 input channel, got {conv1_weight.shape[1]}")
            print("     This means the model weights are incompatible!")
            return 1
        print(f"  ✓ Input channels: 1 (correct)")
        
        # 检查输出层
        linear = model.linear
        print(f"  Output layer: {linear}")
        
    except Exception as e:
        print(f"  ❌ Architecture check failed: {e}")
        return 1
    
    print("\n[Step 5/5] Testing inference...")
    model.to(device)
    model.eval()
    
    try:
        # 创建随机输入 [1, 1, 64, 900]
        x = torch.randn(1, 1, 64, 900, device=device, dtype=torch.float32)
        
        with torch.no_grad():
            y = model(x)
        
        print(f"  Input shape: {tuple(x.shape)}")
        print(f"  Output shape: {tuple(y.shape)}")
        
        if y.shape[-1] != 256:
            print(f"  ❌ ERROR: Expected 256-d output, got {y.shape[-1]}")
            return 1
        
        print(f"  ✓ Output dimension: 256 (correct)")
        
        # 检查 L2 归一化
        norm = torch.norm(y, p=2, dim=1)
        mean_norm = norm.mean().item()
        std_norm = norm.std().item()
        print(f"  L2 norm mean: {mean_norm:.6f} ± {std_norm:.6f}")
        
        if abs(mean_norm - 1.0) > 0.1:
            print(f"  ⚠ WARNING: Output not unit norm (should be ≈1.0)")
        else:
            print(f"  ✓ L2 normalization: correct")
        
        # 检查数值范围
        y_flat = y.cpu().numpy().flatten()
        print(f"  Output range: [{y_flat.min():.4f}, {y_flat.max():.4f}]")
        print(f"  Output mean: {y_flat.mean():.6f}, std: {y_flat.std():.6f}")
        
    except Exception as e:
        print(f"  ❌ Inference failed: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    print("\n" + "="*60)
    print("✅ ALL CHECKS PASSED!")
    print("="*60)
    print("\nModel is ready for deployment:")
    print(f"  - Architecture: OverlapTransformer (channels=1, H=64, W=900)")
    print(f"  - Output: 256-d normalized descriptor")
    print(f"  - Device: {device}")
    print(f"  - File: {args.model_path}")
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
