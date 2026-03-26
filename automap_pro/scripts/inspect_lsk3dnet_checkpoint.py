#!/usr/bin/env python3
import torch
import os
import sys
import argparse

def main():
    parser = argparse.ArgumentParser(description="Inspect LSK3DNet checkpoint weights.")
    parser.add_argument("--ckpt", type=str, help="Path to the checkpoint .pt file")
    args = parser.parse_args()

    ckpt_path = args.ckpt
    
    if not ckpt_path:
        # Default paths to try
        candidates = [
            "/workspace/automap_pro/automap_pro/models/LSK3DNET/opensource_9ks_s030_w64_0.pt", # Container path
            os.path.join(os.path.dirname(__file__), "../models/LSK3DNET/opensource_9ks_s030_w64_0.pt"), # Relative to script
            "/home/wqs/Documents/github/automap_pro/automap_pro/models/LSK3DNET/opensource_9ks_s030_w64_0.pt" # Host path
        ]
        for c in candidates:
            if os.path.exists(c):
                ckpt_path = c
                break

    if not ckpt_path or not os.path.exists(ckpt_path):
        print(f"Error: Checkpoint not found. Searched: {ckpt_path if ckpt_path else 'default paths'}")
        sys.exit(1)

    print(f"Inspecting checkpoint: {ckpt_path}")
    try:
        ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=False)
        if "checkpoint" in ckpt:
            sd = ckpt["checkpoint"]
            print(f"Found 'checkpoint' key with {len(sd)} parameters.")
            for k in sorted(sd.keys()):
                if "classifier.0.weight" in k:
                    print(f"{k} shape: {sd[k].shape}")
        else:
            print("No 'checkpoint' key found in the file.")
            # Maybe it's a direct state dict
            for k in sorted(ckpt.keys()):
                if "classifier.0.weight" in k:
                    print(f"{k} shape: {ckpt[k].shape}")
    except Exception as e:
        print(f"Failed to load checkpoint: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
