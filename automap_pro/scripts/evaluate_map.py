#!/usr/bin/env python3
"""Evaluate global map quality: point density, flatness, coverage.
Usage:
    ./evaluate_map.py --map global_map.pcd [--ref reference.pcd]
"""
import argparse
import os
import numpy as np


def load_pcd_xyz(path):
    """Simple PCD ASCII loader."""
    pts = []
    with open(path, 'r') as f:
        data_started = False
        for line in f:
            if line.startswith("DATA"):
                data_started = True
                continue
            if data_started:
                vals = line.strip().split()
                if len(vals) >= 3:
                    try:
                        pts.append([float(v) for v in vals[:3]])
                    except ValueError:
                        pass
    return np.array(pts)


def compute_stats(pts):
    if len(pts) == 0:
        return {}
    bbox_min = pts.min(axis=0)
    bbox_max = pts.max(axis=0)
    extent   = bbox_max - bbox_min
    volume   = np.prod(extent)
    density  = len(pts) / max(volume, 1e-6)
    return {
        "num_points": len(pts),
        "bbox_min":   bbox_min.tolist(),
        "bbox_max":   bbox_max.tolist(),
        "extent_xyz": extent.tolist(),
        "density_pts_m3": float(density),
    }


def main():
    parser = argparse.ArgumentParser(description="AutoMap-Pro Map Evaluation")
    parser.add_argument("--map",    required=True,  help="Global map PCD file")
    parser.add_argument("--ref",    default="",     help="Reference map PCD (optional)")
    parser.add_argument("--outdir", default="eval_map", help="Output directory")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    print(f"[eval_map] Loading map: {args.map}")
    pts = load_pcd_xyz(args.map)
    if len(pts) == 0:
        print("[eval_map] No points loaded. Is this a binary PCD? Use pcl_converter first.")
        return

    stats = compute_stats(pts)
    print("\n=== Map Statistics ===")
    for k, v in stats.items():
        print(f"  {k}: {v}")

    if args.ref:
        print(f"\n[eval_map] Loading reference: {args.ref}")
        ref_pts = load_pcd_xyz(args.ref)
        if len(ref_pts) > 0:
            # Point-to-point distance (random sample)
            sample_size = min(10000, len(pts))
            idx = np.random.choice(len(pts), sample_size, replace=False)
            sample = pts[idx]

            from scipy.spatial import KDTree
            tree = KDTree(ref_pts)
            dists, _ = tree.query(sample, k=1)
            print(f"\n  C2C distance (sampled {sample_size} pts):")
            print(f"    Mean: {dists.mean():.4f} m")
            print(f"    Median: {np.median(dists):.4f} m")
            print(f"    P95: {np.percentile(dists, 95):.4f} m")
            print(f"    Max: {dists.max():.4f} m")

    import json
    out_path = os.path.join(args.outdir, "map_stats.json")
    with open(out_path, 'w') as f:
        json.dump(stats, f, indent=2)
    print(f"\n[eval_map] Stats saved to {out_path}")


if __name__ == "__main__":
    main()
