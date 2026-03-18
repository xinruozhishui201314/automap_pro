#!/usr/bin/env python3
"""
根据建图完成后保存的 deviation_curve.csv / accuracy_trajectories.csv 生成标定精度曲线图。
建图主进程会在时间戳目录下写入 CSV，并调用本脚本生成 accuracy_curves.png；也可手动调用。

用法:
  python3 plot_accuracy_curves.py --dir <时间戳目录>
  python3 plot_accuracy_curves.py --dir /path/to/automap_output/20260317_2137

依赖: pandas, matplotlib, numpy（与 plot_trajectory_compare.py 一致）
"""
import argparse
import os
import sys

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
except ImportError as e:
    print("Need: pip install matplotlib numpy", file=sys.stderr)
    raise SystemExit(1) from e

try:
    import pandas as pd
except ImportError:
    pd = None


def main():
    parser = argparse.ArgumentParser(description="Plot HBA vs GPS accuracy curves from CSV")
    parser.add_argument("--dir", required=True, help="Directory containing deviation_curve.csv or accuracy_trajectories.csv")
    parser.add_argument("--out", default=None, help="Output PNG path (default: <dir>/accuracy_curves.png)")
    args = parser.parse_args()
    dir_path = os.path.abspath(args.dir)
    if not os.path.isdir(dir_path):
        print(f"[plot_accuracy_curves] Not a directory: {dir_path}", file=sys.stderr)
        sys.exit(1)

    out_path = args.out or os.path.join(dir_path, "accuracy_curves.png")

    # 优先使用 deviation_curve.csv（两列：cum_dist_m, deviation_m）
    dev_csv = os.path.join(dir_path, "deviation_curve.csv")
    traj_csv = os.path.join(dir_path, "accuracy_trajectories.csv")

    cum_dist = None
    deviation = None

    if os.path.isfile(dev_csv):
        if pd is None:
            # 无 pandas 时简单按行解析
            cum_dist, deviation = [], []
            with open(dev_csv) as f:
                next(f)  # header
                for line in f:
                    parts = line.strip().split(",")
                    if len(parts) >= 2:
                        try:
                            cum_dist.append(float(parts[0]))
                            deviation.append(float(parts[1]))
                        except ValueError:
                            pass
            cum_dist = np.array(cum_dist) if cum_dist else None
            deviation = np.array(deviation) if deviation else None
        else:
            df = pd.read_csv(dev_csv)
            if "cum_dist_m" in df.columns and "deviation_m" in df.columns:
                cum_dist = df["cum_dist_m"].values
                deviation = df["deviation_m"].values
    if cum_dist is None and os.path.isfile(traj_csv) and pd is not None:
        df = pd.read_csv(traj_csv)
        if "cum_dist_m" in df.columns and "deviation_m" in df.columns:
            cum_dist = df["cum_dist_m"].values
            deviation = df["deviation_m"].values
        elif "deviation_m" in df.columns:
            deviation = df["deviation_m"].values
            cum_dist = np.arange(len(deviation), dtype=float)

    if cum_dist is None or deviation is None or len(cum_dist) == 0:
        print(f"[plot_accuracy_curves] No data in {dir_path} (need deviation_curve.csv or accuracy_trajectories.csv)", file=sys.stderr)
        sys.exit(1)

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # 上图：偏差随累积里程
    axes[0].plot(cum_dist, deviation, "r-", linewidth=1.0, label="Deviation (m)")
    axes[0].set_ylabel("Deviation (m)")
    axes[0].set_title("HBA vs GPS: deviation along trajectory")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)

    # 下图：偏差直方图
    axes[1].hist(deviation, bins=min(50, max(10, len(deviation) // 5)), color="steelblue", edgecolor="black", alpha=0.7)
    axes[1].set_xlabel("Deviation (m)")
    axes[1].set_ylabel("Count")
    axes[1].set_title("Deviation distribution")
    axes[1].grid(True, alpha=0.3)

    mean_dev = float(np.mean(deviation))
    max_dev = float(np.max(deviation))
    fig.suptitle(f"HBA vs GPS accuracy (mean={mean_dev:.3f}m max={max_dev:.3f}m)", fontsize=12)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"[plot_accuracy_curves] saved {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
