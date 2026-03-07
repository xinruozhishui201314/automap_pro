#!/usr/bin/env python3
"""
绘制建图轨迹与 GPS 轨迹对比图，用于分析点云建图精度。

用法:
  python3 plot_trajectory_compare.py --odom trajectory_odom_20260307_204500.csv --gps trajectory_gps_20260307_204500.csv
  python3 plot_trajectory_compare.py --dir logs   # 自动查找该目录下最新 trajectory_odom_*.csv / trajectory_gps_*.csv
  python3 plot_trajectory_compare.py --dir logs --out compare.png

CSV 格式（由 AutoMapSystem 写入）:
  - trajectory_odom_*.csv: timestamp,x,y,z,qx,qy,qz,qw,pos_std_x,pos_std_y,pos_std_z
  - trajectory_gps_*.csv:  timestamp,x,y,z,frame
"""
import argparse
import glob
import os
import sys

try:
    import pandas as pd
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as e:
    print("Need: pip install pandas matplotlib", file=sys.stderr)
    raise SystemExit(1) from e


def find_latest_pair(log_dir: str):
    """在 log_dir 下找最新的一对 trajectory_odom_*.csv 与 trajectory_gps_*.csv（按 session 后缀匹配）。"""
    odom_files = sorted(glob.glob(os.path.join(log_dir, "trajectory_odom_*.csv")), key=os.path.getmtime, reverse=True)
    gps_files  = sorted(glob.glob(os.path.join(log_dir, "trajectory_gps_*.csv")),  key=os.path.getmtime, reverse=True)
    if not odom_files:
        return None, None
    # 取最新 odom 的 session 后缀，找同后缀的 gps
    base = os.path.basename(odom_files[0])
    # trajectory_odom_20260307_204500.csv -> 20260307_204500
    suffix = base.replace("trajectory_odom_", "").replace(".csv", "")
    gps_cand = os.path.join(log_dir, f"trajectory_gps_{suffix}.csv")
    odom_path = odom_files[0]
    gps_path  = gps_cand if os.path.isfile(gps_cand) else (gps_files[0] if gps_files else None)
    return odom_path, gps_path


def load_odom(path: str):
    df = pd.read_csv(path)
    df = df.rename(columns=str.strip)
    return df


def load_gps(path: str):
    df = pd.read_csv(path)
    df = df.rename(columns=str.strip)
    return df


def plot_compare(odom_df, gps_df, out_path: str, title_suffix: str = ""):
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    # 1) XY 平面轨迹
    ax = axes[0, 0]
    if odom_df is not None and len(odom_df):
        ax.plot(odom_df["x"], odom_df["y"], "b-", alpha=0.7, label="Odom (建图)", linewidth=0.8)
    if gps_df is not None and len(gps_df):
        ax.plot(gps_df["x"], gps_df["y"], "r.", markersize=2, alpha=0.8, label="GPS")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("XY 平面轨迹对比" + title_suffix)
    ax.legend(loc="best", fontsize=8)
    ax.axis("equal")
    ax.grid(True, alpha=0.3)

    # 2) X - time
    ax = axes[0, 1]
    if odom_df is not None and len(odom_df):
        ax.plot(odom_df["timestamp"], odom_df["x"], "b-", alpha=0.7, label="Odom x", linewidth=0.6)
    if gps_df is not None and len(gps_df):
        ax.plot(gps_df["timestamp"], gps_df["x"], "r.", markersize=2, alpha=0.8, label="GPS x")
    ax.set_xlabel("timestamp (s)")
    ax.set_ylabel("x (m)")
    ax.set_title("X - 时间" + title_suffix)
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)

    # 3) Y - time
    ax = axes[1, 0]
    if odom_df is not None and len(odom_df):
        ax.plot(odom_df["timestamp"], odom_df["y"], "b-", alpha=0.7, label="Odom y", linewidth=0.6)
    if gps_df is not None and len(gps_df):
        ax.plot(gps_df["timestamp"], gps_df["y"], "r.", markersize=2, alpha=0.8, label="GPS y")
    ax.set_xlabel("timestamp (s)")
    ax.set_ylabel("y (m)")
    ax.set_title("Y - 时间" + title_suffix)
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)

    # 4) Z - time
    ax = axes[1, 1]
    if odom_df is not None and len(odom_df):
        ax.plot(odom_df["timestamp"], odom_df["z"], "b-", alpha=0.7, label="Odom z", linewidth=0.6)
    if gps_df is not None and len(gps_df):
        ax.plot(gps_df["timestamp"], gps_df["z"], "r.", markersize=2, alpha=0.8, label="GPS z")
    ax.set_xlabel("timestamp (s)")
    ax.set_ylabel("z (m)")
    ax.set_title("Z - 时间" + title_suffix)
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if out_path:
        plt.savefig(out_path, dpi=150, bbox_inches="tight")
        print(f"[plot] Saved {out_path}")
    else:
        plt.show()
    plt.close()


def main():
    parser = argparse.ArgumentParser(description="建图轨迹与 GPS 轨迹对比图")
    parser.add_argument("--odom", default="", help="trajectory_odom_*.csv 路径")
    parser.add_argument("--gps",  default="", help="trajectory_gps_*.csv 路径")
    parser.add_argument("--dir",  default="", help="日志目录，自动查找最新一对 odom/gps CSV")
    parser.add_argument("--out",  default="trajectory_compare.png", help="输出图片路径")
    args = parser.parse_args()

    odom_path = args.odom.strip()
    gps_path  = args.gps.strip()
    if args.dir:
        odom_path, gps_path = find_latest_pair(args.dir)
        if not odom_path:
            print(f"[error] No trajectory_odom_*.csv in {args.dir}", file=sys.stderr)
            sys.exit(1)
        print(f"[info] Using odom={odom_path} gps={gps_path or '(none)'}")

    if not odom_path or not os.path.isfile(odom_path):
        print("[error] Missing or invalid --odom file", file=sys.stderr)
        sys.exit(1)

    odom_df = load_odom(odom_path)
    gps_df  = load_gps(gps_path) if gps_path and os.path.isfile(gps_path) else None
    n_odom = len(odom_df) if odom_df is not None else 0
    n_gps  = len(gps_df) if gps_df is not None else 0
    title_suffix = f" (odom={n_odom} pts, gps={n_gps} pts)"
    plot_compare(odom_df, gps_df, args.out, title_suffix)


if __name__ == "__main__":
    main()
