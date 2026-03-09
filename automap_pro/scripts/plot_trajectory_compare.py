#!/usr/bin/env python3
"""
绘制建图轨迹与 GPS 轨迹对比图，用于分析点云建图精度。

用法:
  python3 plot_trajectory_compare.py trajectory_odom_xxx.csv --out compare.png
  python3 plot_trajectory_compare.py --odom trajectory_odom_xxx.csv --out compare.png
  python3 plot_trajectory_compare.py --dir logs   # 自动查找该目录下最新 trajectory_odom_*.csv
  python3 plot_trajectory_compare.py --dir logs --out compare.png

CSV 格式（由 AutoMapSystem 写入）:
  - trajectory_odom_*.csv（新格式）: ...,gps_x,gps_y,gps_z,gps_frame,gps_valid,gps_hdop,gps_quality
    gps_quality: 0=INVALID 1=LOW 2=MEDIUM 3=HIGH 4=EXCELLENT；有匹配即记录，不要求 MEDIUM
  - trajectory_odom_*.csv（旧格式）: timestamp,x,y,z,qx,qy,qz,qw,pos_std_x,pos_std_y,pos_std_z
  - trajectory_gps_*.csv（可选）: timestamp,x,y,z,frame

注意：新格式已将GPS信息集成到odom文件中，无需单独指定GPS文件。
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
    import matplotlib.font_manager as fm
    import numpy as np
except ImportError as e:
    print("Need: pip install pandas matplotlib numpy", file=sys.stderr)
    raise SystemExit(1) from e

# 中文字体：优先使用系统内支持 CJK 的字体，避免 "Glyph missing from DejaVu Sans" 警告
_CJK_FONTS = [
    "Noto Sans CJK SC",
    "Noto Sans SC",
    "WenQuanYi Micro Hei",
    "WenQuanYi Zen Hei",
    "SimHei",
    "Microsoft YaHei",
    "DejaVu Sans",
]


def _setup_chinese_font():
    """若系统有中文字体则设置，否则抑制缺失字形警告。"""
    available = [f.name for f in fm.fontManager.ttflist]
    for name in _CJK_FONTS:
        if name in available:
            plt.rcParams["font.sans-serif"] = [name] + plt.rcParams["font.sans-serif"]
            return
    import warnings
    warnings.filterwarnings("ignore", category=UserWarning, message=".*Glyph.*missing from font.*")


_setup_chinese_font()


def find_latest_odom(log_dir: str):
    """在 log_dir 下找最新的 trajectory_odom_*.csv。"""
    odom_files = sorted(glob.glob(os.path.join(log_dir, "trajectory_odom_*.csv")), 
                       key=os.path.getmtime, reverse=True)
    if not odom_files:
        return None
    return odom_files[0]


def load_odom(path: str):
    """加载里程计数据，支持新旧格式"""
    df = pd.read_csv(path)
    df = df.rename(columns=str.strip)
    return df


def plot_compare(odom_df, out_path: str, title_suffix: str = ""):
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    # 检查是否有GPS列（新格式含 gps_x,y,z；可选 gps_valid, gps_hdop, gps_quality）
    has_gps = all(col in odom_df.columns for col in ['gps_x', 'gps_y', 'gps_z'])
    # 有 GPS 数据的行：优先 gps_valid==1（MEDIUM+），否则取 gps_x/gps_y 非零（含 LOW 等）
    if has_gps:
        gps_med = odom_df[odom_df.get('gps_valid', 0) == 1] if 'gps_valid' in odom_df.columns else pd.DataFrame()
        gps_any = odom_df[(odom_df["gps_x"] != 0) | (odom_df["gps_y"] != 0)] if has_gps else pd.DataFrame()
        gps_plot = gps_med if len(gps_med) > 0 else gps_any
    else:
        gps_plot = pd.DataFrame()

    # 1) XY 平面轨迹
    ax = axes[0, 0]
    if odom_df is not None and len(odom_df):
        ax.plot(odom_df["x"], odom_df["y"], "b-", alpha=0.7, 
                label="Odom (建图)", linewidth=0.8)
        if has_gps and len(gps_plot) > 0:
            ax.plot(gps_plot["gps_x"], gps_plot["gps_y"], "r.", 
                    markersize=2, alpha=0.8, label="GPS" + (" (MEDIUM+)" if len(gps_med) > 0 else " (all)"))
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("XY 平面轨迹对比" + title_suffix)
    ax.legend(loc="best", fontsize=8)
    ax.axis("equal")
    ax.grid(True, alpha=0.3)

    # 2) X - time
    ax = axes[0, 1]
    if odom_df is not None and len(odom_df):
        ax.plot(odom_df["timestamp"], odom_df["x"], "b-", alpha=0.7, 
                label="Odom x", linewidth=0.6)
        if has_gps and len(gps_plot) > 0:
            ax.plot(gps_plot["timestamp"], gps_plot["gps_x"], "r.", 
                    markersize=2, alpha=0.8, label="GPS x")
    ax.set_xlabel("timestamp (s)")
    ax.set_ylabel("x (m)")
    ax.set_title("X - 时间" + title_suffix)
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)

    # 3) Y - time
    ax = axes[1, 0]
    if odom_df is not None and len(odom_df):
        ax.plot(odom_df["timestamp"], odom_df["y"], "b-", alpha=0.7, 
                label="Odom y", linewidth=0.6)
        if has_gps:
            gps_valid = odom_df[odom_df['gps_valid'] == 1]
            if len(gps_valid) > 0:
                ax.plot(gps_valid["timestamp"], gps_valid["gps_y"], "r.", 
                        markersize=2, alpha=0.8, label="GPS y")
    ax.set_xlabel("timestamp (s)")
    ax.set_ylabel("y (m)")
    ax.set_title("Y - 时间" + title_suffix)
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)

    # 4) Z - time
    ax = axes[1, 1]
    if odom_df is not None and len(odom_df):
        ax.plot(odom_df["timestamp"], odom_df["z"], "b-", alpha=0.7, 
                label="Odom z", linewidth=0.6)
        if has_gps:
            gps_valid = odom_df[odom_df['gps_valid'] == 1]
            if len(gps_valid) > 0:
                ax.plot(gps_valid["timestamp"], gps_valid["gps_z"], "r.", 
                        markersize=2, alpha=0.8, label="GPS z")
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


def print_statistics(odom_df):
    """打印偏差统计信息"""
    if not all(col in odom_df.columns for col in ['gps_x', 'gps_y', 'gps_z', 'gps_valid']):
        print("[info] Old CSV format (no GPS columns), skipping statistics")
        return
    
    gps_valid = odom_df[odom_df['gps_valid'] == 1].copy()
    if len(gps_valid) == 0:
        print("[info] No valid GPS points, skipping statistics")
        return
    
    # 计算距离偏差
    gps_valid['dx'] = gps_valid['x'] - gps_valid['gps_x']
    gps_valid['dy'] = gps_valid['y'] - gps_valid['gps_y']
    gps_valid['dz'] = gps_valid['z'] - gps_valid['gps_z']
    gps_valid['dist'] = np.sqrt(gps_valid['dx']**2 + 
                                  gps_valid['dy']**2 + 
                                  gps_valid['dz']**2)
    
    print("\n=== 里程计 vs GPS 偏差统计 ===")
    print(f"有效GPS点数: {len(gps_valid)}")
    print(f"平均偏差:     {gps_valid['dist'].mean():.3f} m")
    print(f"中位数偏差:   {gps_valid['dist'].median():.3f} m")
    print(f"标准差:       {gps_valid['dist'].std():.3f} m")
    print(f"最大偏差:     {gps_valid['dist'].max():.3f} m")
    print(f"最小偏差:     {gps_valid['dist'].min():.3f} m")
    print(f"X方向偏差:   {gps_valid['dx'].mean():.3f} m (std: {gps_valid['dx'].std():.3f})")
    print(f"Y方向偏差:   {gps_valid['dy'].mean():.3f} m (std: {gps_valid['dy'].std():.3f})")
    print(f"Z方向偏差:   {gps_valid['dz'].mean():.3f} m (std: {gps_valid['dz'].std():.3f})")
    print("==================================\n")


def main():
    parser = argparse.ArgumentParser(description="建图轨迹与 GPS 轨迹对比图")
    parser.add_argument("odom_file", nargs="?", default="", help="trajectory_odom_*.csv 路径（可选，也可用 --odom）")
    parser.add_argument("--odom", default="", help="trajectory_odom_*.csv 路径")
    parser.add_argument("--gps", default="", help="trajectory_gps_*.csv 路径（新格式中不使用）")
    parser.add_argument("--dir", default="", help="日志目录，自动查找最新的 odom CSV")
    parser.add_argument("--out", default="trajectory_compare.png", help="输出图片路径")
    parser.add_argument("--stats", action="store_true", help="打印偏差统计信息")
    args = parser.parse_args()

    odom_path = (args.odom_file or args.odom).strip()
    if args.dir:
        odom_path = find_latest_odom(args.dir)
        if not odom_path:
            print(f"[error] No trajectory_odom_*.csv in {args.dir}", file=sys.stderr)
            sys.exit(1)
        print(f"[info] Using odom={odom_path}")

    if not odom_path or not os.path.isfile(odom_path):
        print("[error] Missing or invalid --odom file", file=sys.stderr)
        sys.exit(1)

    odom_df = load_odom(odom_path)
    
    # 打印统计信息
    if args.stats:
        print_statistics(odom_df)
    
    # 检查新格式
    has_gps = all(col in odom_df.columns for col in ['gps_x', 'gps_y', 'gps_z', 'gps_valid'])
    if has_gps:
        print("[info] Using new CSV format (GPS columns in odom file)")
        if args.gps:
            print("[warn] --gps parameter ignored (GPS already in odom file)", file=sys.stderr)
    else:
        print("[info] Using old CSV format (no GPS columns)")
        # 旧格式需要GPS文件
        gps_path = args.gps.strip()
        if args.dir and not gps_path:
            # 尝试在相同目录找GPS文件
            gps_files = sorted(glob.glob(os.path.join(args.dir, "trajectory_gps_*.csv")), 
                               key=os.path.getmtime, reverse=True)
            if gps_files:
                gps_path = gps_files[0]
                print(f"[info] Auto-detected GPS file: {gps_path}")
        
        if not gps_path or not os.path.isfile(gps_path):
            print("[error] Old CSV format requires --gps file", file=sys.stderr)
            sys.exit(1)
        
        gps_df = pd.read_csv(gps_path)
        gps_df = gps_df.rename(columns=str.strip)
        
        # 合并到odom_df
        odom_df['gps_x'] = np.nan
        odom_df['gps_y'] = np.nan
        odom_df['gps_z'] = np.nan
        odom_df['gps_frame'] = ''
        odom_df['gps_valid'] = 0
        
        # 基于时间戳匹配
        for idx, row in gps_df.iterrows():
            # 找到最接近的odom时间戳
            ts_diff = abs(odom_df['timestamp'] - row['timestamp'])
            closest_idx = ts_diff.idxmin()
            if ts_diff.iloc[closest_idx] < 0.1:  # 100ms以内
                odom_df.loc[closest_idx, 'gps_x'] = row['x']
                odom_df.loc[closest_idx, 'gps_y'] = row['y']
                odom_df.loc[closest_idx, 'gps_z'] = row['z']
                odom_df.loc[closest_idx, 'gps_frame'] = row['frame']
                odom_df.loc[closest_idx, 'gps_valid'] = 1
    
    n_odom = len(odom_df) if odom_df is not None else 0
    n_gps = len(odom_df[odom_df['gps_valid'] == 1]) if has_gps and odom_df is not None else 0
    title_suffix = f" (odom={n_odom} pts, gps={n_gps} pts)"
    plot_compare(odom_df, args.out, title_suffix)


if __name__ == "__main__":
    main()
