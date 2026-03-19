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

    def plot_deviation_curve(cum_dist_arr, dev_arr, out_file, ylabel, title_suffix, color="r"):
        """绘制偏差曲线 + 直方图，与主图风格一致。"""
        fig2, ax2 = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        ax2[0].plot(cum_dist_arr, dev_arr, color=color, linestyle="-", linewidth=1.0, label=ylabel)
        ax2[0].axhline(y=0, color="gray", linestyle="--", alpha=0.5)
        ax2[0].set_ylabel(ylabel + " (m)")
        ax2[0].set_title(f"HBA vs GPS: {title_suffix} along trajectory")
        ax2[0].legend(loc="upper right")
        ax2[0].grid(True, alpha=0.3)
        ax2[1].hist(dev_arr, bins=min(50, max(10, len(dev_arr) // 5)), color="steelblue", edgecolor="black", alpha=0.7)
        ax2[1].axvline(x=0, color="gray", linestyle="--", alpha=0.5)
        ax2[1].set_xlabel(ylabel + " (m)")
        ax2[1].set_ylabel("Count")
        ax2[1].set_title(f"{title_suffix} distribution")
        ax2[1].grid(True, alpha=0.3)
        mean_d = float(np.mean(dev_arr))
        std_d = float(np.std(dev_arr))
        fig2.suptitle(f"HBA vs GPS {title_suffix} (mean={mean_d:.3f}m std={std_d:.3f}m)", fontsize=12)
        plt.tight_layout()
        plt.savefig(out_file, dpi=150, bbox_inches="tight")
        plt.close()

    # 1) 总偏差图：deviation_m vs cum_dist（原有）
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(cum_dist, deviation, "r-", linewidth=1.0, label="Deviation (m)")
    axes[0].set_ylabel("Deviation (m)")
    axes[0].set_title("HBA vs GPS: deviation along trajectory")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)
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

    # 2) gps_x-hba_x、gps_y-hba_y、gps_z-hba_z 偏差图：需 accuracy_trajectories.csv 中对应列
    traj_path = os.path.join(dir_path, "accuracy_trajectories.csv")
    if os.path.isfile(traj_path):
        if pd is not None:
            df_t = pd.read_csv(traj_path)
            req = ["hba_x_m", "gps_x_m", "hba_y_m", "gps_y_m", "hba_z_m", "gps_z_m", "cum_dist_m"]
            if all(c in df_t.columns for c in req):
                cum = df_t["cum_dist_m"].values
                dev_x = (df_t["gps_x_m"] - df_t["hba_x_m"]).values
                dev_y = (df_t["gps_y_m"] - df_t["hba_y_m"]).values
                dev_z = (df_t["gps_z_m"] - df_t["hba_z_m"]).values
                out_x = os.path.join(dir_path, "accuracy_curves_x.png")
                out_y = os.path.join(dir_path, "accuracy_curves_y.png")
                out_z = os.path.join(dir_path, "accuracy_curves_z.png")
                plot_deviation_curve(cum, dev_x, out_x, "X deviation (gps_x - hba_x)", "x deviation", color="C0")
                print(f"[plot_accuracy_curves] saved {out_x}")
                plot_deviation_curve(cum, dev_y, out_y, "Y deviation (gps_y - hba_y)", "y deviation", color="C1")
                print(f"[plot_accuracy_curves] saved {out_y}")
                plot_deviation_curve(cum, dev_z, out_z, "Z deviation (gps_z - hba_z)", "z deviation", color="C2")
                print(f"[plot_accuracy_curves] saved {out_z}")
            else:
                print(f"[plot_accuracy_curves] accuracy_trajectories.csv missing columns for x/y/z curves, skip", file=sys.stderr)
        else:
            cum, hba_x, gps_x, hba_y, gps_y, hba_z, gps_z = [], [], [], [], [], [], []
            with open(traj_path) as f:
                header = next(f).strip().split(",")
                idx_cum = header.index("cum_dist_m") if "cum_dist_m" in header else -1
                idx_hx = header.index("hba_x_m") if "hba_x_m" in header else -1
                idx_gx = header.index("gps_x_m") if "gps_x_m" in header else -1
                idx_hy = header.index("hba_y_m") if "hba_y_m" in header else -1
                idx_gy = header.index("gps_y_m") if "gps_y_m" in header else -1
                idx_hz = header.index("hba_z_m") if "hba_z_m" in header else -1
                idx_gz = header.index("gps_z_m") if "gps_z_m" in header else -1
                max_idx = max(idx_cum, idx_hx, idx_gx, idx_hy, idx_gy, idx_hz, idx_gz)
                if idx_cum < 0 or idx_hx < 0 or idx_gx < 0 or idx_hy < 0 or idx_gy < 0 or idx_hz < 0 or idx_gz < 0:
                    print(f"[plot_accuracy_curves] accuracy_trajectories.csv missing columns for x/y/z curves, skip", file=sys.stderr)
                else:
                    for line in f:
                        p = line.strip().split(",")
                        if len(p) > max_idx:
                            try:
                                cum.append(float(p[idx_cum]))
                                hba_x.append(float(p[idx_hx]))
                                gps_x.append(float(p[idx_gx]))
                                hba_y.append(float(p[idx_hy]))
                                gps_y.append(float(p[idx_gy]))
                                hba_z.append(float(p[idx_hz]))
                                gps_z.append(float(p[idx_gz]))
                            except ValueError:
                                pass
                    if cum:
                        cum = np.array(cum)
                        dev_x = np.array(gps_x) - np.array(hba_x)
                        dev_y = np.array(gps_y) - np.array(hba_y)
                        dev_z = np.array(gps_z) - np.array(hba_z)
                        out_x = os.path.join(dir_path, "accuracy_curves_x.png")
                        out_y = os.path.join(dir_path, "accuracy_curves_y.png")
                        out_z = os.path.join(dir_path, "accuracy_curves_z.png")
                        plot_deviation_curve(cum, dev_x, out_x, "X deviation (gps_x - hba_x)", "x deviation", color="C0")
                        print(f"[plot_accuracy_curves] saved {out_x}")
                        plot_deviation_curve(cum, dev_y, out_y, "Y deviation (gps_y - hba_y)", "y deviation", color="C1")
                        print(f"[plot_accuracy_curves] saved {out_y}")
                        plot_deviation_curve(cum, dev_z, out_z, "Z deviation (gps_z - hba_z)", "z deviation", color="C2")
                        print(f"[plot_accuracy_curves] saved {out_z}")

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
