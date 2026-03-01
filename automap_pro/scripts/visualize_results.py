#!/usr/bin/env python3
"""Visualize mapping results: trajectory, loop closures, submap centers.
Usage:
    ./visualize_results.py --output_dir /data/automap_output [--3d]
"""
import argparse
import os
import json
import numpy as np

try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("[visualize] matplotlib not found. Install with: pip install matplotlib")


def load_tum_trajectory(path):
    """Load TUM format trajectory. Returns (timestamps, positions)."""
    ts, pos = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            vals = line.split()
            if len(vals) >= 4:
                ts.append(float(vals[0]))
                pos.append([float(vals[1]), float(vals[2]), float(vals[3])])
    return np.array(ts), np.array(pos)


def load_loop_report(path):
    with open(path) as f:
        return json.load(f)


def load_keyframe_poses(path):
    with open(path) as f:
        return json.load(f)


def main():
    parser = argparse.ArgumentParser(description="AutoMap-Pro Results Visualization")
    parser.add_argument("--output_dir", required=True, help="AutoMap output directory")
    parser.add_argument("--3d",         action="store_true", dest="show_3d", help="3D plot")
    parser.add_argument("--save",       default="", help="Save plot to file (PNG)")
    args = parser.parse_args()

    if not HAS_MATPLOTLIB:
        return

    traj_path = os.path.join(args.output_dir, "trajectory", "optimized_trajectory_tum.txt")
    raw_path  = os.path.join(args.output_dir, "trajectory", "raw_trajectory_tum.txt")
    loop_path = os.path.join(args.output_dir, "loop_closures", "loop_report.json")
    kf_path   = os.path.join(args.output_dir, "trajectory", "keyframe_poses.json")

    fig = plt.figure(figsize=(14, 10))

    if args.show_3d:
        ax = fig.add_subplot(111, projection='3d')
        label_x, label_y, label_z = 'X (m)', 'Y (m)', 'Z (m)'
    else:
        ax = fig.add_subplot(111)
        label_x, label_y = 'X (m)', 'Y (m)'

    # Plot raw trajectory
    if os.path.exists(raw_path):
        _, raw_pos = load_tum_trajectory(raw_path)
        if len(raw_pos) > 0:
            if args.show_3d:
                ax.plot(raw_pos[:,0], raw_pos[:,1], raw_pos[:,2],
                        'b-', alpha=0.4, linewidth=0.8, label='Raw odometry')
            else:
                ax.plot(raw_pos[:,0], raw_pos[:,1],
                        'b-', alpha=0.4, linewidth=0.8, label='Raw odometry')

    # Plot optimized trajectory
    if os.path.exists(traj_path):
        _, opt_pos = load_tum_trajectory(traj_path)
        if len(opt_pos) > 0:
            if args.show_3d:
                ax.plot(opt_pos[:,0], opt_pos[:,1], opt_pos[:,2],
                        'g-', linewidth=1.5, label='Optimized trajectory')
                ax.scatter(opt_pos[0,0], opt_pos[0,1], opt_pos[0,2],
                           c='green', s=80, marker='o', zorder=5, label='Start')
                ax.scatter(opt_pos[-1,0], opt_pos[-1,1], opt_pos[-1,2],
                           c='red', s=80, marker='*', zorder=5, label='End')
            else:
                ax.plot(opt_pos[:,0], opt_pos[:,1],
                        'g-', linewidth=1.5, label='Optimized trajectory')
                ax.scatter(opt_pos[0,0], opt_pos[0,1],
                           c='green', s=80, marker='o', zorder=5, label='Start')
                ax.scatter(opt_pos[-1,0], opt_pos[-1,1],
                           c='red', s=80, marker='*', zorder=5, label='End')

    # Plot loop closures
    if os.path.exists(loop_path) and os.path.exists(kf_path):
        loops = load_loop_report(loop_path)
        kfs   = load_keyframe_poses(kf_path)

        # Build submap center lookup (approximate from keyframes)
        sm_centers = {}
        for kf in kfs:
            sid = kf.get("submap_id", -1)
            if sid >= 0:
                p = kf["position"]
                if sid not in sm_centers:
                    sm_centers[sid] = []
                sm_centers[sid].append(p)
        sm_pos = {sid: np.mean(pts, axis=0) for sid, pts in sm_centers.items()}

        for lc in loops:
            i, j = lc["submap_i"], lc["submap_j"]
            if i in sm_pos and j in sm_pos:
                pi, pj = sm_pos[i], sm_pos[j]
                color = 'orange' if lc.get("is_inter_session") else 'red'
                if args.show_3d:
                    ax.plot([pi[0], pj[0]], [pi[1], pj[1]], [pi[2], pj[2]],
                            color=color, alpha=0.5, linewidth=0.6)
                else:
                    ax.plot([pi[0], pj[0]], [pi[1], pj[1]],
                            color=color, alpha=0.5, linewidth=0.6)

        # Dummy artists for legend
        from matplotlib.lines import Line2D
        ax.legend(handles=[
            *ax.lines[:3],
            Line2D([0],[0], color='red',    linewidth=1, label=f'Loops ({len(loops)})'),
            Line2D([0],[0], color='orange', linewidth=1, label='Inter-session loops'),
        ], loc='best')
    else:
        ax.legend(loc='best')

    ax.set_xlabel(label_x)
    ax.set_ylabel(label_y)
    if args.show_3d: ax.set_zlabel(label_z)
    ax.set_title("AutoMap-Pro Mapping Results")
    ax.set_aspect('equal' if not args.show_3d else 'auto')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if args.save:
        plt.savefig(args.save, dpi=150, bbox_inches='tight')
        print(f"[visualize] Saved to {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
