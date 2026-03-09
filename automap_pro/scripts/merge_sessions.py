#!/usr/bin/env python3
"""Merge multiple mapping sessions into a unified map.
Usage:
    ./merge_sessions.py --sessions /data/session0 /data/session1 --output /data/merged
"""
import argparse
import os
import json
import subprocess


def merge_descriptor_dbs(session_dirs, output_dir):
    """Concatenate descriptor databases from multiple sessions."""
    all_entries = []
    for sd in session_dirs:
        db_path = os.path.join(sd, "descriptor_db.json")
        if os.path.exists(db_path):
            with open(db_path) as f:
                entries = json.load(f)
            all_entries.extend(entries)
            print(f"  Loaded {len(entries)} descriptors from {db_path}")

    out_path = os.path.join(output_dir, "descriptor_db_merged.json")
    with open(out_path, 'w') as f:
        json.dump(all_entries, f, indent=2)
    print(f"  Merged {len(all_entries)} descriptors → {out_path}")
    return out_path


def merge_trajectories(session_dirs, output_dir):
    """Concatenate TUM trajectory files."""
    all_lines = []
    for sd in session_dirs:
        traj_path = os.path.join(sd, "trajectory", "optimized_trajectory_tum.txt")
        if os.path.exists(traj_path):
            with open(traj_path) as f:
                all_lines.extend(f.readlines())
            print(f"  Loaded trajectory: {traj_path}")

    # Sort by timestamp
    all_lines.sort(key=lambda l: float(l.split()[0]) if l.strip() else 0)

    out_path = os.path.join(output_dir, "merged_trajectory_tum.txt")
    with open(out_path, 'w') as f:
        f.writelines(all_lines)
    print(f"  Merged {len(all_lines)} poses → {out_path}")


def merge_loop_reports(session_dirs, output_dir):
    all_loops = []
    for sd in session_dirs:
        lp = os.path.join(sd, "loop_closures", "loop_report.json")
        if os.path.exists(lp):
            with open(lp) as f:
                all_loops.extend(json.load(f))
    out = os.path.join(output_dir, "merged_loop_report.json")
    with open(out, 'w') as f:
        json.dump(all_loops, f, indent=2)
    print(f"  Merged {len(all_loops)} loop constraints → {out}")


def main():
    parser = argparse.ArgumentParser(description="AutoMap-Pro Session Merger")
    parser.add_argument("--sessions", nargs="+", required=True, help="Session directories")
    parser.add_argument("--output",   required=True,            help="Output directory")
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)
    os.makedirs(os.path.join(args.output, "trajectory"), exist_ok=True)
    os.makedirs(os.path.join(args.output, "loop_closures"), exist_ok=True)

    print(f"[merge_sessions] Merging {len(args.sessions)} sessions → {args.output}")

    merge_descriptor_dbs(args.sessions, args.output)
    merge_trajectories(args.sessions, args.output)
    merge_loop_reports(args.sessions, args.output)

    # Write session metadata
    meta = {
        "merged_sessions": args.sessions,
        "output_dir": args.output,
        "num_sessions": len(args.sessions),
    }
    with open(os.path.join(args.output, "merge_meta.json"), 'w') as f:
        json.dump(meta, f, indent=2)

    print("\n[merge_sessions] Done. Use automap_incremental.launch with prev_session to re-optimize.")


if __name__ == "__main__":
    main()
