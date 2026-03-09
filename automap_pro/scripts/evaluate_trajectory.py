#!/usr/bin/env python3
"""Evaluate trajectory accuracy using evo library.
Usage:
    ./evaluate_trajectory.py --est <estimated.txt> --ref <groundtruth.txt> [--format tum|kitti]
"""
import argparse
import subprocess
import sys
import os


def run(cmd):
    print(f"[eval] Running: {cmd}")
    ret = subprocess.run(cmd, shell=True)
    return ret.returncode


def main():
    parser = argparse.ArgumentParser(description="AutoMap-Pro Trajectory Evaluation")
    parser.add_argument("--est",    required=True,  help="Estimated trajectory file")
    parser.add_argument("--ref",    required=True,  help="Ground-truth trajectory file")
    parser.add_argument("--format", default="tum",  help="tum / kitti")
    parser.add_argument("--outdir", default="eval_results", help="Output directory")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    if args.format == "tum":
        # APE (Absolute Pose Error)
        run(f"evo_ape tum {args.ref} {args.est} -p --plot_mode=xz "
            f"--save_results {args.outdir}/ape_result.zip --save_plot {args.outdir}/ape.png")

        # RPE (Relative Pose Error)
        run(f"evo_rpe tum {args.ref} {args.est} -p --plot_mode=xz "
            f"--save_results {args.outdir}/rpe_result.zip --save_plot {args.outdir}/rpe.png")

    elif args.format == "kitti":
        run(f"evo_ape kitti {args.ref} {args.est} -p --plot_mode=xz "
            f"--save_results {args.outdir}/ape_result.zip --save_plot {args.outdir}/ape.png")

        run(f"evo_rpe kitti {args.ref} {args.est} -p --plot_mode=xz "
            f"--save_results {args.outdir}/rpe_result.zip --save_plot {args.outdir}/rpe.png")
    else:
        print(f"Unknown format: {args.format}")
        sys.exit(1)

    print(f"\n[eval] Results saved to {args.outdir}/")


if __name__ == "__main__":
    main()
