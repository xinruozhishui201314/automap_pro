#!/usr/bin/env python3
"""
修复 ROS2 bag metadata.yaml 以兼容 ros2 bag play（yaml-cpp bad conversion）。

原因：rosbags 生成的 metadata 使用 offered_qos_profiles: []，
     ROS2 Humble 的 rosbag2_storage 用 yaml-cpp 解析时期望字符串，导致 bad conversion。
用法：
    python3 scripts/fix_ros2_bag_metadata.py data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2
    或
    python3 scripts/fix_ros2_bag_metadata.py <bag目录路径>

若 metadata.yaml 由 Docker 创建导致无写权限，可先改权限再运行：
    chmod u+w data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2/metadata.yaml
    或
    sudo python3 scripts/fix_ros2_bag_metadata.py data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2
"""
import argparse
import os
import re
import sys
from pathlib import Path


def _target_qos_style(style: str, ros_distro: str) -> str:
    if style in ("string", "list"):
        return style
    distro = (ros_distro or os.environ.get("ROS_DISTRO", "")).strip().lower()
    # Jazzy+/rolling 兼容 rosbags metadata 时更稳妥的是 list 形式。
    if distro in ("jazzy", "iron", "rolling"):
        return "list"
    # Humble 及更早版本常见 bad conversion 需要字符串形式。
    return "string"


def fix_metadata(metadata_path: Path, qos_style: str, ros_distro: str) -> bool:
    try:
        text = metadata_path.read_text(encoding="utf-8")
    except Exception as e:
        print(f"[fix_ros2_bag_metadata] 读取失败: {metadata_path} {e}", file=sys.stderr)
        raise
    original = text
    # 1) offered_qos_profiles 在不同 ROS2 发行版对 metadata 兼容性不同。
    #    统一双向规范化，避免反复修复导致无效。
    style = _target_qos_style(qos_style, ros_distro)
    if style == "string":
        text = re.sub(r"offered_qos_profiles:\s*\[\]\s*", "offered_qos_profiles: ''\n", text)
    else:
        text = re.sub(r"offered_qos_profiles:\s*''\s*", "offered_qos_profiles: []\n", text)
    # 2) type_description_hash 多行改为单行（部分环境对多行解析有问题）
    text = re.sub(
        r"type_description_hash:\s*\n\s+(RIHS01_[a-fA-F0-9]+)",
        r"type_description_hash: \1",
        text,
    )
    if text == original:
        return False
    if not os.access(metadata_path, os.W_OK):
        print(f"[fix_ros2_bag_metadata] 无写权限: {metadata_path}，请执行: chmod u+w {metadata_path}", file=sys.stderr)
        sys.exit(3)
    try:
        metadata_path.write_text(text, encoding="utf-8")
    except PermissionError as e:
        print(f"[fix_ros2_bag_metadata] 写入失败(权限): {metadata_path} {e}", file=sys.stderr)
        sys.exit(3)
    except Exception as e:
        print(f"[fix_ros2_bag_metadata] 写入失败: {metadata_path} {e}", file=sys.stderr)
        raise
    return True


def main():
    parser = argparse.ArgumentParser(description="Fix ROS2 bag metadata.yaml for rosbag2 compatibility.")
    parser.add_argument("bag_dir", help="ROS2 bag directory path")
    parser.add_argument(
        "--qos-style",
        choices=["auto", "string", "list"],
        default="auto",
        help="Normalize offered_qos_profiles as string('') or list([]). auto uses ROS_DISTRO.",
    )
    parser.add_argument(
        "--ros-distro",
        default=os.environ.get("ROS_DISTRO", ""),
        help="ROS distro hint for --qos-style auto (e.g. jazzy/humble).",
    )
    args = parser.parse_args()
    bag_dir = Path(args.bag_dir).resolve()
    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file():
        print(f"未找到: {metadata_path}", file=sys.stderr)
        sys.exit(2)
    try:
        if fix_metadata(metadata_path, args.qos_style, args.ros_distro):
            print(f"已修复: {metadata_path}")
        else:
            print(f"无需修改: {metadata_path}")
    except Exception as e:
        print(f"[fix_ros2_bag_metadata] 异常: {e}", file=sys.stderr)
        raise


if __name__ == "__main__":
    main()
