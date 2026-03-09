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
import re
import sys
from pathlib import Path


def fix_metadata(metadata_path: Path) -> bool:
    text = metadata_path.read_text(encoding="utf-8")
    original = text
    # 1) 空列表改为空字符串，避免 yaml-cpp bad conversion
    text = re.sub(r"offered_qos_profiles:\s*\[\]\s*", "offered_qos_profiles: ''\n", text)
    # 2) type_description_hash 多行改为单行（部分环境对多行解析有问题）
    text = re.sub(
        r"type_description_hash:\s*\n\s+(RIHS01_[a-fA-F0-9]+)",
        r"type_description_hash: \1",
        text,
    )
    if text == original:
        return False
    metadata_path.write_text(text, encoding="utf-8")
    return True


def main():
    if len(sys.argv) < 2:
        print("用法: python3 fix_ros2_bag_metadata.py <ros2_bag目录>", file=sys.stderr)
        sys.exit(1)
    bag_dir = Path(sys.argv[1]).resolve()
    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file():
        print(f"未找到: {metadata_path}", file=sys.stderr)
        sys.exit(2)
    if fix_metadata(metadata_path):
        print(f"已修复: {metadata_path}")
    else:
        print(f"无需修改: {metadata_path}")


if __name__ == "__main__":
    main()
