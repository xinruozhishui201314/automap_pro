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
import shutil
import subprocess
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


def _apply_style(text: str, style: str) -> str:
    if style == "string":
        text = re.sub(r"offered_qos_profiles:\s*\[\]\s*", "offered_qos_profiles: ''\n", text)
    else:
        text = re.sub(r"offered_qos_profiles:\s*''\s*", "offered_qos_profiles: []\n", text)

    # 修复缩进破坏的 serialization_format（应位于 topic_metadata 下）
    # 典型坏例：
    #   topic_metadata:
    #     name: /xxx
    #     offered_qos_profiles: []
    # serialization_format: cdr
    #     type: ...
    #
    # 只在它紧跟在 topic_metadata 的 name/offered_qos_profiles 之后时才缩进，避免误改其它字段。
    text = re.sub(
        r"(\n[ ]{6}offered_qos_profiles:[^\n]*\n)serialization_format:",
        r"\1      serialization_format:",
        text,
    )
    text = re.sub(
        r"(\n[ ]{6}name:[^\n]*\n)serialization_format:",
        r"\1      serialization_format:",
        text,
    )

    text = re.sub(
        r"type_description_hash:\s*\n\s+(RIHS01_[a-fA-F0-9]+)",
        r"type_description_hash: \1",
        text,
    )
    return text


def _run_ros2_bag_info(bag_dir: Path) -> tuple[bool, str]:
    if not shutil.which("ros2"):
        return False, "ros2 command not found"
    try:
        result = subprocess.run(
            ["ros2", "bag", "info", str(bag_dir)],
            capture_output=True,
            text=True,
            timeout=15,
            env=os.environ.copy(),
        )
        if result.returncode == 0:
            return True, ""
        stderr = (result.stderr or "").strip()
        stdout = (result.stdout or "").strip()
        detail = stderr if stderr else stdout
        return False, detail[:500]
    except Exception as e:
        return False, str(e)


def fix_metadata(metadata_path: Path, bag_dir: Path, qos_style: str, ros_distro: str, verify_with_ros2_info: bool) -> tuple[bool, str]:
    try:
        original = metadata_path.read_text(encoding="utf-8")
    except Exception as e:
        print(f"[fix_ros2_bag_metadata] 读取失败: {metadata_path} {e}", file=sys.stderr)
        raise

    if not os.access(metadata_path, os.W_OK):
        print(f"[fix_ros2_bag_metadata] 无写权限: {metadata_path}，请执行: chmod u+w {metadata_path}", file=sys.stderr)
        sys.exit(3)

    preferred = _target_qos_style(qos_style, ros_distro)
    candidates = [preferred] if qos_style in ("string", "list") else [preferred, "list" if preferred == "string" else "string"]
    last_err = ""

    for style in candidates:
        text = _apply_style(original, style)
        try:
            metadata_path.write_text(text, encoding="utf-8")
        except PermissionError as e:
            print(f"[fix_ros2_bag_metadata] 写入失败(权限): {metadata_path} {e}", file=sys.stderr)
            sys.exit(3)
        except Exception as e:
            print(f"[fix_ros2_bag_metadata] 写入失败: {metadata_path} {e}", file=sys.stderr)
            raise

        if not verify_with_ros2_info:
            changed = (text != original)
            return changed, style

        ok, err = _run_ros2_bag_info(bag_dir)
        if ok:
            changed = (text != original)
            return changed, style
        last_err = err
        print(f"[fix_ros2_bag_metadata] style={style} 验证失败: {err}", file=sys.stderr)

    # 验证都失败，回滚避免留下错误中间状态
    metadata_path.write_text(original, encoding="utf-8")
    raise RuntimeError(f"all styles verification failed; last_error={last_err}")


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
    parser.add_argument(
        "--verify-with-ros2-info",
        action="store_true",
        help="After patching metadata, run `ros2 bag info` to verify; auto-fallback style on failure.",
    )
    args = parser.parse_args()
    bag_dir = Path(args.bag_dir).resolve()
    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file():
        print(f"未找到: {metadata_path}", file=sys.stderr)
        sys.exit(2)
    try:
        changed, used_style = fix_metadata(
            metadata_path=metadata_path,
            bag_dir=bag_dir,
            qos_style=args.qos_style,
            ros_distro=args.ros_distro,
            verify_with_ros2_info=args.verify_with_ros2_info,
        )
        if changed:
            print(f"已修复: {metadata_path} (style={used_style})")
        else:
            print(f"无需修改: {metadata_path} (style={used_style})")
    except Exception as e:
        print(f"[fix_ros2_bag_metadata] 异常: {e}", file=sys.stderr)
        raise


if __name__ == "__main__":
    main()
