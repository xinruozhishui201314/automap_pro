#!/usr/bin/env python3
"""
AutoMap-Pro ROS2 Bag 诊断与修复工具

功能：
  1. 检测 bag 目录结构
  2. 解析并修复 metadata.yaml（bad conversion）
  3. 检查 bag 存储格式（sqlite3/mcap）
  4. 快速预检查，避免容器内长时间等待
  5. 诊断 topic 列表与消息数量
  
使用：
  python3 scripts/diagnose_and_fix_bag.py <bag_dir> [--verbose] [--fix] [--list-topics]

退出码：
  0: 成功，bag 可用
  1: 参数错误
  2: metadata.yaml 不存在
  3: 权限问题（无法修复）
  4: 元数据格式异常（无法自动修复）
  5: bag 存储文件不存在
"""
import os
import sys
import re
import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple

try:
    import yaml
except ImportError:
    yaml = None

_SCRIPT_NAME = "[diagnose_and_fix_bag.py]"


def log_info(msg: str):
    print(f"{_SCRIPT_NAME} [INFO] {msg}")


def log_warn(msg: str):
    print(f"{_SCRIPT_NAME} [WARN] {msg}", file=sys.stderr)


def log_error(msg: str):
    print(f"{_SCRIPT_NAME} [ERROR] {msg}", file=sys.stderr)


def log_debug(msg: str, verbose: bool = False):
    if verbose:
        print(f"{_SCRIPT_NAME} [DEBUG] {msg}", file=sys.stderr)


def check_bag_structure(bag_dir: Path, verbose: bool = False) -> Tuple[bool, str]:
    """检查 bag 目录结构（sqlite3 或 mcap）"""
    log_debug(f"检查 bag 目录: {bag_dir}", verbose)
    
    if not bag_dir.is_dir():
        return False, f"目录不存在: {bag_dir}"
    
    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file():
        return False, f"metadata.yaml 不存在: {metadata_path}"
    
    # 检查存储格式
    sqlite_file = bag_dir / "*.db3"
    mcap_file = bag_dir / "*.mcap"
    
    # 用 glob 简单检查
    import glob
    db3_files = glob.glob(str(bag_dir / "*.db3"))
    mcap_files = glob.glob(str(bag_dir / "*.mcap"))
    
    if db3_files:
        log_debug(f"检测到 sqlite3 存储: {db3_files[0]}", verbose)
        return True, "sqlite3"
    elif mcap_files:
        log_debug(f"检测到 mcap 存储: {mcap_files[0]}", verbose)
        return True, "mcap"
    else:
        return False, f"未找到存储文件 (*.db3 或 *.mcap) 在: {bag_dir}"


def read_metadata_yaml(metadata_path: Path, verbose: bool = False) -> Optional[Dict]:
    """安全读取 metadata.yaml（Python yaml）"""
    if not yaml:
        log_warn("PyYAML 未安装，使用正则表达式进行修复")
        return None
    
    try:
        with open(metadata_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        log_debug(f"YAML 解析成功: {metadata_path}", verbose)
        return data
    except yaml.YAMLError as e:
        log_error(f"YAML 解析异常: {e}")
        return None
    except Exception as e:
        log_error(f"读取失败: {e}")
        return None


def fix_metadata_yaml(metadata_path: Path, verbose: bool = False) -> Tuple[bool, str]:
    """
    修复 metadata.yaml 中的 bad conversion 问题。
    
    规则：
    1. offered_qos_profiles: [] → offered_qos_profiles: ''
    2. type_description_hash 多行 → 单行
    """
    log_debug(f"开始修复: {metadata_path}", verbose)
    
    if not metadata_path.is_file():
        return False, f"文件不存在: {metadata_path}"
    
    if not os.access(metadata_path, os.W_OK):
        try:
            stat_info = metadata_path.stat()
            # 若文件属主不是当前用户，chmod u+w 无效，需先改属主
            if hasattr(stat_info, "st_uid") and stat_info.st_uid != 0 and stat_info.st_uid != os.getuid():
                return False, f"无写权限（文件属主非当前用户，请执行: sudo chown $(whoami):$(whoami) {metadata_path}）"
            if hasattr(stat_info, "st_uid") and stat_info.st_uid == 0:
                return False, f"无写权限（文件属主为 root，请执行: sudo chown $(whoami):$(whoami) {metadata_path}）"
        except OSError:
            pass
        return False, f"无写权限（请执行: chmod u+w {metadata_path}）"
    
    try:
        text = metadata_path.read_text(encoding='utf-8')
    except Exception as e:
        return False, f"读取失败: {e}"
    
    original = text
    
    # 规则 1: offered_qos_profiles: [] → ''
    text = re.sub(
        r"offered_qos_profiles:\s*\[\]\s*",
        "offered_qos_profiles: ''\n",
        text
    )
    
    # 规则 2: type_description_hash 多行 → 单行
    text = re.sub(
        r"type_description_hash:\s*\n\s+(RIHS01_[a-fA-F0-9]+)",
        r"type_description_hash: \1",
        text
    )
    
    # 规则 3: 紧接 offered_qos_profiles 后的 serialization_format 缺少缩进（根级导致 YAML 解析错误）
    text = re.sub(
        r"(offered_qos_profiles: ''\n)(serialization_format: )",
        r"\1      \2",
        text
    )
    
    if text == original:
        log_debug("无需修改", verbose)
        return True, "无需修改"
    
    try:
        metadata_path.write_text(text, encoding='utf-8')
        log_info(f"已修复: {metadata_path}")
        return True, "已修复"
    except PermissionError:
        return False, f"写入失败（权限）: chmod u+w {metadata_path}"
    except Exception as e:
        return False, f"写入失败: {e}"


def list_topics_and_counts(metadata_path: Path, verbose: bool = False) -> Tuple[bool, Dict[str, int]]:
    """
    从 metadata.yaml 中提取 topic 列表及消息数量
    
    Returns:
      (success, {topic_name: message_count})
    """
    data = read_metadata_yaml(metadata_path, verbose)
    if not data:
        return False, {}
    
    topics = {}
    try:
        topics_with_msg_count = data.get("rosbag2_bagfile_information", {}).get("topics_with_message_count", [])
        for topic_info in topics_with_msg_count:
            name = topic_info.get("topic_metadata", {}).get("name", "unknown")
            count = topic_info.get("message_count", 0)
            topics[name] = count
    except Exception as e:
        log_warn(f"提取 topic 失败: {e}")
        return False, {}
    
    return True, topics


def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description="AutoMap-Pro ROS2 Bag 诊断与修复工具",
        epilog="退出码: 0=成功，1=参数错误，2=metadata不存在，3=权限问题，4=格式异常，5=存储文件不存在"
    )
    parser.add_argument("bag_dir", help="ROS2 bag 目录路径")
    parser.add_argument("--verbose", "-v", action="store_true", help="详细输出")
    parser.add_argument("--fix", action="store_true", help="自动修复 metadata.yaml")
    parser.add_argument("--list-topics", action="store_true", help="列出所有 topic 及消息数量")
    parser.add_argument("--json", action="store_true", help="JSON 输出（便于脚本调用）")
    
    args = parser.parse_args()
    
    bag_dir = Path(args.bag_dir).resolve()
    metadata_path = bag_dir / "metadata.yaml"
    
    result = {
        "bag_dir": str(bag_dir),
        "success": False,
        "message": "",
        "storage_format": None,
        "topics": {},
        "actions_taken": []
    }
    
    # 第 1 步：检查目录结构
    ok, msg = check_bag_structure(bag_dir, args.verbose)
    if not ok:
        result["message"] = msg
        if args.json:
            print(json.dumps(result, indent=2))
        else:
            log_error(msg)
        return 5 if "存储文件" in msg else 2
    result["storage_format"] = msg
    log_info(f"存储格式: {msg}")
    
    # 第 2 步：修复 metadata（如果需要）
    if args.fix or not args.json:  # 默认尝试修复
        ok, msg = fix_metadata_yaml(metadata_path, args.verbose)
        result["actions_taken"].append({"action": "fix_metadata", "success": ok, "message": msg})
        if not ok and not args.json:
            log_warn(msg)
    
    # 第 3 步：解析 metadata 与 topic 列表
    if args.list_topics or args.json:
        ok, topics = list_topics_and_counts(metadata_path, args.verbose)
        if ok:
            result["topics"] = topics
            if args.list_topics and not args.json:
                log_info(f"Topic 数量: {len(topics)}")
                for name, count in sorted(topics.items()):
                    log_info(f"  {name}: {count} 条消息")
        else:
            log_warn("无法提取 topic 列表（YAML 解析失败）")
    
    # 第 4 步：最终检查（用 Python yaml 重新验证）
    if yaml:
        data = read_metadata_yaml(metadata_path, args.verbose)
        if data:
            result["success"] = True
            result["message"] = "Bag 可用，metadata.yaml 通过验证"
            exit_code = 0
        else:
            result["message"] = "metadata.yaml 仍无法解析（可能需要手动修复）"
            exit_code = 4
    else:
        # 未安装 yaml，假设成功（修复脚本已执行）
        result["success"] = True
        result["message"] = "已执行修复（未安装 PyYAML，无法验证）"
        exit_code = 0
    
    if args.json:
        print(json.dumps(result, indent=2))
    else:
        log_info(result["message"])
    
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
