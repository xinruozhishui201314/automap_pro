#!/usr/bin/env python3
"""
ROS1 to ROS2 Bag Converter Script

此脚本用于将ROS1格式的bag文件转换为ROS2格式
支持单文件和批量转换
"""

import os
import sys
import argparse
import subprocess
from pathlib import Path

def check_ros2_environment():
    """检查ROS2环境"""
    try:
        result = subprocess.run(['ros2', '--version'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            print(f"✓ ROS2 环境: {result.stdout.strip()}")
            return True
    except Exception as e:
        print(f"✗ ROS2 环境检查失败: {e}")
        return False

def convert_bag(input_bag, output_dir):
    """
    转换单个bag文件

    Args:
        input_bag: 输入bag文件路径
        output_dir: 输出目录路径
    """
    # 检查输入文件
    if not os.path.exists(input_bag):
        print(f"✗ 输入文件不存在: {input_bag}")
        return False

    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)

    # 检查输入bag格式
    try:
        file_result = subprocess.run(['file', input_bag], 
                                   capture_output=True, text=True)
        if 'data' in file_result.stdout:
            print(f"✓ 检测到 ROS1 格式: {input_bag}")
        elif 'SQLite' in file_result.stdout:
            print(f"⚠ 已经是 ROS2 格式: {input_bag}")
            print(f"  跳过转换")
            return True
        else:
            print(f"✗ 无法识别格式: {input_bag}")
            return False
    except Exception as e:
        print(f"✗ 格式检查失败: {e}")
        return False

    # 尝试转换
    print(f"\n开始转换: {input_bag} -> {output_dir}")
    
    try:
        # 方法1: 使用 ros2 bag convert
        cmd = ['ros2', 'bag', 'convert', input_bag, output_dir]
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode == 0:
            print(f"✓ 转换成功!")
            
            # 显示转换后的信息
            info_cmd = ['ros2', 'bag', 'info', output_dir]
            info_result = subprocess.run(info_cmd, capture_output=True, text=True)
            
            if info_result.returncode == 0:
                print(f"\n转换后的bag信息:")
                print(info_result.stdout)
            
            return True
        else:
            print(f"✗ 转换失败: {result.stderr}")
            return False
            
    except Exception as e:
        print(f"✗ 转换异常: {e}")
        return False

def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='ROS1 to ROS2 Bag Converter',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 转换单个bag文件
  python3 convert_ros1_to_ros2.py input.bag output_dir

  # 批量转换目录中的所有bag文件
  python3 convert_ros1_to_ros2.py --batch /path/to/bags output_base

  # 只转换指定话题
  python3 convert_ros1_to_ros2.py input.bag output_dir --topics /imu/imu /os1_cloud_node1/points

  # 详细模式
  python3 convert_ros1_to_ros2.py input.bag output_dir --verbose
        """
    )
    
    parser.add_argument('input', help='输入bag文件或目录（批量模式）')
    parser.add_argument('output', help='输出目录或输出基目录（批量模式）')
    parser.add_argument('--batch', action='store_true',
                       help='批量转换模式：转换输入目录中的所有bag文件')
    parser.add_argument('--topics', nargs='+',
                       help='只转换指定的话题')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='详细输出')
    
    args = parser.parse_args()
    
    # 检查ROS2环境
    if not check_ros2_environment():
        print("\n✗ ROS2 环境未正确配置")
        print("请确保已 source /opt/ros/humble/setup.bash")
        sys.exit(1)
    
    # 批量转换模式
    if args.batch:
        if not os.path.isdir(args.input):
            print(f"✗ 输入目录不存在: {args.input}")
            sys.exit(1)
        
        # 查找所有bag文件
        bag_files = list(Path(args.input).glob('*.bag'))
        
        if not bag_files:
            print(f"✗ 未找到bag文件: {args.input}")
            sys.exit(1)
        
        print(f"找到 {len(bag_files)} 个bag文件\n")
        
        # 创建输出基目录
        os.makedirs(args.output, exist_ok=True)
        
        # 逐个转换
        success_count = 0
        for i, bag_file in enumerate(bag_files, 1):
            print(f"\n[{i}/{len(bag_files)}] 处理: {bag_file.name}")
            
            # 输出目录为 bag文件名（去掉.bag后缀）
            output_dir = os.path.join(args.output, bag_file.stem)
            
            if convert_bag(str(bag_file), output_dir):
                success_count += 1
        
        # 总结
        print(f"\n{'='*60}")
        print(f"批量转换完成!")
        print(f"成功: {success_count}/{len(bag_files)}")
        print(f"{'='*60}")
        
        sys.exit(0 if success_count == len(bag_files) else 1)
    
    # 单文件转换模式
    else:
        if args.verbose:
            print(f"输入文件: {args.input}")
            print(f"输出目录: {args.output}")
        
        success = convert_bag(args.input, args.output)
        sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
