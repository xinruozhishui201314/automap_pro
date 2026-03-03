#!/usr/bin/env python3
"""
AutoMap-Pro: Composable Node 启动文件（推荐，零拷贝）

架构：
  Component Container
    ├── fast_livo::LIVMapperComposable  (FAST-LIVO2, LiDAR+IMU+Camera)
    └── automap_pro::AutoMapSystem      (主控层)

两节点在同一进程内运行，rclcpp 自动启用 Intra-Process Communication。
话题传递使用零拷贝（shared_ptr 直接传递，无序列化）。

使用方式：
  # 有 GPU（LibTorch CUDA）
  ros2 launch automap_pro automap_composable.launch.py
  
  # 指定配置文件
  ros2 launch automap_pro automap_composable.launch.py config:=/path/to/config.yaml
  
  # 不使用 Composable（独立进程模式，用于调试）
  ros2 launch automap_pro automap_composable.launch.py composable:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 解析默认路径（避免 default_value='' 导致 launch 以目录 '.' 打开文件触发 [Errno 21]）
    automap_share = get_package_share_directory('automap_pro')
    default_config_path = os.path.join(automap_share, 'config', 'system_config.yaml')
    try:
        fast_livo_share = get_package_share_directory('fast_livo')
        default_fl_config_path = os.path.join(fast_livo_share, 'config', 'avia.yaml')
    except Exception:
        default_fl_config_path = os.path.join(automap_share, 'config', 'fast_livo2_config.yaml')

    # ── 参数声明 ─────────────────────────────────────────────────────────
    config_arg = DeclareLaunchArgument(
        'config', default_value=default_config_path,
        description='Path to system_config.yaml')

    fast_livo_config_arg = DeclareLaunchArgument(
        'fast_livo_config', default_value=default_fl_config_path,
        description='Path to fast_livo config')

    composable_arg = DeclareLaunchArgument(
        'composable', default_value='true',
        description='Whether to use Composable Node mode (zero-copy)')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Whether to launch RViz2')

    config  = LaunchConfiguration('config')
    fl_config = LaunchConfiguration('fast_livo_config')
    composable = LaunchConfiguration('composable')
    use_rviz   = LaunchConfiguration('use_rviz')

    # 用于 Substitution 的默认路径（非空，避免参数被当作目录）
    default_config = PathJoinSubstitution([
        FindPackageShare('automap_pro'), 'config', 'system_config.yaml'])
    default_fl_config = PathJoinSubstitution([
        FindPackageShare('fast_livo'), 'config', 'avia.yaml'])

    # ── Composable Node 模式（推荐，零拷贝）────────────────────────────────
    composable_container = ComposableNodeContainer(
        condition=IfCondition(composable),
        name='automap_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # FAST-LIVO2 Composable Node
            ComposableNode(
                package='fast_livo',
                plugin='fast_livo::LIVMapperComposable',
                name='fast_livo',
                parameters=[fl_config],
                remappings=[
                    ('/cloud_registered', '/cloud_registered'),
                    ('/aft_mapped_to_init', '/aft_mapped_to_init'),
                    ('/fast_livo/keyframe_info', '/fast_livo/keyframe_info'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # AutoMapSystem Composable Node
            ComposableNode(
                package='automap_pro',
                plugin='automap_pro::AutoMapSystem',
                name='automap_system',
                parameters=[{
                    'config_file': config,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    # ── 独立进程模式（调试用）────────────────────────────────────────────
    fast_livo_node = Node(
        condition=UnlessCondition(composable),
        package='fast_livo',
        executable='mapping',
        name='fast_livo',
        output='screen',
        parameters=[fl_config],
    )

    automap_node = Node(
        condition=UnlessCondition(composable),
        package='automap_pro',
        executable='automap_system',
        name='automap_system',
        output='screen',
        parameters=[{'config_file': config}],
    )

    # ── RViz2（可选，使用已存在的 rviz/automap.rviz，避免 -d 指向目录触发 Errno 21）──
    rviz_config = PathJoinSubstitution([
        FindPackageShare('automap_pro'), 'rviz', 'automap.rviz'])
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        config_arg,
        fast_livo_config_arg,
        composable_arg,
        use_rviz_arg,
        composable_container,
        fast_livo_node,
        automap_node,
        rviz_node,
    ])
