#!/usr/bin/env python3
"""
AutoMap-Pro: Composable Node 启动文件

架构：
  - fast_livo：独立进程 (fastlivo_mapping)
      参数来源：由 params_from_system_config.get_fast_livo2_params 从 system_config 的
               sensor: + fast_livo: 节合并生成临时 ROS2 params 文件，话题唯一来自 sensor:。
  - automap_system：composable=true 时在 Component Container 内加载

数据流：rosbag → <lid_topic>, <imu_topic>  （话题名唯一来自 system_config 的 sensor: 节）
        → fast_livo → /aft_mapped_to_init, /cloud_registered, /fast_livo/keyframe_info
        → automap_system

使用方式：
  ros2 launch automap_pro automap_composable.launch.py
  ros2 launch automap_pro automap_composable.launch.py config:=/path/to/system_config.yaml
  ros2 launch automap_pro automap_composable.launch.py composable:=false
"""

import os
import sys
import yaml
import tempfile

# 与 offline/online launch 一致：从 sensor + fast_livo 合并生成 params，话题唯一来自 sensor
_launch_dir = os.path.dirname(os.path.abspath(__file__))
if _launch_dir not in sys.path:
    sys.path.insert(0, _launch_dir)
from params_from_system_config import get_fast_livo2_params

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


# ── OpaqueFunction：从 system_config 合并 sensor + fast_livo 生成 fast_livo ROS2 params ──
def _setup_fast_livo(context, *args, **kwargs):
    """
    在 launch 时读取 system_config，用 get_fast_livo2_params 合并 sensor 与 fast_livo，
    话题（lid_topic/imu_topic/img_topic）唯一来自 sensor: 节，写入临时 params 文件。
    """
    config_file = LaunchConfiguration('config').perform(context)

    # ── 读取 system_config.yaml ──────────────────────────────────────────────
    if not os.path.isfile(config_file):
        raise FileNotFoundError(
            f"[automap_composable] config file not found: {config_file}")

    with open(config_file, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    if not cfg:
        raise RuntimeError(f"[automap_composable] config empty or invalid: {config_file}")

    if not cfg.get('fast_livo') and not (cfg.get('frontend') or {}).get('fast_livo2'):
        raise RuntimeError(
            f"[automap_composable] 'fast_livo:' (or frontend.fast_livo2) missing in {config_file}. "
            "Required for extrin_calib, preprocess, etc.")

    # 话题唯一来自 sensor: 节，与 offline/online launch 一致
    fl_params = get_fast_livo2_params(cfg)

    # ── 生成 ROS2 /**:ros__parameters: YAML ─────────────────────────────────
    tmp_path = f'/tmp/automap_fl_params_{os.getpid()}.yaml'
    with open(tmp_path, 'w', encoding='utf-8') as f:
        f.write('/**:\n')
        f.write('  ros__parameters:\n')
        inner = yaml.dump(fl_params,
                          default_flow_style=False,
                          allow_unicode=True,
                          sort_keys=False)
        for line in inner.splitlines():
            f.write('    ' + line + '\n')

    # ── 打印摘要（便于排障）────────────────────────────────────────────────
    common = fl_params.get('common', {})
    print(f'[automap_composable] fast_livo params generated: {tmp_path}')
    print(f'[automap_composable]   lid_topic = {common.get("lid_topic", "?")}')
    print(f'[automap_composable]   imu_topic = {common.get("imu_topic", "?")}')
    print(f'[automap_composable]   img_en    = {common.get("img_en", "?")} '
          f'(0=LIO-only, 1=LIVO)')
    print(f'[automap_composable]   sections  = {list(fl_params.keys())}')

    # ── 构建 fastlivo_mapping 进程 ────────────────────────────────────────
    fast_livo_proc = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'fast_livo', 'fastlivo_mapping',
            '--ros-args',
            '-r', '__node:=fast_livo',
            '-p', 'use_sim_time:=true',
            '--params-file', tmp_path,
        ],
        output='screen',
        name='fastlivo_mapping',
    )
    return [fast_livo_proc]


# ── LaunchDescription ─────────────────────────────────────────────────────────
def generate_launch_description():
    automap_share = get_package_share_directory('automap_pro')
    default_config_path = os.path.join(
        automap_share, 'config', 'system_config.yaml')

    # ── 参数声明 ──────────────────────────────────────────────────────────
    config_arg = DeclareLaunchArgument(
        'config', default_value=default_config_path,
        description='Path to system_config.yaml (unified config for all modules)')

    composable_arg = DeclareLaunchArgument(
        'composable', default_value='true',
        description='Whether to run automap_system as Composable Node (zero-copy)')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Whether to launch RViz2')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time (set true when playing bag with --clock)')

    config       = LaunchConfiguration('config')
    composable   = LaunchConfiguration('composable')
    use_rviz     = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── fast_livo：OpaqueFunction 在 launch 时动态生成 ROS2 params 文件 ──
    fast_livo_action = OpaqueFunction(function=_setup_fast_livo)

    # ── Composable 模式：Container 内加载 automap_system ─────────────────
    composable_container = ComposableNodeContainer(
        condition=IfCondition(composable),
        name='automap_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        parameters=[{'use_sim_time': use_sim_time}],
        composable_node_descriptions=[
            ComposableNode(
                package='automap_pro',
                plugin='automap_pro::AutoMapSystem',
                name='automap_system',
                parameters=[{'config_file': config, 'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    # ── 非 Composable 模式：automap_system 独立进程 ───────────────────────
    automap_node = Node(
        condition=UnlessCondition(composable),
        package='automap_pro',
        executable='automap_system',
        name='automap_system',
        output='screen',
        parameters=[{'config_file': config, 'use_sim_time': use_sim_time}],
    )

    # ── RViz2（可选）：前端 / 后端各一个窗口 ─────────────────────────────────
    rviz_frontend_config = PathJoinSubstitution([
        FindPackageShare('automap_pro'), 'rviz', 'automap_frontend.rviz'])
    rviz_backend_config = PathJoinSubstitution([
        FindPackageShare('automap_pro'), 'rviz', 'automap_backend.rviz'])
    rviz_frontend_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz_frontend',
        arguments=['-d', rviz_frontend_config],
        output='screen',
    )
    rviz_backend_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz_backend',
        arguments=['-d', rviz_backend_config],
        output='screen',
    )

    # ── 启动时日志（用 LaunchConfiguration 替代 default_config_path，
    #            确保日志显示实际传入的 config 路径而非硬编码的 install 默认值）──
    from launch.substitutions import PythonExpression
    log_start = LogInfo(
        msg=['[automap_composable] Launch started. Unified config=',
             config,
             ' (fast_livo params will be extracted from fast_livo: section)'])

    return LaunchDescription([
        config_arg,
        composable_arg,
        use_rviz_arg,
        use_sim_time_arg,
        log_start,
        fast_livo_action,      # OpaqueFunction: 生成临时 params 并启动 fast_livo
        composable_container,
        automap_node,
        rviz_frontend_node,
        rviz_backend_node,
    ])
