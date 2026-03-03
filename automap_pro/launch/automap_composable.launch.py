#!/usr/bin/env python3
"""
AutoMap-Pro: Composable Node 启动文件

架构：
  - fast_livo：独立进程 (fastlivo_mapping)
      参数来源：从 system_config.yaml 的 fast_livo: 节自动生成临时 ROS2
               params 文件 (/tmp/automap_fl_params_<pid>.yaml)，通过
               --params-file 传入，无需单独维护 avia.yaml。
  - automap_system：composable=true 时在 Component Container 内加载

数据流：rosbag → <lid_topic>, <imu_topic>  （话题名由 system_config.yaml 的 fast_livo.common 节决定）
        → fast_livo → /aft_mapped_to_init, /cloud_registered, /fast_livo/keyframe_info
        → automap_system

使用方式：
  ros2 launch automap_pro automap_composable.launch.py
  ros2 launch automap_pro automap_composable.launch.py config:=/path/to/system_config.yaml
  ros2 launch automap_pro automap_composable.launch.py composable:=false
"""

import os
import yaml
import tempfile

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


# ── OpaqueFunction：从 system_config.yaml 生成 fast_livo ROS2 params ──────────
def _setup_fast_livo(context, *args, **kwargs):
    """
    在 launch 时读取 system_config.yaml 的 fast_livo: 节，
    转换为 ROS2 --params-file 格式并写入临时文件，
    返回 fastlivo_mapping 的 ExecuteProcess action。
    """
    config_file = LaunchConfiguration('config').perform(context)

    # ── 读取 system_config.yaml ──────────────────────────────────────────────
    if not os.path.isfile(config_file):
        raise FileNotFoundError(
            f"[automap_composable] config file not found: {config_file}")

    with open(config_file, 'r') as f:
        cfg = yaml.safe_load(f)

    fl_params = cfg.get('fast_livo')
    if not fl_params:
        raise RuntimeError(
            f"[automap_composable] 'fast_livo:' section missing in {config_file}. "
            "All fast_livo parameters must be defined under 'fast_livo:' key.")

    # ── 生成 ROS2 /**:ros__parameters: YAML ─────────────────────────────────
    # 手动写顶层 "/**:" 避免 PyYAML 将 * 字符引号化导致 rcl 解析失败
    tmp_path = f'/tmp/automap_fl_params_{os.getpid()}.yaml'
    with open(tmp_path, 'w') as f:
        f.write('/**:\n')
        f.write('  ros__parameters:\n')
        # 将 fast_livo: 节的内容缩进 4 空格写入
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

    # ── RViz2（可选）──────────────────────────────────────────────────────
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
        rviz_node,
    ])
