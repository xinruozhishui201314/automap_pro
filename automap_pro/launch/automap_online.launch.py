#!/usr/bin/env python3
# AutoMap-Pro Online Mapping Launch (ROS2)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('automap_pro')
    config_default = os.path.join(pkg_share, 'config', 'system_config.yaml')
    rviz_config_default = os.path.join(pkg_share, 'config', 'automap.rviz')

    config_arg = DeclareLaunchArgument(
        'config', default_value=config_default,
        description='Path to system_config.yaml')
    output_dir_arg = DeclareLaunchArgument(
        'output_dir', default_value='/data/automap_output',
        description='Output directory for maps and logs')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to start RViz')
    use_external_frontend_arg = DeclareLaunchArgument(
        'use_external_frontend', default_value='true',
        description='If true, launch fast-livo2 node (config frontend.mode must be external_fast_livo)')
    use_external_overlap_arg = DeclareLaunchArgument(
        'use_external_overlap', default_value='false',
        description='If true, launch OverlapTransformer descriptor service (config loop_closure.overlap_transformer.mode must be external_service)')
    bag_file_arg = DeclareLaunchArgument(
        'bag_file', default_value='',
        description='Optional bag file to play (empty = live only)')

    bag_file = LaunchConfiguration('bag_file')
    bag_given = IfCondition(PythonExpression("'", bag_file, "' != ''"))
    use_external_frontend = LaunchConfiguration('use_external_frontend', default='false')
    use_external_overlap = LaunchConfiguration('use_external_overlap', default='false')

    # Optional rosbag2 play (only when bag_file is set)
    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file, '--rate', '1.0', '--clock'],
        output='screen',
        condition=bag_given,
    )

    # use_sim_time true when playing a bag so nodes follow /clock
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # fast-livo2-humble 节点（仅当 use_external_frontend=true 且 fast_livo 已安装时可用）
    fast_livo_node = None
    try:
        fast_livo_share = get_package_share_directory('fast_livo')
        avia_params = os.path.join(fast_livo_share, 'config', 'mid360.yaml')
        camera_params = os.path.join(fast_livo_share, 'config', 'camera_pinhole.yaml')
        fast_livo_node = Node(
            package='fast_livo',
            executable='fastlivo_mapping',
            name='laserMapping',
            parameters=[avia_params, camera_params],
            output='screen',
            condition=IfCondition(use_external_frontend),
        )
    except Exception:
        pass  # fast_livo 未安装时跳过

    overlap_descriptor_node = None
    try:
        overlap_descriptor_node = Node(
            package='overlap_transformer_ros2',
            executable='descriptor_server',
            name='overlap_transformer_descriptor_server',
            output='screen',
            parameters=[{'model_path': ''}],
            condition=IfCondition(use_external_overlap),
        )
    except Exception:
        pass

    automap_node = Node(
        package='automap_pro',
        executable='automap_system_node',
        name='automap_system',
        output='screen',
        parameters=[
            {'config': LaunchConfiguration('config')},
            {'output_dir': LaunchConfiguration('output_dir')},
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/livox/lidar', '/livox/lidar'),
            ('/livox/imu', '/livox/imu'),
            ('/gps/fix', '/gps/fix'),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_default],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
    )

    ld_actions = [
        config_arg,
        output_dir_arg,
        use_rviz_arg,
        use_external_frontend_arg,
        use_external_overlap_arg,
        bag_file_arg,
        rosbag_play,
        automap_node,
        rviz_node,
        static_tf,
    ]
    if fast_livo_node is not None:
        ld_actions.insert(ld_actions.index(automap_node), fast_livo_node)
    if overlap_descriptor_node is not None:
        ld_actions.insert(ld_actions.index(automap_node), overlap_descriptor_node)
    return LaunchDescription(ld_actions)
