#!/usr/bin/env python3
# Visualization-only launch (connect to running system) (ROS2)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('automap_pro')
    rviz_config_default = os.path.join(pkg_share, 'config', 'automap.rviz')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=rviz_config_default,
        description='Path to RViz config file')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_map_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
    )

    return LaunchDescription([
        rviz_config_arg,
        rviz_node,
        static_tf,
    ])
