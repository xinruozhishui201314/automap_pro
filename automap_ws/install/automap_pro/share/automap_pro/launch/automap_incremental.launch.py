#!/usr/bin/env python3
# AutoMap-Pro Incremental (Multi-Session) Mapping Launch (ROS2)
# 从 system_config.yaml 读取并启动：fast-livo2、overlap_transformer_ros2、HBA、automap_system

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes_incremental(context, *args, **kwargs):
    config_path = LaunchConfiguration("config").perform(context)
    pkg_share = get_package_share_directory("automap_pro")
    rviz_config_default = os.path.join(pkg_share, "config", "automap.rviz")
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    if launch_dir not in __import__("sys").path:
        __import__("sys").path.insert(0, launch_dir)
    try:
        from params_from_system_config import (
            load_system_config,
            get_overlap_transformer_params,
            get_fast_livo2_params,
            get_hba_params,
            get_hba_cal_mme_params,
            get_hba_visualize_params,
        )
        system_config = load_system_config(config_path)
        ot_params = get_overlap_transformer_params(system_config)
        fl2_params = get_fast_livo2_params(system_config)
        hba_params = get_hba_params(system_config)
        hba_cal_mme_params = get_hba_cal_mme_params(system_config)
        hba_visualize_params = get_hba_visualize_params(system_config)
    except Exception:
        ot_params = {"model_path": ""}
        fl2_params = {}
        hba_params = {}
        hba_cal_mme_params = {}
        hba_visualize_params = {}

    use_external_frontend = LaunchConfiguration("use_external_frontend", default="false")
    use_external_overlap = LaunchConfiguration("use_external_overlap", default="false")
    use_hba = LaunchConfiguration("use_hba", default="true")
    use_hba_cal_mme = LaunchConfiguration("use_hba_cal_mme", default="false")
    use_hba_visualize = LaunchConfiguration("use_hba_visualize", default="false")
    nodes = []

    # fast-livo2 节点：只使用从 system_config 生成的参数
    # 注意：移除 camera_pinhole.yaml 避免参数冲突
    try:
        get_package_share_directory("fast_livo")
        if fl2_params:
            nodes.append(Node(
                package="fast_livo", executable="fastlivo_mapping", name="laserMapping",
                parameters=[fl2_params],
                output="screen", condition=IfCondition(use_external_frontend),
            ))
    except Exception:
        pass

    try:
        nodes.append(Node(
            package="overlap_transformer_ros2", executable="descriptor_server",
            name="overlap_transformer_descriptor_server", output="screen",
            parameters=[ot_params], condition=IfCondition(use_external_overlap),
        ))
    except Exception:
        pass

    # HBA 模块节点（参数来自 system_config.backend.hba / hba_cal_mme / hba_visualize）
    if hba_params:
        nodes.append(Node(
            package="hba", namespace="hba", executable="hba", name="hba_node",
            output="screen", parameters=[hba_params], condition=IfCondition(use_hba),
        ))
    if hba_cal_mme_params:
        nodes.append(Node(
            package="hba", namespace="cal_MME", executable="calculate_MME", name="cal_MME_node",
            output="screen", parameters=[hba_cal_mme_params], condition=IfCondition(use_hba_cal_mme),
        ))
    if hba_visualize_params:
        nodes.append(Node(
            package="hba", namespace="visualize", executable="visualize", name="visualize_node",
            output="screen", parameters=[hba_visualize_params], condition=IfCondition(use_hba_visualize),
        ))

    nodes.append(Node(
        package="automap_pro", executable="automap_system_node", name="automap_system",
        output="screen",
        parameters=[
            {"config": LaunchConfiguration("config")},
            {"output_dir": LaunchConfiguration("output_dir")},
            {"session_id": LaunchConfiguration("session_id")},
            {"prev_session": LaunchConfiguration("prev_session")},
            {"mode": "incremental"},
        ],
        remappings=[("/livox/lidar", "/livox/lidar"), ("/livox/imu", "/livox/imu"), ("/gps/fix", "/gps/fix")],
    ))
    nodes.append(Node(
        package="rviz2", executable="rviz2", name="rviz",
        arguments=["-d", rviz_config_default], output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    ))
    nodes.append(Node(
        package="tf2_ros", executable="static_transform_publisher", name="world_map_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
    ))
    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory("automap_pro")
    config_default = os.path.join(pkg_share, "config", "system_config.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=config_default, description="Path to system_config.yaml"),
        DeclareLaunchArgument("prev_session", default_value="", description="Path to previous session directory"),
        DeclareLaunchArgument("output_dir", default_value="/data/automap_output", description="Output directory"),
        DeclareLaunchArgument("session_id", default_value="1", description="Session ID"),
        DeclareLaunchArgument("use_rviz", default_value="true", description="Whether to start RViz"),
        DeclareLaunchArgument("use_external_frontend", default_value="true", description="true=use verified fast_livo node (modular); false=use internal FastLIVO2Wrapper (ESIKF)"),
        DeclareLaunchArgument("use_external_overlap", default_value="false", description="Launch OverlapTransformer descriptor (params from system_config)"),
        DeclareLaunchArgument("use_hba", default_value="true", description="Launch HBA backend node (params from system_config.backend.hba)"),
        DeclareLaunchArgument("use_hba_cal_mme", default_value="false", description="Launch HBA cal_MME node (params from system_config.backend.hba_cal_mme)"),
        DeclareLaunchArgument("use_hba_visualize", default_value="false", description="Launch HBA visualize node (params from system_config.backend.hba_visualize)"),
        OpaqueFunction(function=_launch_nodes_incremental),
    ])
