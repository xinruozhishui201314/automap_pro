#!/usr/bin/env python3
# AutoMap-Pro Online Mapping Launch (ROS2)
# 从 system_config.yaml 读取并启动：fast-livo2、overlap_transformer_ros2、HBA、automap_system

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _load_system_config(context):
    config_path = LaunchConfiguration("config").perform(context)
    try:
        import yaml
        with open(config_path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)
    except Exception:
        return {}


def _launch_nodes_with_system_config(context, *args, **kwargs):
    config_path = LaunchConfiguration("config").perform(context)
    pkg_share = get_package_share_directory("automap_pro")
    rviz_config_default = os.path.join(pkg_share, "config", "automap.rviz")

    # 从 system_config 生成各组件参数（launch 与 params_from_system_config 同目录）
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
        if not fl2_params and LaunchConfiguration("use_external_frontend").perform(context) == "true":
            import sys
            print("[automap_pro launch] WARNING: system_config not loaded; fast_livo will have no params from system_config. Config path: {}".format(config_path), file=sys.stderr)
    except Exception as e:
        system_config = {}
        ot_params = {"model_path": ""}
        fl2_params = {}
        hba_params = {}
        hba_cal_mme_params = {}
        hba_visualize_params = {}
        import sys
        print("[automap_pro launch] WARNING: Failed to load system_config from {}: {}. fast_livo/overlap_transformer may use fallback.".format(config_path, e), file=sys.stderr)

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
            fast_livo_node = Node(
                package="fast_livo",
                executable="fastlivo_mapping",
                name="laserMapping",
                parameters=[fl2_params],
                output="screen",
                condition=IfCondition(use_external_frontend),
            )
            nodes.append(fast_livo_node)
    except Exception:
        pass

    # OverlapTransformer descriptor 节点：参数全部来自 system_config
    try:
        overlap_node = Node(
            package="overlap_transformer_ros2",
            executable="descriptor_server",
            name="overlap_transformer_descriptor_server",
            output="screen",
            parameters=[ot_params],
            condition=IfCondition(use_external_overlap),
        )
        nodes.append(overlap_node)
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

    # AutoMap System 节点：话题映射由 system_config.yaml 决定，不使用固定的 remappings
    automap_node = Node(
        package="automap_pro",
        executable="automap_system_node",
        name="automap_system",
        output="screen",
        parameters=[
            {"config": LaunchConfiguration("config")},
            {"output_dir": LaunchConfiguration("output_dir")},
            {"use_sim_time": LaunchConfiguration("use_sim_time", default="false")},
        ],
        # 移除固定 remappings，让系统从 system_config.yaml 读取话题名
    )
    nodes.append(automap_node)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config_default],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    nodes.append(rviz_node)

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_map_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
    )
    nodes.append(static_tf)

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory("automap_pro")
    config_default = os.path.join(pkg_share, "config", "system_config.yaml")

    config_arg = DeclareLaunchArgument(
        "config", default_value=config_default,
        description="Path to system_config.yaml (overlap_transformer / fast_livo2 / HBA 参数均从此文件读取)",
    )
    output_dir_arg = DeclareLaunchArgument(
        "output_dir", default_value="/data/automap_output",
        description="Output directory for maps and logs",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="Whether to start RViz",
    )
    use_external_frontend_arg = DeclareLaunchArgument(
        "use_external_frontend", default_value="true",
        description="true=use verified fast_livo node (modular); false=use internal FastLIVO2Wrapper (ESIKF)",
    )
    use_external_overlap_arg = DeclareLaunchArgument(
        "use_external_overlap", default_value="false",
        description="If true, launch OverlapTransformer descriptor (params from system_config.loop_closure.overlap_transformer)",
    )
    use_hba_arg = DeclareLaunchArgument(
        "use_hba", default_value="true",
        description="If true, launch HBA backend node (params from system_config.backend.hba)",
    )
    use_hba_cal_mme_arg = DeclareLaunchArgument(
        "use_hba_cal_mme", default_value="false",
        description="If true, launch HBA cal_MME node (params from system_config.backend.hba_cal_mme)",
    )
    use_hba_visualize_arg = DeclareLaunchArgument(
        "use_hba_visualize", default_value="false",
        description="If true, launch HBA visualize node (params from system_config.backend.hba_visualize)",
    )
    bag_file_arg = DeclareLaunchArgument(
        "bag_file", default_value="",
        description="Optional bag file to play (empty = live only)",
    )

    bag_file = LaunchConfiguration("bag_file")
    bag_given = IfCondition(PythonExpression("'", bag_file, "' != ''"))
    rosbag_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", bag_file, "--rate", "1.0", "--clock"],
        output="screen",
        condition=bag_given,
    )

    return LaunchDescription([
        config_arg,
        output_dir_arg,
        use_rviz_arg,
        use_external_frontend_arg,
        use_external_overlap_arg,
        use_hba_arg,
        use_hba_cal_mme_arg,
        use_hba_visualize_arg,
        bag_file_arg,
        rosbag_play,
        OpaqueFunction(function=_launch_nodes_with_system_config),
    ])
