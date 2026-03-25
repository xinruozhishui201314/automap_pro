#!/usr/bin/env python3
# AutoMap-Pro Incremental (Multi-Session) Mapping Launch (ROS2)
# 从 system_config.yaml 读取并启动：fast-livo2、overlap_transformer_ros2、HBA、automap_system

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes_incremental(context, *args, **kwargs):
    config_path = LaunchConfiguration("config").perform(context)
    pkg_share = get_package_share_directory("automap_pro")
    rviz_frontend_config = os.path.join(pkg_share, "rviz", "automap_frontend.rviz")
    rviz_backend_config = os.path.join(pkg_share, "rviz", "automap_backend.rviz")
    if not os.path.isfile(rviz_frontend_config):
        rviz_frontend_config = os.path.join(pkg_share, "rviz", "automap.rviz")
    if not os.path.isfile(rviz_backend_config):
        rviz_backend_config = os.path.join(pkg_share, "rviz", "automap.rviz")
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    if launch_dir not in sys.path:
        sys.path.insert(0, launch_dir)
    from rviz_utils import is_rviz2_installed
    try:
        from params_from_system_config import (
            load_system_config,
            get_overlap_transformer_params,
            resolve_default_overlap_model_path,
            get_fast_livo2_params,
            get_hba_params,
            get_hba_cal_mme_params,
            get_hba_visualize_params,
        )
        system_config = load_system_config(config_path)
        out_dir_launch = LaunchConfiguration("output_dir").perform(context)
        if out_dir_launch and isinstance(system_config.get("system"), dict):
            system_config["system"]["output_dir"] = out_dir_launch
        ot_params = get_overlap_transformer_params(system_config)
        if not (ot_params.get("model_path") or "").strip():
            default_ot = resolve_default_overlap_model_path(launch_dir)
            if default_ot:
                ot_params["model_path"] = default_ot
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

    use_external_frontend_val = LaunchConfiguration("use_external_frontend", default="false").perform(context).strip().lower() == "true"
    use_external_overlap_val = LaunchConfiguration("use_external_overlap", default="true").perform(context).strip().lower() == "true"
    use_hba_val = LaunchConfiguration("use_hba", default="true").perform(context).strip().lower() == "true"
    use_hba_cal_mme_val = LaunchConfiguration("use_hba_cal_mme", default="false").perform(context).strip().lower() == "true"
    use_hba_visualize_val = LaunchConfiguration("use_hba_visualize", default="false").perform(context).strip().lower() == "true"
    use_rviz_val = LaunchConfiguration("use_rviz", default="true").perform(context).strip().lower() == "true"
    nodes = []

    # fast-livo2 节点：只使用从 system_config 生成的参数
    try:
        get_package_share_directory("fast_livo")
        if fl2_params and use_external_frontend_val:
            nodes.append(Node(
                package="fast_livo", executable="fastlivo_mapping", name="laserMapping",
                parameters=[fl2_params],
                output="screen",
            ))
    except Exception:
        pass

    if use_external_overlap_val:
        try:
            nodes.append(Node(
                package="overlap_transformer_ros2", executable="descriptor_server",
                name="overlap_transformer_descriptor_server", output="screen",
                parameters=[ot_params],
            ))
        except Exception:
            pass

    # HBA 模块节点（参数来自 system_config.backend.hba / hba_cal_mme / hba_visualize）
    if hba_params and use_hba_val:
        nodes.append(Node(
            package="hba", namespace="hba", executable="hba", name="hba_node",
            output="screen", parameters=[hba_params],
        ))
    if hba_cal_mme_params and use_hba_cal_mme_val:
        nodes.append(Node(
            package="hba", namespace="cal_MME", executable="calculate_MME", name="cal_MME_node",
            output="screen", parameters=[hba_cal_mme_params],
        ))
    if hba_visualize_params and use_hba_visualize_val:
        nodes.append(Node(
            package="hba", namespace="visualize", executable="visualize", name="visualize_node",
            output="screen", parameters=[hba_visualize_params],
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
    if use_rviz_val:
        if not is_rviz2_installed():
            sys.stderr.write(
                "[automap_incremental] [WARN] use_rviz=true 但系统无可用 rviz2；跳过 RViz。"
                "可 apt 安装 ros-{}-rviz2 或传 use_rviz:=false\n".format(
                    os.environ.get("ROS_DISTRO", "<distro>")))
            sys.stderr.flush()
        else:
            nodes.append(Node(
                package="rviz2", executable="rviz2", name="rviz_frontend",
                arguments=["-d", rviz_frontend_config], output="screen",
            ))
            nodes.append(Node(
                package="rviz2", executable="rviz2", name="rviz_backend",
                arguments=["-d", rviz_backend_config], output="screen",
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
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Whether to start RViz",
        ),
        DeclareLaunchArgument("use_external_frontend", default_value="true", description="true=use verified fast_livo node (modular); false=use internal FastLIVO2Wrapper (ESIKF)"),
        DeclareLaunchArgument("use_external_overlap", default_value="true", description="Launch OverlapTransformer descriptor; true=使用 pretrained_overlap_transformer.pth.tar 做回环粗匹配"),
        DeclareLaunchArgument("use_hba", default_value="true", description="Launch HBA backend node (params from system_config.backend.hba)"),
        DeclareLaunchArgument("use_hba_cal_mme", default_value="false", description="Launch HBA cal_MME node (params from system_config.backend.hba_cal_mme)"),
        DeclareLaunchArgument("use_hba_visualize", default_value="false", description="Launch HBA visualize node (params from system_config.backend.hba_visualize)"),
        OpaqueFunction(function=_launch_nodes_incremental),
    ])
