# HBA 可视化节点启动；参数可从 system_config.yaml 的 backend.hba_visualize 读取
# 推荐由 automap_pro/launch 统一启动（use_hba_visualize:=true）；本文件用于单独调试。

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import launch_ros
from launch_ros.actions import Node


def _visualize_params_from_system_config(config):
    """从 system_config 字典提取 backend.hba_visualize 参数字典。"""
    viz = config.get("backend", {}).get("hba_visualize", {})
    return {
        "file_path": viz.get("file_path", "/data/automap_output/hba_export"),
        "downsample_size": float(viz.get("downsample_size", 0.1)),
        "pcd_name_fill_num": int(viz.get("pcd_name_fill_num", 0)),
        "marker_size": float(viz.get("marker_size", 0.5)),
    }


def _launch_visualize_node(context, *args, **kwargs):
    system_config_path = LaunchConfiguration("system_config_path", default="").perform(context)
    default_config = PathJoinSubstitution([get_package_share_directory("hba"), "config", "visualize.yaml"])
    rviz_cfg = PathJoinSubstitution([get_package_share_directory("hba"), "rviz_cfg", "visualize.rviz"])

    if system_config_path and os.path.isfile(system_config_path):
        try:
            import yaml
            with open(system_config_path, "r", encoding="utf-8") as f:
                config = yaml.safe_load(f)
            params = _visualize_params_from_system_config(config)
            parameters = [params]
        except Exception:
            parameters = [default_config]
    else:
        parameters = [default_config]

    return [
        launch_ros.actions.Node(
            package="hba",
            namespace="visualize",
            executable="visualize",
            name="visualize_launch",
            output="screen",
            parameters=parameters,
        ),
        launch_ros.actions.Node(
            package="rviz2",
            namespace="visualize",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_cfg.perform(context)],
        ),
    ]


def generate_launch_description():
    default_config = PathJoinSubstitution([get_package_share_directory("hba"), "config", "visualize.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument(
            "system_config_path",
            default_value="",
            description="If set, load visualize params from backend.hba_visualize of this system_config.yaml",
        ),
        LogInfo(msg=[default_config]),
        OpaqueFunction(function=_launch_visualize_node),
    ])
