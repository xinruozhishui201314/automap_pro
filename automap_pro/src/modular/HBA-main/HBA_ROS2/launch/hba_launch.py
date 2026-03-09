# HBA 主节点启动；参数可从 system_config.yaml 的 backend.hba 读取
# 推荐由 automap_pro/launch（automap_offline/online/incremental）统一启动；本文件用于单独调试时：
#   ros2 launch hba hba_launch.py system_config_path:=/path/to/automap_pro/config/system_config.yaml

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _hba_params_from_system_config(config):
    """从 system_config 字典提取 backend.hba 参数字典（与 automap_pro/launch/params_from_system_config 一致）。"""
    hba = config.get("backend", {}).get("hba", {})
    return {
        "data_path": hba.get("data_path", "/data/automap_output/hba_export"),
        "total_layer_num": int(hba.get("total_layer_num", 3)),
        "pcd_name_fill_num": int(hba.get("pcd_name_fill_num", 0)),
        "thread_num": int(hba.get("thread_num", 16)),
        "enable_gps_factor": bool(hba.get("enable_gps_factor", True)),
    }


def _launch_hba_node(context, *args, **kwargs):
    system_config_path = LaunchConfiguration("system_config_path", default="").perform(context)
    default_config = PathJoinSubstitution([get_package_share_directory("hba"), "config", "hba.yaml"])

    if system_config_path and os.path.isfile(system_config_path):
        try:
            import yaml
            with open(system_config_path, "r", encoding="utf-8") as f:
                config = yaml.safe_load(f)
            params = _hba_params_from_system_config(config)
            parameters = [params]
        except Exception:
            parameters = [default_config]
    else:
        parameters = [default_config]

    return [
        Node(
            package="hba",
            namespace="hba",
            executable="hba",
            name="hba_launch",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description():
    default_config = PathJoinSubstitution([get_package_share_directory("hba"), "config", "hba.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument(
            "system_config_path",
            default_value="",
            description="If set, load HBA params from backend.hba of this system_config.yaml; else use config/hba.yaml",
        ),
        LogInfo(msg=[default_config]),
        OpaqueFunction(function=_launch_hba_node),
    ])
