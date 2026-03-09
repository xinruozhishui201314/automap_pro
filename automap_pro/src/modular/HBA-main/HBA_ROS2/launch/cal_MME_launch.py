# cal_MME 节点启动；参数可从 system_config.yaml 的 backend.hba_cal_mme 读取
# 推荐由 automap_pro/launch 统一启动（use_hba_cal_mme:=true）；本文件用于单独调试。

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _cal_mme_params_from_system_config(config):
    """从 system_config 字典提取 backend.hba_cal_mme 参数字典。"""
    cal = config.get("backend", {}).get("hba_cal_mme", {})
    return {
        "file_path": cal.get("file_path", "/data/automap_output/hba_export"),
        "THR_NUM": int(cal.get("thr_num", 16)),
    }


def _launch_cal_mme_node(context, *args, **kwargs):
    system_config_path = LaunchConfiguration("system_config_path", default="").perform(context)
    default_config = PathJoinSubstitution([get_package_share_directory("hba"), "config", "cal_MME.yaml"])

    if system_config_path and os.path.isfile(system_config_path):
        try:
            import yaml
            with open(system_config_path, "r", encoding="utf-8") as f:
                config = yaml.safe_load(f)
            params = _cal_mme_params_from_system_config(config)
            parameters = [params]
        except Exception:
            parameters = [default_config]
    else:
        parameters = [default_config]

    return [
        Node(
            package="hba",
            namespace="cal_MME",
            executable="calculate_MME",
            name="cal_MME_launch",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description():
    default_config = PathJoinSubstitution([get_package_share_directory("hba"), "config", "cal_MME.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument(
            "system_config_path",
            default_value="",
            description="If set, load cal_MME params from backend.hba_cal_mme of this system_config.yaml",
        ),
        LogInfo(msg=[default_config]),
        OpaqueFunction(function=_launch_cal_mme_node),
    ])
