#!/usr/bin/env python3
# AutoMap-Pro Offline / Bag Replay Launch (ROS2)
# 从 system_config.yaml 读取并启动：fast-livo2、overlap_transformer_ros2、HBA、automap_system

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes_offline(context, *args, **kwargs):
    config_path = LaunchConfiguration("config").perform(context)
    pkg_share = get_package_share_directory("automap_pro")
    rviz_config_default = os.path.join(pkg_share, "config", "automap.rviz")
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    # fast_livo 参数固定从 automap_pro/config/system_config.yaml 读取，不使用 config:= 传入的其它配置
    config_dir = os.path.join(os.path.dirname(launch_dir), "config")
    system_config_yaml_path = os.path.join(config_dir, "system_config.yaml")
    if launch_dir not in __import__("sys").path:
        __import__("sys").path.insert(0, launch_dir)
    try:
        from params_from_system_config import (
            load_system_config,
            get_overlap_transformer_params,
            get_fast_livo2_params,
            write_fast_livo_params_file,
            get_hba_params,
            get_hba_cal_mme_params,
            get_hba_visualize_params,
        )
        system_config = load_system_config(config_path)
        ot_params = get_overlap_transformer_params(system_config)
        hba_params = get_hba_params(system_config)
        hba_cal_mme_params = get_hba_cal_mme_params(system_config)
        hba_visualize_params = get_hba_visualize_params(system_config)
        # fast_livo 固定使用 automap_pro/config/system_config.yaml，不使用 config:= 传入的其它配置
        if os.path.isfile(system_config_yaml_path):
            system_config_for_fast_livo = load_system_config(system_config_yaml_path)
            fl2_params = get_fast_livo2_params(system_config_for_fast_livo)
            config_used_for_fast_livo_path = system_config_yaml_path
        else:
            system_config_for_fast_livo = system_config
            fl2_params = get_fast_livo2_params(system_config)
            config_used_for_fast_livo_path = config_path
    except Exception:
        ot_params = {"model_path": ""}
        fl2_params = {}
        hba_params = {}
        hba_cal_mme_params = {}
        hba_visualize_params = {}
        system_config_for_fast_livo = None
        config_used_for_fast_livo_path = None

    use_external_frontend = LaunchConfiguration("use_external_frontend", default="false")
    use_external_overlap = LaunchConfiguration("use_external_overlap", default="false")
    use_hba = LaunchConfiguration("use_hba", default="true")
    use_hba_cal_mme = LaunchConfiguration("use_hba_cal_mme", default="false")
    use_hba_visualize = LaunchConfiguration("use_hba_visualize", default="false")
    nodes = []

    # Fast-LIVO2：用 ExecuteProcess + --params-file 直接加载我们写的 YAML，避免 launch 合并参数时产生 parameter ''
    # 参数来源固定为 automap_pro/config/system_config.yaml（与 config:= 无关）
    # 日志目录：优先环境变量 AUTOMAP_LOG_DIR（与工程统一 logs/ 一致），否则 automap_pro/logs
    if fl2_params:
        log_dir = os.environ.get("AUTOMAP_LOG_DIR") or os.path.join(os.path.dirname(launch_dir), "logs")
        os.makedirs(log_dir, exist_ok=True)
        fl2_params_path = os.path.join(log_dir, "fast_livo_params.yaml")
        fl2_params_abs = os.path.abspath(fl2_params_path)
        fl2_params_real = os.path.realpath(fl2_params_path) if os.path.exists(fl2_params_path) else "(file not yet written)"
        launch_file_abs = os.path.abspath(__file__)
        launch_dir_abs = os.path.abspath(launch_dir)
        log_dir_abs = os.path.abspath(log_dir)
        config_abs = os.path.abspath(config_path) if config_path else None
        import sys
        _lp = "[LINK_3_LAUNCH]"
        sys.stderr.write("{} [PATHS] launch_file(absolute)={}\n".format(_lp, launch_file_abs))
        sys.stderr.write("{} [PATHS] launch_dir(absolute)={}\n".format(_lp, launch_dir_abs))
        sys.stderr.write("{} [PATHS] log_dir(absolute)={}\n".format(_lp, log_dir_abs))
        sys.stderr.write("{} [PATHS] config_path(raw)={}\n".format(_lp, config_path))
        sys.stderr.write("{} [PATHS] config_path(absolute)={}\n".format(_lp, config_abs))
        sys.stderr.write("{} [PATHS] fast_livo_config(used)={}\n".format(_lp, config_used_for_fast_livo_path or "(none)"))
        sys.stderr.write("{} [PATHS] fast_livo_params_path(raw)={}\n".format(_lp, fl2_params_path))
        sys.stderr.write("{} [PATHS] fast_livo_params_path(absolute)={}\n".format(_lp, fl2_params_abs))
        sys.stderr.write("{} [PATHS] fast_livo_params_path(realpath)={}\n".format(_lp, fl2_params_real))
        sys.stderr.write("{} [PATHS] params_file_for_fast_livo(use_this_in_grep)={}\n".format(_lp, fl2_params_abs))
        sys.stderr.flush()
        write_fast_livo_params_file(system_config_for_fast_livo, fl2_params_path, config_path=config_used_for_fast_livo_path)
        fl2_params_real_after = os.path.realpath(fl2_params_path) if os.path.exists(fl2_params_path) else fl2_params_abs
        sys.stderr.write("{} [PATHS] fast_livo_params_path(realpath_after_write)={}\n".format(_lp, fl2_params_real_after))
        # 使用绝对路径传给 --params-file，避免子进程 cwd 不同导致读错文件
        fast_livo_cmd = ["ros2", "run", "fast_livo", "fastlivo_mapping", "--ros-args", "-r", "__node:=laserMapping", "-p", "use_sim_time:=true", "--params-file", fl2_params_abs]
        sys.stderr.write("{} [CMD] fastlivo_mapping full_cmd={}\n".format(_lp, fast_livo_cmd))
        sys.stderr.write("{} [CMD] --params-file value(absolute)={}\n".format(_lp, fl2_params_abs))
        sys.stderr.write("{} [DIAG] 若 ros2-2 仍报 parameter ''，请确认建图前已编译 fast_livo；启动后应看到 stderr: [fast_livo] main: automatically_declare_parameters_from_overrides=false\n".format(_lp))
        try:
            import subprocess
            p = subprocess.run(["ros2", "pkg", "prefix", "fast_livo"], capture_output=True, text=True, timeout=5)
            if p.returncode == 0 and p.stdout:
                prefix = p.stdout.strip()
                import os as _os
                exe_path = _os.path.join(prefix, "lib", "fast_livo", "fastlivo_mapping")
                sys.stderr.write("{} [DIAG] fast_livo 安装路径 prefix={} exe={}\n".format(_lp, prefix, exe_path))
        except Exception as e:
            sys.stderr.write("{} [DIAG] 无法解析 fast_livo 安装路径: {}\n".format(_lp, e))
        sys.stderr.flush()
        from launch.actions import ExecuteProcess as EP
        nodes.append(EP(
            cmd=fast_livo_cmd,
            output="screen",
            condition=IfCondition(use_external_frontend),
        ))

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

    # AutoMap System 节点：话题映射由 system_config.yaml 决定，不使用固定的 remappings
    # remappings 会覆盖系统配置中的话题名，导致话题不匹配
    nodes.append(Node(
        package="automap_pro", executable="automap_system_node", name="automap_system",
        output="screen",
        parameters=[{"config": LaunchConfiguration("config")}, {"use_sim_time": True}],
        # 移除固定 remappings，让系统从 system_config.yaml 读取话题名
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
        DeclareLaunchArgument("bag_file", default_value=os.path.expanduser("~/data/mapping.db3"), description="Path to rosbag2"),
        DeclareLaunchArgument("rate", default_value="1.0", description="Playback rate"),
        DeclareLaunchArgument("use_rviz", default_value="true", description="Whether to start RViz"),
        DeclareLaunchArgument("use_external_frontend", default_value="true", description="true=use verified fast_livo node (modular); false=use internal FastLIVO2Wrapper (ESIKF)"),
        DeclareLaunchArgument("use_external_overlap", default_value="false", description="Launch OverlapTransformer descriptor (params from system_config)"),
        DeclareLaunchArgument("use_hba", default_value="true", description="Launch HBA backend node (params from system_config.backend.hba)"),
        DeclareLaunchArgument("use_hba_cal_mme", default_value="false", description="Launch HBA cal_MME node (params from system_config.backend.hba_cal_mme)"),
        DeclareLaunchArgument("use_hba_visualize", default_value="false", description="Launch HBA visualize node (params from system_config.backend.hba_visualize)"),
        ExecuteProcess(cmd=["ros2", "bag", "play", LaunchConfiguration("bag_file"), "--rate", LaunchConfiguration("rate"), "--clock"], output="screen"),
        OpaqueFunction(function=_launch_nodes_offline),
    ])
