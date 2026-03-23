#!/usr/bin/env python3
# AutoMap-Pro Offline / Bag Replay Launch (ROS2)
# 全工程唯一配置：仅使用 config:= 传入的 YAML，不生成 fast_livo_params.yaml；fast-livo2 / HBA / automap_system 均从此 config 读参

import os
import sys
import subprocess
import traceback
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_LP = "[automap_offline]"


def _log_launch_exception(step_name, e):
    """记录 launch 内异常，便于看到问题原因。"""
    sys.stderr.write("{} [EXCEPTION] step={} type={} message={}\n".format(
        _LP, step_name, type(e).__name__, str(e)))
    try:
        for line in traceback.format_exc().strip().split("\n"):
            sys.stderr.write("{} [EXCEPTION]   {}\n".format(_LP, line))
    except Exception:
        pass
    sys.stderr.flush()


def _launch_nodes_offline(context, *args, **kwargs):
    config_path_raw = (LaunchConfiguration("config").perform(context) or "").strip()
    session_out = ""
    try:
        pkg_share = get_package_share_directory("automap_pro")
    except Exception as e:
        _log_launch_exception("get_package_share_directory(automap_pro)", e)
        pkg_share = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        sys.stderr.write("{} [FALLBACK] 使用 launch 相对路径 pkg_share={}\n".format(_LP, pkg_share))
        sys.stderr.flush()
    # 全工程唯一配置：规范化为绝对路径，确保 load_system_config / fast_livo 临时 YAML / automap_system config_file 均用同一路径
    if config_path_raw and not os.path.isabs(config_path_raw) and os.path.isfile(config_path_raw):
        config_path = os.path.abspath(config_path_raw)
    elif config_path_raw and not os.path.isabs(config_path_raw):
        config_path = os.path.join(pkg_share, "config", os.path.basename(config_path_raw))
        if not os.path.isfile(config_path):
            config_path = config_path_raw
    else:
        config_path = config_path_raw
    if config_path:
        sys.stderr.write("{} [CONFIG] 全工程唯一配置文件: {}\n".format(_LP, os.path.abspath(config_path) if config_path and os.path.isfile(config_path) else config_path))
        sys.stderr.flush()
    # 统一离线会话输出目录：让 fast_livo 与 automap_system 写入同一 run_* 目录，
    # 并覆盖 map.frame_config_path，避免 map_frame.cfg 落在基础目录。
    if config_path and os.path.isfile(config_path):
        try:
            import yaml as _yaml
            with open(config_path, "r", encoding="utf-8") as _f:
                _cfg = _yaml.safe_load(_f) or {}
            _system = _cfg.get("system") if isinstance(_cfg.get("system"), dict) else {}
            _base_out = (_system.get("output_dir") or "").strip() or "/data/automap_output"
            _base_out = os.path.abspath(_base_out.rstrip("/"))
            if "/run_" in _base_out:
                session_out = _base_out
            else:
                session_out = _base_out + "/run_" + datetime.now().strftime("%Y%m%d_%H%M%S")
            os.makedirs(session_out, exist_ok=True)
            os.environ["AUTOMAP_SESSION_OUTPUT_DIR"] = session_out

            _cfg.setdefault("system", {})
            _cfg["system"]["output_dir"] = session_out
            _cfg.setdefault("map", {})
            _cfg["map"]["frame_config_path"] = os.path.join(session_out, "map_frame.cfg")

            _patched_config = "/tmp/automap_offline_system_config_{}.yaml".format(os.getpid())
            with open(_patched_config, "w", encoding="utf-8") as _f:
                _yaml.safe_dump(_cfg, _f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            config_path = _patched_config
            sys.stderr.write("{} [OUTPUT] AUTOMAP_SESSION_OUTPUT_DIR={}\n".format(_LP, session_out))
            sys.stderr.write("{} [CONFIG] offline patched config: {}\n".format(_LP, config_path))
            sys.stderr.flush()
        except Exception as e:
            _log_launch_exception("构建离线 run_* 会话输出目录/补丁配置", e)
    # RViz 配置：前端 / 后端分两个窗口
    rviz_frontend_config = os.path.join(pkg_share, "rviz", "automap_frontend.rviz")
    rviz_backend_config = os.path.join(pkg_share, "rviz", "automap_backend.rviz")
    if not os.path.isfile(rviz_frontend_config):
        rviz_frontend_config = os.path.join(pkg_share, "rviz", "automap.rviz")
    if not os.path.isfile(rviz_backend_config):
        rviz_backend_config = os.path.join(pkg_share, "rviz", "automap.rviz")
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    if launch_dir not in sys.path:
        sys.path.insert(0, launch_dir)
    ot_params = {"model_path": ""}
    fl2_params = {}
    hba_params = {}
    hba_cal_mme_params = {}
    hba_visualize_params = {}
    try:
        from params_from_system_config import (
            load_system_config,
            get_overlap_transformer_params,
            resolve_default_overlap_model_path,
            get_fast_livo2_params,
            get_hba_params,
            get_hba_cal_mme_params,
            get_hba_visualize_params,
            write_fast_livo_params_file,
        )
        system_config = load_system_config(config_path)
        ot_params = get_overlap_transformer_params(system_config)
        if not (ot_params.get("model_path") or "").strip():
            default_ot = resolve_default_overlap_model_path(launch_dir)
            if default_ot:
                ot_params["model_path"] = default_ot
                sys.stderr.write("{} [OVERLAP] model_path 为空，使用仓库内默认: {}\n".format(_LP, default_ot))
                sys.stderr.flush()
        hba_params = get_hba_params(system_config)
        hba_cal_mme_params = get_hba_cal_mme_params(system_config)
        hba_visualize_params = get_hba_visualize_params(system_config)
        fl2_params = get_fast_livo2_params(system_config)
    except Exception as e:
        _log_launch_exception("加载 config 或生成参数(load_system_config/get_*_params)", e)
        sys.stderr.write("{} [FALLBACK] 使用空/默认参数继续启动，部分节点可能不可用\n".format(_LP))
        sys.stderr.flush()

    # 在 OpaqueFunction 内用 .perform(context) 读参数，按需添加节点，避免 IfCondition/PythonExpression 触发 Humble 的 TypeError
    use_external_frontend_val = LaunchConfiguration("use_external_frontend", default="false").perform(context).strip().lower() == "true"
    use_external_overlap_val = LaunchConfiguration("use_external_overlap", default="true").perform(context).strip().lower() == "true"
    use_hba_val = LaunchConfiguration("use_hba", default="true").perform(context).strip().lower() == "true"
    use_hba_cal_mme_val = LaunchConfiguration("use_hba_cal_mme", default="false").perform(context).strip().lower() == "true"
    use_hba_visualize_val = LaunchConfiguration("use_hba_visualize", default="false").perform(context).strip().lower() == "true"
    nodes = []

    # Fast-LIVO2：参数来自全工程唯一 config，写入临时 YAML 再以 parameters=[path] 传入
    if fl2_params and use_external_frontend_val:
        sys.stderr.write("{} [CONFIG] fast_livo 参数来源: 同上唯一 config，写入临时 YAML 再加载\n".format(_LP))
        pb = fl2_params.get("parameter_blackboard") if isinstance(fl2_params.get("parameter_blackboard"), dict) else {}
        sys.stderr.write("{} [FAST_LIVO_PARAMS] fl2_params 顶层键: {}\n".format(_LP, sorted(fl2_params.keys())))
        sys.stderr.write("{} [FAST_LIVO_PARAMS] parameter_blackboard 键: {}, model={!r}\n".format(
            _LP, sorted(pb.keys()), pb.get("model")))
        if not (pb.get("model") or "").strip():
            sys.stderr.write("{} [FAST_LIVO_PARAMS] [WARN] parameter_blackboard.model 为空，将导致 Camera model not specified\n".format(_LP))
            if "parameter_blackboard" not in fl2_params or not isinstance(fl2_params["parameter_blackboard"], dict):
                fl2_params["parameter_blackboard"] = {}
            fl2_params["parameter_blackboard"].setdefault("model", "Pinhole")
            fl2_params["parameter_blackboard"].setdefault("width", 752)
            fl2_params["parameter_blackboard"].setdefault("height", 480)
            fl2_params["parameter_blackboard"].setdefault("scale", 1.0)
            for k in ("fx", "fy", "cx", "cy", "d0", "d1", "d2", "d3"):
                fl2_params["parameter_blackboard"].setdefault(k, 0.0 if k.startswith("d") else 400.0)
            sys.stderr.write("{} [FAST_LIVO_PARAMS] 已注入兜底 parameter_blackboard.model=Pinhole\n".format(_LP))
        sys.stderr.flush()
        try:
            import yaml as _yaml
            fast_livo_params_file = "/tmp/automap_fl_params_offline_{}.yaml".format(os.getpid())
            write_fast_livo_params_file(system_config, fast_livo_params_file, config_path)
            with open(fast_livo_params_file, "r", encoding="utf-8") as _f:
                _data = _yaml.safe_load(_f)
            _node_key = "laserMapping" if "laserMapping" in _data else "/laserMapping"
            _data.setdefault(_node_key, {}).setdefault("ros__parameters", {})["use_sim_time"] = True
            with open(fast_livo_params_file, "w", encoding="utf-8") as _f:
                _yaml.dump(_data, _f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            get_package_share_directory("fast_livo")
            run_fast_livo_under_gdb = LaunchConfiguration("run_fast_livo_under_gdb", default="false").perform(context).lower() == "true"
            fast_livo_prefix = []
            if run_fast_livo_under_gdb and os.path.isfile(gdb_wrapper := os.path.join(launch_dir, "run_under_gdb.sh")):
                fast_livo_prefix = [gdb_wrapper]
                sys.stderr.write("{} [GDB] fastlivo_mapping 将以 GDB 启动，崩溃时自动打印 backtrace\n".format(_LP))
                sys.stderr.flush()
            nodes.append(Node(
                package="fast_livo",
                executable="fastlivo_mapping",
                name="laserMapping",
                parameters=[fast_livo_params_file],
                output="screen",
                prefix=fast_livo_prefix,
            ))
        except Exception as e:
            _log_launch_exception("创建 fast_livo Node(get_package_share_directory 或 Node())", e)
            sys.stderr.write("{} [FALLBACK] 未添加 fast_livo 节点，请检查 fast_livo 是否已编译安装\n".format(_LP))
            sys.stderr.flush()

    if use_external_overlap_val:
        try:
            nodes.append(Node(
                package="overlap_transformer_ros2", executable="descriptor_server",
                name="overlap_transformer_descriptor_server", output="screen",
                parameters=[ot_params],
            ))
        except Exception as e:
            _log_launch_exception("创建 overlap_transformer_ros2 Node", e)
            sys.stderr.write("{} [FALLBACK] 未添加 overlap_transformer 节点\n".format(_LP))
            sys.stderr.flush()

    # HBA 模块节点（参数来自 system_config.backend.hba / hba_cal_mme / hba_visualize）
    if hba_params and use_hba_val:
        try:
            nodes.append(Node(
                package="hba", namespace="hba", executable="hba", name="hba_node",
                output="screen", parameters=[hba_params],
            ))
        except Exception as e:
            _log_launch_exception("创建 HBA 主节点", e)
            sys.stderr.write("{} [FALLBACK] 未添加 hba_node\n".format(_LP))
            sys.stderr.flush()
    if hba_cal_mme_params and use_hba_cal_mme_val:
        try:
            nodes.append(Node(
                package="hba", namespace="cal_MME", executable="calculate_MME", name="cal_MME_node",
                output="screen", parameters=[hba_cal_mme_params],
            ))
        except Exception as e:
            _log_launch_exception("创建 HBA cal_MME 节点", e)
    if hba_visualize_params and use_hba_visualize_val:
        try:
            nodes.append(Node(
                package="hba", namespace="visualize", executable="visualize", name="visualize_node",
                output="screen", parameters=[hba_visualize_params],
            ))
        except Exception as e:
            _log_launch_exception("创建 HBA visualize 节点", e)

    # AutoMap System 节点：话题映射由 system_config 决定；节点读取的是 config_file 参数（非 config）
    config_path_str = (config_path or "").strip() if config_path else ""
    if not config_path_str:
        sys.stderr.write("{} [WARN] config_path 为空，automap_system 将使用默认配置\n".format(_LP))
        sys.stderr.flush()
    run_automap_under_gdb = LaunchConfiguration("run_automap_under_gdb", default="false").perform(context).lower() == "true"
    # 可选：预加载 libgtsam 以尝试规避 lago 静态初始化 double free（见 docs/FIX_GTSAM_LAGO_STATIC_INIT_DOUBLE_FREE.md）
    gtsam_preload_path = LaunchConfiguration("gtsam_preload_path", default="").perform(context).strip()
    automap_env = {}
    if gtsam_preload_path and os.path.isfile(gtsam_preload_path):
        automap_env["LD_PRELOAD"] = gtsam_preload_path
        sys.stderr.write("{} [GTSAM] LD_PRELOAD={} (avoid lago static-init double free)\n".format(_LP, gtsam_preload_path))
        sys.stderr.flush()
    elif gtsam_preload_path:
        sys.stderr.write("{} [WARN] gtsam_preload_path 指定但文件不存在: {}，忽略\n".format(_LP, gtsam_preload_path))
        sys.stderr.flush()
    if session_out:
        automap_env["AUTOMAP_SESSION_OUTPUT_DIR"] = session_out
    # launch_ros Node 的 prefix 多元素 list 会被错误拼接；用包装脚本可靠调用 gdb
    gdb_wrapper = os.path.join(launch_dir, "run_under_gdb.sh")
    automap_prefix = [gdb_wrapper] if (run_automap_under_gdb and os.path.isfile(gdb_wrapper)) else []
    if run_automap_under_gdb:
        if automap_prefix:
            sys.stderr.write("{} [GDB] automap_system_node 将以 GDB 启动，崩溃时自动打印 backtrace\n".format(_LP))
        else:
            sys.stderr.write("{} [GDB] 未找到 {}，请安装 gdb 并确保 launch 目录存在 run_under_gdb.sh，跳过 GDB\n".format(_LP, gdb_wrapper))
        sys.stderr.flush()
    automap_system_node = None
    try:
        node_params = [{"config_file": config_path_str}, {"use_sim_time": True}]
        if session_out:
            node_params.append({"output_dir": session_out})
        node_kw = dict(
            package="automap_pro", executable="automap_system_node", name="automap_system",
            output="screen",
            parameters=node_params,
            prefix=automap_prefix,
        )
        if automap_env:
            node_kw["additional_env"] = automap_env
        automap_system_node = Node(**node_kw)
        nodes.append(automap_system_node)
    except Exception as e:
        _log_launch_exception("创建 automap_system Node", e)
        sys.stderr.write("{} [ERROR] automap_system 节点未添加，建图核心不可用\n".format(_LP))
        sys.stderr.flush()
    use_rviz_val = LaunchConfiguration("use_rviz", default="true").perform(context).strip().lower() == "true"
    if use_rviz_val:
        try:
            nodes.append(Node(
                package="rviz2", executable="rviz2", name="rviz_frontend",
                arguments=["-d", rviz_frontend_config], output="screen",
            ))
            nodes.append(Node(
                package="rviz2", executable="rviz2", name="rviz_backend",
                arguments=["-d", rviz_backend_config], output="screen",
            ))
        except Exception as e:
            _log_launch_exception("创建 rviz2 Node", e)
    try:
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher", name="world_map_tf",
            arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
        ))
    except Exception as e:
        _log_launch_exception("创建 static_transform_publisher Node", e)
    # Fast-LIVO2 使用 frame_id=camera_init，RViz Fixed Frame=map；需发布 map->camera_init 才能显示点云/轨迹
    try:
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher", name="map_camera_init_tf",
            arguments=["0", "0", "0", "0", "0", "0", "map", "camera_init"],
        ))
    except Exception as e:
        _log_launch_exception("创建 map_camera_init_tf Node", e)
    # 结束建图后全部进程关闭：automap_system 退出（finish_mapping → rclcpp::shutdown）时触发 launch Shutdown，终止 fast_livo / rviz / overlap 等所有进程
    if automap_system_node is not None:
        nodes.append(RegisterEventHandler(
            OnProcessExit(
                target_action=automap_system_node,
                on_exit=[EmitEvent(event=Shutdown(reason="finish_mapping: automap_system exited"))],
            )
        ))
    return nodes


def _log_bag_topics(bag_file):
    """调用 ros2 bag info 或读取 metadata.yaml 获取 bag 内发布的话题名称。"""
    topics = []

    def parse_ros2_bag_info_stdout(stdout):
        """解析 ros2 bag info 输出：支持 'Topics:' 与 'Topic information:' 两种节标题及多种行格式。"""
        out_topics = []
        in_topics = False
        for line in stdout.splitlines():
            stripped = line.strip()
            if stripped == "Topics:" or "Topic information" in stripped:
                in_topics = True
                continue
            if not in_topics or not stripped:
                if in_topics and not stripped:
                    break
                continue
            # 格式1: "  /topic_name: msg_type (count msgs)"
            parts = stripped.split(":", 1)
            if len(parts) >= 1 and parts[0].strip().startswith("/"):
                out_topics.append(parts[0].strip())
                continue
            # 格式2: "Topic: /name | Type: ..." (Humble ros2 bag info)
            if "Topic:" in stripped and "/" in stripped:
                for seg in stripped.replace("|", " ").split():
                    if seg.startswith("/") and seg not in out_topics:
                        out_topics.append(seg)
                        break
        return out_topics

    try:
        result = subprocess.run(
            ["ros2", "bag", "info", bag_file],
            capture_output=True,
            text=True,
            timeout=10,
            env=os.environ.copy(),
        )
        if result.returncode == 0 and result.stdout:
            topics = parse_ros2_bag_info_stdout(result.stdout)
    except FileNotFoundError:
        pass  # 下方用 metadata 回退
    except subprocess.TimeoutExpired:
        sys.stderr.write("{} [BAG] [WARN] ros2 bag info 超时，尝试从 metadata 读取\n".format(_LP))
    except Exception as e:
        sys.stderr.write("{} [BAG] [WARN] ros2 bag info 失败: {}，尝试从 metadata 读取\n".format(_LP, e))

    # 回退：bag 目录下的 metadata.yaml（rosbag2 标准结构）
    if not topics and bag_file:
        try:
            import yaml
            meta_dir = bag_file if os.path.isdir(bag_file) else os.path.dirname(bag_file)
            meta_path = os.path.join(meta_dir, "metadata.yaml")
            if os.path.isfile(meta_path):
                with open(meta_path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f)
                for t in (data or {}).get("rosbag2_bagfile_information", {}).get("topics_with_message_count", []):
                    name = (t.get("topic_metadata") or {}).get("name")
                    if name and name not in topics:
                        topics.append(name)
        except Exception:
            pass
    return topics


def _log_bag_path(context, *args, **kwargs):
    """启动时打印 bag 路径、bag 内发布的话题名称与依赖提示。"""
    bag_file = LaunchConfiguration("bag_file").perform(context)
    sys.stderr.write("{} [BAG] 离线回放 bag_file={}\n".format(_LP, bag_file))

    topics = _log_bag_topics(bag_file)
    if topics:
        sys.stderr.write("{} [BAG] rosbag2 将发布的话题 ({} 个): {}\n".format(
            _LP, len(topics), ", ".join(topics)))
    else:
        sys.stderr.write("{} [BAG] 无法获取 bag 话题列表（请确认 bag 路径正确且为 ROS2 格式）\n".format(_LP))

    sys.stderr.write("{} [BAG] 若 ros2 bag play 报 Exception on parsing info file / bad conversion，将无数据；odom 来自 fast_livo，请确保 config 中 lid_topic/imu_topic 与 bag 一致\n".format(_LP))
    sys.stderr.write("{} [BAG] 离线模式默认不启动 standalone HBA 节点(use_hba=false)，后端优化由 automap_system 内 HBAOptimizer 负责\n".format(_LP))
    sys.stderr.flush()


def generate_launch_description():
    pkg_share = get_package_share_directory("automap_pro")
    config_default = os.path.join(pkg_share, "config", "system_config.yaml")
    # bag 播完后调用 finish_mapping，执行最终 HBA + 保存 + shutdown（离线“播完再结束”）
    bag_play_action = ExecuteProcess(
        cmd=["ros2", "bag", "play", LaunchConfiguration("bag_file"), "--rate", LaunchConfiguration("rate"), "--clock"],
        output="screen",
    )
    finish_after_bag_action = ExecuteProcess(
        cmd=["bash", "-c", "sleep 5 && ros2 service call /automap/finish_mapping std_srvs/srv/Trigger '{}'"],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=config_default, description="Path to system_config.yaml"),
        DeclareLaunchArgument("bag_file", default_value=os.path.expanduser("~/data/mapping.db3"), description="Path to rosbag2"),
        DeclareLaunchArgument("rate", default_value="0.5", description="Playback rate (0.5=half speed, 1.0=realtime)"),
        DeclareLaunchArgument("use_rviz", default_value="true", description="Whether to start RViz"),
        DeclareLaunchArgument("use_external_frontend", default_value="true", description="true=use verified fast_livo node (modular); false=use internal FastLIVO2Wrapper (ESIKF)"),
        DeclareLaunchArgument("use_external_overlap", default_value="true", description="Launch OverlapTransformer descriptor; true=使用 pretrained_overlap_transformer.pth.tar 做回环粗匹配"),
        DeclareLaunchArgument("use_hba", default_value="false", description="Launch standalone HBA node (reads pose.json at startup; offline 时默认 false 避免零位姿崩溃，优化由 automap_system 内 HBAOptimizer 负责)"),
        DeclareLaunchArgument("use_hba_cal_mme", default_value="false", description="Launch HBA cal_MME node (params from system_config.backend.hba_cal_mme)"),
        DeclareLaunchArgument("use_hba_visualize", default_value="false", description="Launch HBA visualize node (params from system_config.backend.hba_visualize)"),
        DeclareLaunchArgument("run_automap_under_gdb", default_value="false", description="Run automap_system_node under GDB; on crash prints full backtrace (requires gdb in container)"),
        DeclareLaunchArgument("gtsam_preload_path", default_value="", description="If set to path of libgtsam.so.4, set LD_PRELOAD to try avoiding lago static-init double free (see docs/FIX_GTSAM_LAGO_STATIC_INIT_DOUBLE_FREE.md)"),
        DeclareLaunchArgument("run_fast_livo_under_gdb", default_value="false", description="Run fastlivo_mapping under GDB; use when frontend SIGSEGV to get backtrace (e.g. frame=10 crash)"),
        OpaqueFunction(function=_log_bag_path),
        bag_play_action,
        OpaqueFunction(function=_launch_nodes_offline),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=bag_play_action,
                on_exit=[finish_after_bag_action],
            )
        ),
    ])
