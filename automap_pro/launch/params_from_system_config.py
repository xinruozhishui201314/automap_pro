#!/usr/bin/env python3
"""
从 system_config.yaml 生成 overlap_transformer_ros2、HBA、fast-livo2 的 ROS2 节点参数字典。
保证参数唯一来源于 system_config，无重复。

配置与代码键对应（避免 YAML 与读取逻辑不匹配）：
- overlap_transformer: loop_closure.overlap_transformer.*, range_image.height/width
- HBA: backend.hba (data_path, total_layer_num, pcd_name_fill_num, thread_num, enable_gps_factor)
- HBA cal_MME: backend.hba_cal_mme (file_path, thr_num)
- HBA visualize: backend.hba_visualize (file_path, downsample_size, pcd_name_fill_num, marker_size)
- fast_livo2: sensor.lidar.topic, sensor.imu.topic, frontend.fast_livo2.common/extrin_calib/time_offset/
  preprocess/vio/imu/lio/local_map/uav/publish/evo/pcd_save/image_save
  common 内 lid_topic/imu_topic 由 sensor 覆盖；img_topic 来自 fast_livo2.common.img_topic
"""

import os
import re
import sys
import traceback
import yaml

# 环节日志：精准定位，grep "LINK_4_PARAMS" 可追踪参数生成层全流程
_LINK4 = "[LINK_4_PARAMS]"

# 诊断日志：输出到 stderr，带标签便于 grep 精准定位（如 grep "\[KEYS\]" 或 "LINK_4_PARAMS"）
def _diag(msg, level="INFO", tag="DIAG"):
    prefix = "{} [params_from_system_config][{}]".format(_LINK4, tag)
    if level == "ERROR":
        sys.stderr.write("{} [ERROR] {}\n".format(prefix, msg))
    elif level == "WARN":
        sys.stderr.write("{} [WARN] {}\n".format(prefix, msg))
    else:
        sys.stderr.write("{} {}\n".format(prefix, msg))
    sys.stderr.flush()


def _log_exception(step_name, e, tag="EXCEPTION"):
    """统一记录异常：类型、消息、堆栈，便于精准定位问题原因。"""
    _diag("step=[{}] exception_type={} message={}".format(
        step_name, type(e).__name__, str(e)), level="ERROR", tag=tag)
    try:
        tb = traceback.format_exc()
        if tb:
            for line in tb.strip().split("\n"):
                sys.stderr.write("{} [params_from_system_config][{}]   {}\n".format(_LINK4, tag, line))
        sys.stderr.flush()
    except Exception:
        pass


def load_system_config(path):
    """
    加载 system_config.yaml，返回字典。
    文件不存在或 YAML 解析失败时返回空字典并打日志，避免调用方因 KeyError/TypeError 报错。
    """
    if not path or not isinstance(path, str):
        _diag("load_system_config: path 为空或非字符串", level="WARN", tag="CONFIG")
        return {}
    path_str = path.strip()
    if not path_str:
        _diag("load_system_config: path 仅空白", level="WARN", tag="CONFIG")
        return {}
    if not os.path.isfile(path_str):
        _diag("load_system_config: 文件不存在 path={}".format(path_str), level="WARN", tag="CONFIG")
        return {}
    try:
        with open(path_str, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if data is None:
            _diag("load_system_config: YAML 内容为空 path={}".format(path_str), level="WARN", tag="CONFIG")
            return {}
        if not isinstance(data, dict):
            _diag("load_system_config: 根节点非 dict path={} type={}".format(path_str, type(data).__name__), level="WARN", tag="CONFIG")
            return {}
        return data
    except yaml.YAMLError as e:
        _diag("load_system_config: YAML 解析失败 path={} err={}".format(path_str, e), level="ERROR", tag="CONFIG")
        return {}
    except (IOError, OSError) as e:
        _diag("load_system_config: 读取失败 path={} err={}".format(path_str, e), level="ERROR", tag="CONFIG")
        return {}
    except Exception as e:
        _diag("load_system_config: 未知错误 path={} err={}".format(path_str, e), level="ERROR", tag="CONFIG")
        _log_exception("load_system_config", e, tag="CONFIG")
        return {}


def _safe_int(val, default=0):
    """避免 None 或非法类型导致 int() 抛错，与配置文件匹配。"""
    if val is None:
        return default
    try:
        return int(val)
    except (TypeError, ValueError):
        return default


def _safe_float(val, default=0.0):
    """避免 None 或非法类型导致 float() 抛错。"""
    if val is None:
        return default
    try:
        return float(val)
    except (TypeError, ValueError):
        return default


def _safe_bool(val, default=False):
    """YAML 可能为 true/false 或 0/1 或字符串，统一为 bool。"""
    if val is None:
        return default
    if isinstance(val, bool):
        return val
    if isinstance(val, (int, float)):
        return bool(val)
    if isinstance(val, str):
        return val.strip().lower() in ("true", "1", "yes", "on")
    return default


def resolve_default_overlap_model_path(launch_dir):
    """
    当 config 中 model_path 为空时，解析仓库内捆绑的 pretrained_overlap_transformer.pth.tar 路径。
    launch_dir 通常为 automap_pro/launch 的绝对路径；返回的路径供 Python descriptor_server 加载 .pth.tar。
    若文件不存在则返回空字符串。
    """
    if not launch_dir or not os.path.isdir(launch_dir):
        return ""
    # automap_pro/launch -> automap_pro/src/modular/OverlapTransformer-master/model/...
    default_path = os.path.join(
        launch_dir, "..", "src", "modular", "OverlapTransformer-master",
        "model", "pretrained_overlap_transformer.pth.tar"
    )
    abs_path = os.path.abspath(default_path)
    return abs_path if os.path.isfile(abs_path) else ""


def get_overlap_transformer_params(config):
    """
    从 system_config 的 loop_closure.overlap_transformer 生成 descriptor_server 节点参数。
    节点使用 proj_H/proj_W。兼容两种写法：
    - 顶层 proj_H/proj_W（如 system_config_M2DGR.yaml）
    - range_image.height/width（与 proj_H/proj_W 一一对应）
    使用 _safe_* 避免配置缺失或类型不符时报错。
    """
    if not isinstance(config, dict):
        return {"model_path": "", "fov_up": 3.0, "fov_down": -25.0, "proj_H": 64, "proj_W": 900, "max_range": 80.0}
    ot = config.get("loop_closure") if isinstance(config.get("loop_closure"), dict) else {}
    ot = ot.get("overlap_transformer") if isinstance(ot.get("overlap_transformer"), dict) else {}
    ri = ot.get("range_image") if isinstance(ot.get("range_image"), dict) else {}
    # 优先顶层 proj_H/proj_W（M2DGR 等），否则用 range_image.height/width
    proj_h = _safe_int(ot.get("proj_H"), _safe_int(ri.get("height"), 64))
    proj_w = _safe_int(ot.get("proj_W"), _safe_int(ri.get("width"), 900))
    return {
        "model_path": ot.get("model_path") if isinstance(ot.get("model_path"), str) else "",
        "fov_up": _safe_float(ot.get("fov_up"), 3.0),
        "fov_down": _safe_float(ot.get("fov_down"), -25.0),
        "proj_H": proj_h,
        "proj_W": proj_w,
        "max_range": _safe_float(ot.get("max_range"), 80.0),
    }


def get_hba_params(config):
    """从 system_config 的 backend.hba 生成 HBA 主节点参数。键与 system_config.yaml 的 backend.hba 对应。"""
    if not isinstance(config, dict):
        return {"data_path": "/data/automap_output/hba_export", "total_layer_num": 3, "pcd_name_fill_num": 0, "thread_num": 16, "enable_gps_factor": True}
    backend = config.get("backend") if isinstance(config.get("backend"), dict) else {}
    hba = backend.get("hba") if isinstance(backend.get("hba"), dict) else {}
    return {
        "data_path": hba.get("data_path") if isinstance(hba.get("data_path"), str) else "/data/automap_output/hba_export",
        "total_layer_num": _safe_int(hba.get("total_layer_num"), 3),
        "pcd_name_fill_num": _safe_int(hba.get("pcd_name_fill_num"), 0),
        "thread_num": _safe_int(hba.get("thread_num"), 16),
        "enable_gps_factor": _safe_bool(hba.get("enable_gps_factor"), True),
    }


def get_hba_cal_mme_params(config):
    """从 system_config 的 backend.hba_cal_mme 生成 cal_MME 节点参数。键 thr_num 对应 YAML 的 thr_num。"""
    if not isinstance(config, dict):
        return {"file_path": "/data/automap_output/hba_export", "THR_NUM": 16}
    backend = config.get("backend") if isinstance(config.get("backend"), dict) else {}
    cal = backend.get("hba_cal_mme") if isinstance(backend.get("hba_cal_mme"), dict) else {}
    return {
        "file_path": cal.get("file_path") if isinstance(cal.get("file_path"), str) else "/data/automap_output/hba_export",
        "THR_NUM": _safe_int(cal.get("thr_num"), 16),
    }


def get_hba_visualize_params(config):
    """从 system_config 的 backend.hba_visualize 生成 visualize 节点参数。"""
    if not isinstance(config, dict):
        return {"file_path": "/data/automap_output/hba_export", "downsample_size": 0.1, "pcd_name_fill_num": 0, "marker_size": 0.5}
    backend = config.get("backend") if isinstance(config.get("backend"), dict) else {}
    viz = backend.get("hba_visualize") if isinstance(backend.get("hba_visualize"), dict) else {}
    return {
        "file_path": viz.get("file_path") if isinstance(viz.get("file_path"), str) else "/data/automap_output/hba_export",
        "downsample_size": _safe_float(viz.get("downsample_size"), 0.1),
        "pcd_name_fill_num": _safe_int(viz.get("pcd_name_fill_num"), 0),
        "marker_size": _safe_float(viz.get("marker_size"), 0.5),
    }


def _flatten_dict_unused(d, parent_key="", sep="."):
    """Unused: 保留以防需要扁平化。"""
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict) and not (v and isinstance(next(iter(v.values())), (list, dict)) and not any((isinstance(x, (list, dict)) for x in v.values()))):
            # 只压一层：允许 value 为 list（如 extrinsic_T）
            sub = {}
            for kk, vv in v.items():
                sub[f"{new_key}{sep}{kk}"] = vv
            items.extend(_flatten_dict_unused(sub, "", sep="").items() if len(sub) > 0 and not isinstance(list(v.values())[0], (list, dict)) else [(new_key, v)])
        elif isinstance(v, dict):
            items.extend(_flatten_dict_unused(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)


# fast_livo 声明为 string 的参数，不能为 not set / 空
_FAST_LIVO_STRING_KEYS = frozenset(("lid_topic", "imu_topic", "img_topic", "seq_name"))

def _sanitize_fast_livo_value(key, value):
    """None 时仅对已知 string 参数给默认值，避免 expected [string] got [not set]。"""
    if value is None:
        if key == "lid_topic":
            return "/livox/lidar"
        if key == "imu_topic":
            return "/livox/imu"
        if key == "img_topic":
            return "/left/image_raw"
        if key == "seq_name":
            return "default"
        return None
    if isinstance(value, str) and key in _FAST_LIVO_STRING_KEYS:
        v = value.strip()
        if not v:
            return "default" if key == "seq_name" else "/left/image_raw" if key == "img_topic" else "/livox/lidar" if key == "lid_topic" else "/livox/imu"
        return v
    return value


def _sanitize_fast_livo_params_recursive(obj):
    """
    递归清理：去掉空键；None 仅对已知 string 参数替换为默认，其余键省略（由节点默认值生效）。
    同时剔除仅空格的键名，避免 launch 序列化后产生 parameter ''。
    """
    if obj is None:
        return None
    if isinstance(obj, dict):
        out = {}
        for k, v in obj.items():
            if not isinstance(k, str) or not k.strip():
                continue
            v2 = _sanitize_fast_livo_params_recursive(v)
            if v2 is None:
                v2 = _sanitize_fast_livo_value(k, None)
                if v2 is None:
                    continue
            out[k] = v2
        return out
    if isinstance(obj, list):
        return [_sanitize_fast_livo_params_recursive(x) for x in obj]
    return obj


def get_fast_livo2_params(config):
    """
    从 system_config 的 sensor + frontend.fast_livo2（或顶层 fast_livo，如 M2DGR 配置）生成 fastlivo_mapping 节点参数。
    lid_topic / imu_topic 来自 sensor.lidar.topic 与 sensor.imu.topic；其余与 fast_livo 各段一一对应。
    必填字符串参数（common.img_topic、evo.seq_name 等）保证非空，避免节点 "expected [string] got [not set]"。
    同时生成 parameter_blackboard 命名空间的相机参数，供 vk::camera_loader 使用。
    兼容：优先 frontend.fast_livo2，若无则使用顶层 fast_livo（如 system_config_M2DGR.yaml）。
    """
    try:
        return _get_fast_livo2_params_impl(config)
    except Exception as e:
        _log_exception("get_fast_livo2_params", e, tag="EXCEPTION")
        _diag("返回空参数字典，fast_livo 将使用节点默认或启动失败", level="WARN", tag="CONFIG")
        return {"common": {"lid_topic": "/livox/lidar", "imu_topic": "/livox/imu", "img_topic": "/left/image_raw"},
                "parameter_blackboard": {"model": "Pinhole", "width": 752, "height": 480, "scale": 1.0,
                                         "fx": 425.0, "fy": 426.0, "cx": 386.0, "cy": 241.0,
                                         "d0": 0.0, "d1": 0.0, "d2": 0.0, "d3": 0.0},
                "evo": {"seq_name": "default", "pose_output_en": False}}


def _get_fast_livo2_params_impl(config):
    """get_fast_livo2_params 的实际实现，内部异常由外层捕获并记录。"""
    if not isinstance(config, dict):
        config = {}
    sensor = config.get("sensor") if isinstance(config.get("sensor"), dict) else {}
    frontend = config.get("frontend") if isinstance(config.get("frontend"), dict) else {}
    fl2 = frontend.get("fast_livo2") if isinstance(frontend.get("fast_livo2"), dict) else {}
    if not fl2 and isinstance(config.get("fast_livo"), dict):
        fl2 = config.get("fast_livo")

    lidar = sensor.get("lidar") if isinstance(sensor.get("lidar"), dict) else {}
    imu = sensor.get("imu") if isinstance(sensor.get("imu"), dict) else {}
    camera = sensor.get("camera") if isinstance(sensor.get("camera"), dict) else {}
    common_src = fl2.get("common") if isinstance(fl2.get("common"), dict) else {}
    common = dict(common_src)
    # 传感器话题统一从 sensor: 节读取（默认从配置文件）
    common["lid_topic"] = (lidar.get("topic") or common.get("lid_topic") or "/livox/lidar").strip() or "/livox/lidar"
    common["imu_topic"] = (imu.get("topic") or common.get("imu_topic") or "/livox/imu").strip() or "/livox/imu"
    img_from_sensor = (camera.get("topic") or "").strip()
    common["img_topic"] = img_from_sensor or (common.get("img_topic") or "/left/image_raw").strip() or "/left/image_raw"

    params = {"common": common}
    
    # -------------------------------------------------------------------------
    # 添加 parameter_blackboard 相机参数（供 vk::camera_loader::loadFromRosNs 使用）
    # vikit camera_loader.h 期望键名：model, width, height, scale, fx, fy, cx, cy, d0-d3（非 cam_*）
    # 优先 sensor.camera_left，否则用 fast_livo.camera_calib（如 M2DGR 配置）
    # -------------------------------------------------------------------------
    cam_left = sensor.get("camera_left") if isinstance(sensor.get("camera_left"), dict) else {}
    if not cam_left and isinstance(fl2.get("camera_calib"), dict):
        cam_left = fl2["camera_calib"]
    params["parameter_blackboard"] = {
        "model": "Pinhole",
        "width": _safe_int(cam_left.get("image_width") or cam_left.get("width"), 752),
        "height": _safe_int(cam_left.get("image_height") or cam_left.get("height"), 480),
        "scale": 1.0,
        "fx": _safe_float(cam_left.get("fx"), 425.0),
        "fy": _safe_float(cam_left.get("fy"), 426.0),
        "cx": _safe_float(cam_left.get("cx"), 386.0),
        "cy": _safe_float(cam_left.get("cy"), 241.0),
        "d0": _safe_float(cam_left.get("k1"), 0.0),
        "d1": _safe_float(cam_left.get("k2"), 0.0),
        "d2": _safe_float(cam_left.get("p1"), 0.0),
        "d3": _safe_float(cam_left.get("p2"), 0.0),
    }
    for section in ("extrin_calib", "time_offset", "preprocess", "vio", "imu", "lio",
                    "local_map", "uav", "publish", "evo", "pcd_save", "image_save"):
        if section not in fl2:
            continue
        val = fl2[section]
        if isinstance(val, dict):
            params[section] = dict(val)
        else:
            params[section] = val

    # 前端 PCD 保存目录与 system.output_dir 统一，始终覆盖为 system.output_dir，保证与后端 automap_output 同目录
    system = config.get("system") if isinstance(config.get("system"), dict) else {}
    out_dir = (system.get("output_dir") or "").strip() or "/data/automap_output"
    if "pcd_save" not in params:
        params["pcd_save"] = {}
    if isinstance(params["pcd_save"], dict):
        params["pcd_save"]["output_dir"] = out_dir

    # 确保 evo.seq_name 等字符串参数非空（fast_livo 声明为 string，不能为 not set）
    if "evo" in params:
        evo = params["evo"]
        if isinstance(evo, dict):
            params["evo"] = dict(evo)
            if not (params["evo"].get("seq_name") or "").strip():
                params["evo"]["seq_name"] = "default"
    else:
        params["evo"] = {"seq_name": "default", "pose_output_en": False}

    # 递归清理：去除 None、空键，保证 string 参数非空，避免 parameter '' / not set
    params = _sanitize_fast_livo_params_recursive(params)
    # 再次保证必填字符串（清理可能把 None 变成 0 的边界情况）
    params.setdefault("common", {})
    params["common"]["lid_topic"] = (params["common"].get("lid_topic") or "/livox/lidar").strip() or "/livox/lidar"
    params["common"]["imu_topic"] = (params["common"].get("imu_topic") or "/livox/imu").strip() or "/livox/imu"
    params["common"]["img_topic"] = (params["common"].get("img_topic") or "/left/image_raw").strip() or "/left/image_raw"
    if "evo" in params and isinstance(params["evo"], dict):
        params["evo"]["seq_name"] = (params["evo"].get("seq_name") or "default").strip() or "default"

    # 最终兜底：递归移除空键和空字符串值，避免 parameter '' expected [string] got [not set]
    def _drop_empty_keys(obj):
        if isinstance(obj, dict):
            out = {}
            for k, v in obj.items():
                if not isinstance(k, str) or not k.strip():
                    continue
                v2 = _drop_empty_keys(v)
                # 跳过空字符串值，否则 fast_livo2 报 expected [string] got [not set]
                if isinstance(v2, str) and not v2.strip():
                    continue
                out[k] = v2
            return out
        if isinstance(obj, list):
            return [_drop_empty_keys(x) for x in obj]
        return obj
    params = _drop_empty_keys(params)

    # ── 诊断：parameter_blackboard 必须含 model（camera_loader 第一项检查），便于精准排查 Camera model not specified
    pb = params.get("parameter_blackboard") if isinstance(params.get("parameter_blackboard"), dict) else {}
    _diag("get_fast_livo2_params 返回: parameter_blackboard 键列表={}".format(sorted(pb.keys())), tag="KEYS")
    _diag("parameter_blackboard.model={!r} (camera_loader 需非空)".format(pb.get("model")), tag="KEYS")
    if not (pb.get("model") or "").strip():
        _diag("parameter_blackboard.model 为空，fast_livo 将报 Camera model not correctly specified", level="WARN", tag="CONFIG")

    return params


def _strip_empty_and_non_string_keys(obj):
    """
    递归移除所有「非字符串」或「空/仅空白」的键，避免 ROS2 解析时产生 parameter ''。
    automatically_declare_parameters_from_overrides(true) 会把文件里每个 key 都声明为参数，
    若存在空键会触发 parameter '' has invalid type: expected [string] got [not set]。
    """
    if obj is None:
        return None
    if isinstance(obj, dict):
        out = {}
        for k, v in obj.items():
            if not isinstance(k, str) or not k.strip():
                continue
            v2 = _strip_empty_and_non_string_keys(v)
            if v2 is not None:
                out[k] = v2
        return out
    if isinstance(obj, list):
        return [_strip_empty_and_non_string_keys(x) for x in obj]
    return obj


def _collect_param_paths(obj, prefix=""):
    """递归收集所有参数路径（如 common.lid_topic），用于诊断与校验。"""
    paths = []
    if obj is None:
        return paths
    if isinstance(obj, dict):
        for k, v in obj.items():
            seg = (prefix + "." + k) if prefix else k
            if isinstance(v, dict) and v:
                paths.extend(_collect_param_paths(v, seg))
            elif isinstance(v, list):
                paths.append(seg)  # ROS2 中列表整体为一个参数
            else:
                paths.append(seg)
        return paths
    if isinstance(obj, list):
        paths.append(prefix)
        return paths
    return paths


def _check_empty_key_segments(obj, path=""):
    """
    检查参数树中是否存在空或非字符串键，返回 (是否有问题, 问题键路径列表)。
    """
    bad = []
    if obj is None:
        return False, bad
    if isinstance(obj, dict):
        for k, v in obj.items():
            seg = path + ("." + str(k) if path else str(k))
            if not isinstance(k, str):
                bad.append("{} (type={})".format(seg, type(k).__name__))
            elif not k.strip():
                bad.append("{} (empty key)".format(seg))
            sub_ok, sub_bad = _check_empty_key_segments(v, seg)
            bad.extend(sub_bad)
        return len(bad) == 0, bad
    if isinstance(obj, list):
        for i, x in enumerate(obj):
            sub_ok, sub_bad = _check_empty_key_segments(x, "{}[{}]".format(path, i))
            bad.extend(sub_bad)
        return len(bad) == 0, bad
    return True, bad


def _flatten_params_for_ros2(obj, prefix=""):
    """
    将嵌套 dict 压平为单层 key-value，key 为 "section.sub.param"。
    避免 rclcpp 在解析嵌套 YAML 时递归扁平化产生 parameter '' 的 bug。
    仅接受字符串且非空的 key，跳过空键/非字符串键。
    """
    out = {}
    if obj is None:
        return out
    if isinstance(obj, dict):
        for k, v in obj.items():
            if not isinstance(k, str) or not k.strip():
                continue
            full = (prefix + "." + k) if prefix else k
            if isinstance(v, dict) and v:
                out.update(_flatten_params_for_ros2(v, full))
            elif isinstance(v, list):
                out[full] = v
            else:
                out[full] = v
        return out
    return out


def write_fast_livo_params_file(config, output_path, config_path=None):
    """
    将 frontend.fast_livo2 参数写入 YAML 文件，供 fastlivo_mapping 节点通过 parameters=[path] 加载。
    使用「扁平化」格式（ros__parameters 内单层 key 如 common.lid_topic），避免 rclcpp 解析嵌套结构时
    产生 parameter '' has invalid type: expected [string] got [not set]。
    config_path: 可选，用于日志中记录来源配置路径，便于定位问题。
    """
    out_abs = os.path.abspath(output_path)
    out_dir = os.path.dirname(output_path) or "."
    out_dir_abs = os.path.abspath(out_dir)
    config_abs = os.path.abspath(config_path) if config_path else None
    config_real = os.path.realpath(config_path) if config_path and os.path.exists(config_path) else None
    out_dir_real = os.path.realpath(out_dir) if os.path.exists(out_dir) else None

    _diag("=== fast_livo params 写入开始 ===", tag="CONFIG")
    # ----- 所有路径汇总（便于精准定位：配置来源、输出位置、节点将读取的文件） -----
    _diag("--- PATHS (all absolute/real for precise location) ---", tag="PATHS")
    _diag("config_path(raw)={}".format(config_path if config_path else "(未传入)"), tag="PATHS")
    _diag("config_path(absolute)={}".format(config_abs), tag="PATHS")
    _diag("config_path(realpath)={}".format(config_real), tag="PATHS")
    _diag("config_exists={}".format(os.path.isfile(config_path) if config_path else False), tag="PATHS")
    _diag("output_path(raw)={}".format(output_path), tag="PATHS")
    _diag("output_path(absolute)={}".format(out_abs), tag="PATHS")
    _diag("output_dir(absolute)={}".format(out_dir_abs), tag="PATHS")
    _diag("output_dir(realpath)={}".format(out_dir_real), tag="PATHS")
    _diag("output_dir_exists={}".format(os.path.isdir(out_dir)), tag="PATHS")
    _diag("params_file_for_fast_livo(use_this_in_grep)={}".format(out_abs), tag="PATHS")

    # 全工程唯一配置：支持 frontend.fast_livo2 或顶层 fast_livo（如 system_config_M2DGR.yaml），二者等价
    if not isinstance(config, dict) or not config:
        _diag("config 为空或非 dict，fast_livo 将使用默认参数", level="WARN", tag="CONFIG")
    elif isinstance(config.get("fast_livo"), dict):
        _diag("使用顶层 fast_livo 作为唯一配置来源（与 frontend.fast_livo2 等价）", tag="CONFIG")
    elif not (isinstance(config.get("frontend"), dict) and isinstance(config.get("frontend", {}).get("fast_livo2"), dict)):
        _diag("config 缺少 frontend.fast_livo2 与顶层 fast_livo，部分 fast_livo 参数将使用默认值", level="WARN", tag="CONFIG")
    try:
        params = get_fast_livo2_params(config)
    except Exception as e:
        _log_exception("write_fast_livo_params_file(get_fast_livo2_params)", e, tag="WRITE")
        params = {}
    _diag("get_fast_livo2_params 返回 top-level 键: {}".format(list(params.keys()) if isinstance(params, dict) else type(params).__name__), tag="CONFIG")

    # 写入前再次剔除任意层级的空键/非字符串键
    params = _strip_empty_and_non_string_keys(params)
    if params is None:
        params = {}
    _diag("strip 后 top-level 键数: {}".format(len(params)), tag="PRE-WRITE")

    # ----- 写入前校验：是否存在空/非字符串键 -----
    ok, bad_keys = _check_empty_key_segments(params)
    if not ok:
        _diag("PRE-WRITE CHECK FAILED: invalid keys (will be stripped): {}".format(bad_keys), level="ERROR", tag="PRE-WRITE")
    else:
        _diag("PRE-WRITE CHECK OK: no empty/non-string keys in params dict", tag="PRE-WRITE")

    # 扁平化为单层 key（common.lid_topic 等），避免 C++ 端递归解析产生空参数名
    flat = _flatten_params_for_ros2(params)
    # 再次过滤：扁平后键必须非空；同时检测并记录空键
    empty_keys_found = [k for k in flat if not (isinstance(k, str) and k.strip())]
    if empty_keys_found:
        _diag("FLATTEN 发现空/非字符串键(将被丢弃): repr={}".format([repr(k) for k in empty_keys_found]), level="ERROR", tag="FLATTEN")
    flat = {k: v for k, v in flat.items() if isinstance(k, str) and k.strip()}

    _diag("nested_sections={} -> flat_param_count={}".format(len(params), len(flat)), tag="FLATTEN")
    bad_flat = [k for k in flat if not k.strip() or k.startswith(".") or k.endswith(".") or "" in k.split(".")]
    if bad_flat:
        _diag("FLATTEN invalid keys (excluded): {}".format(bad_flat), level="ERROR", tag="FLATTEN")

    # ----- 完整键列表（便于 grep 排查是否有空名或异常键） -----
    all_keys = sorted(flat.keys())
    _diag("flat_param_count={}".format(len(all_keys)), tag="KEYS")
    _diag("flat_keys_full_list={}".format(all_keys), tag="KEYS")
    # 检查是否存在名为空字符串的键（不应出现）
    if "" in all_keys:
        _diag("CRITICAL: empty string key '' found in flat keys", level="ERROR", tag="KEYS")
    for i, k in enumerate(all_keys):
        if not k or not isinstance(k, str):
            _diag("CRITICAL: key at index {} invalid: repr={}".format(i, repr(k)), level="ERROR", tag="KEYS")

    # ----- 关键 string 参数取值（fast_livo 必填，便于确认未丢或为空） -----
    critical = ["common.lid_topic", "common.imu_topic", "common.img_topic", "evo.seq_name"]
    for c in critical:
        val = flat.get(c)
        if val is None:
            _diag("CRITICAL_PARAM missing: {} -> node may get 'not set'".format(c), level="WARN", tag="CRITICAL_PARAMS")
        elif isinstance(val, str) and not val.strip():
            _diag("CRITICAL_PARAM empty string: {} -> repr={}".format(c, repr(val)), level="WARN", tag="CRITICAL_PARAMS")
        else:
            _diag("{}={}".format(c, repr(val)[:80]), tag="CRITICAL_PARAMS")

    # ROS2 params-file：节点名（无前导 /） + ros__parameters，避免部分 rcl 解析 /laserMapping 时产生空键 parameter ''
    content = {"laserMapping": {"ros__parameters": flat}}
    try:
        os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as f:
            yaml.dump(content, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    except Exception as e:
        _log_exception("write_fast_livo_params_file(写入文件)", e, tag="WRITE")
        raise
    out_real = os.path.realpath(output_path) if os.path.exists(output_path) else None
    _diag("file written: {} (size_bytes={})".format(out_abs, os.path.getsize(output_path)), tag="WRITE")
    _diag("output_path(realpath_after_write)={}".format(out_real), tag="PATHS")

    # ----- 写入后校验：读回文件检查是否出现空键；并扫描原始行精准定位空键所在行 -----
    try:
        with open(output_path, "r", encoding="utf-8") as f:
            raw_lines = f.readlines()
        _diag("file_line_count={}".format(len(raw_lines)), tag="FILE_PREVIEW")
        _diag("params_file_absolute_path={}".format(out_abs), tag="FILE_PREVIEW")
        preview = "".join(raw_lines[:15]).rstrip()
        _diag("file_preview_first_15_lines:\n{}".format(preview), tag="FILE_PREVIEW")

        # 扫描原始文件：可能引发 parameter '' 的 YAML 行（空键、裸冒号等）
        suspicious_lines = []
        for i, line in enumerate(raw_lines, 1):
            s = line.rstrip()
            if re.match(r"^\s*:\s*$", s) or re.match(r"^\s*'':\s*", s) or re.match(r'^\s*"":\s*', s):
                suspicious_lines.append((i, s))
        if suspicious_lines:
            _diag("EMPTY_KEY_SCAN: 以下行可能引发 parameter '' (文件={} 行号:内容)".format(out_abs), level="ERROR", tag="POST-WRITE")
            for ln, content in suspicious_lines:
                _diag("  line {}: {}".format(ln, repr(content)), level="ERROR", tag="POST-WRITE")
        else:
            _diag("EMPTY_KEY_SCAN: 未发现可疑空键行 (文件={})".format(out_abs), tag="POST-WRITE")

        with open(output_path, "r", encoding="utf-8") as f:
            read_back = yaml.safe_load(f)
        node_params = read_back.get("laserMapping", read_back.get("/laserMapping", {})).get("ros__parameters", {})
        bad2 = [k for k in node_params if not isinstance(k, str) or not k.strip()]
        if bad2:
            _diag("POST-WRITE CHECK FAILED: file contains invalid keys: {}".format(bad2), level="ERROR", tag="POST-WRITE")
        else:
            _diag("POST-WRITE CHECK OK: re-read param_count={}, no empty keys".format(len(node_params)), tag="POST-WRITE")
        # 读回后的键列表（用于确认与写入一致）
        read_keys = sorted(node_params.keys())
        _diag("read_back_keys_sample={}".format(read_keys[:10]), tag="POST-WRITE")
        if "" in read_keys:
            _diag("CRITICAL: empty key '' in read_back", level="ERROR", tag="POST-WRITE")
    except Exception as e:
        _diag("POST-WRITE CHECK failed: {}".format(e), level="ERROR", tag="POST-WRITE")
        import traceback
        _diag("traceback: {}".format(traceback.format_exc()), tag="POST-WRITE")

    # ----- 汇总行：便于一眼判断状态与排查命令 -----
    status = "OK" if not (bad_flat or empty_keys_found or ("" in all_keys)) else "CHECK_LOGS"
    _diag("=== SUMMARY: path={} format=flattened param_count={} status={} ===".format(out_abs, len(flat), status), tag="SUMMARY")
    _diag("SUMMARY paths: config_abs={} output_abs={} output_real={}".format(config_abs, out_abs, out_real), tag="SUMMARY")
    _diag("parameter '' 修复: fast_livo main.cpp 需设 automatically_declare_parameters_from_overrides(false)，并重新编译 fast_livo 包", tag="SUMMARY")
    _diag("若仍报 parameter ''，容器内执行: grep -nE \"^[[:space:]]*:[^ ]|'':\" {} 检查空键；并确认已 colcon build fast_livo".format(out_abs), tag="SUMMARY")

    return output_path


def write_hba_params_file(config, output_path):
    """
    将 backend.hba 参数写入 YAML 文件，供 ros2 run hba hba --params-file 使用。
    返回写入的路径。
    """
    params = get_hba_params(config)
    # ROS2 节点期望的格式：/** 下 ros__parameters
    content = {"/**": {"ros__parameters": params}}
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        yaml.dump(content, f, default_flow_style=False, allow_unicode=True)
    return output_path
