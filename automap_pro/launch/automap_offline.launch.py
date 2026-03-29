#!/usr/bin/env python3
# AutoMap-Pro Offline / Bag Replay Launch (ROS2)
# 全工程唯一配置：仅使用 config:= 传入的 YAML，不生成 fast_livo_params.yaml；fast-livo2 / HBA / automap_system 均从此 config 读参

import os
import sys
import subprocess
import traceback
import glob
import tempfile
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
            _orig_config_abs = os.path.abspath(config_path)
            with open(config_path, "r", encoding="utf-8") as _f:
                _cfg = _yaml.safe_load(_f) or {}
            # 离线补丁会把 YAML 写到 /tmp；ConfigManager 用「配置文件路径 dirname×2」展开
            # ${CMAKE_CURRENT_SOURCE_DIR}，对 /tmp/automap_offline_*.yaml 会得到错误根目录（如 CWD/models）。
            # 在写入补丁前将 overlap_transformer.model_path 规范为绝对路径（相对 .../automap_pro/config/*.yaml）。
            try:
                _lc = _cfg.get("loop_closure")
                if isinstance(_lc, dict):
                    _ot = _lc.get("overlap_transformer")
                    if isinstance(_ot, dict):
                        _mp = (_ot.get("model_path") or "").strip()
                        if _mp:
                            _pkg_root = os.path.dirname(os.path.dirname(_orig_config_abs))
                            _cmake = "${CMAKE_CURRENT_SOURCE_DIR}"
                            if _cmake in _mp:
                                _mp = _mp.replace(_cmake, _pkg_root)
                            elif not os.path.isabs(_mp):
                                _mp = os.path.abspath(
                                    os.path.join(os.path.dirname(_orig_config_abs), _mp)
                                )
                            _mp = os.path.normpath(_mp)
                            _ot["model_path"] = _mp
                            _lc["overlap_transformer"] = _ot
                            _cfg["loop_closure"] = _lc
                            sys.stderr.write(
                                "{} [OVERLAP] offline resolved model_path={}\n".format(_LP, _mp)
                            )
                            sys.stderr.flush()
            except Exception as _e_res:
                _log_launch_exception("resolve loop_closure.overlap_transformer.model_path (offline)", _e_res)
            # 语义模型路径离线解析：避免 /tmp 补丁 YAML 导致 ${CMAKE_CURRENT_SOURCE_DIR} 失效。
            # 按 semantic.model_type 做分支解析，并在 LSK3DNet 资产不完整时做最小自愈。
            try:
                _pkg_root = os.path.dirname(os.path.dirname(_orig_config_abs))
                _sm = _cfg.get("semantic")
                if isinstance(_sm, dict) and bool(_sm.get("enabled", True)):
                    _model_type = (_sm.get("model_type") or "sloam").strip()
                    _cmake = "${CMAKE_CURRENT_SOURCE_DIR}"
                    _lsk = _sm.get("lsk3dnet") if isinstance(_sm.get("lsk3dnet"), dict) else {}

                    def _resolve_path(_p, _base_root=None):
                        _p = (_p or "").strip()
                        if not _p:
                            return ""
                        if _cmake in _p:
                            _p = _p.replace(_cmake, _pkg_root)
                        if os.path.isabs(_p):
                            return os.path.normpath(_p)
                        if _base_root:
                            return os.path.normpath(os.path.join(_base_root, _p))
                        return os.path.normpath(os.path.abspath(os.path.join(os.path.dirname(_orig_config_abs), _p)))

                    def _first_existing(_cands):
                        for _c in _cands:
                            if _c and os.path.isfile(_c):
                                return _c
                        return ""

                    if _model_type == "sloam":
                        _sem_mp = _resolve_path(_sm.get("model_path"))
                        if not _sem_mp or (not os.path.isfile(_sem_mp)):
                            _sem_mp = _first_existing([
                                os.path.join(_pkg_root, "models", "squeezesegV2_segmentator.onnx"),
                                os.path.join(_pkg_root, "models", "squeezesegV2-crf_segmentator.onnx"),
                                os.path.join(_pkg_root, "models", "squeezeseg_segmentator.onnx"),
                                os.path.join(_pkg_root, "models", "squeezeseg-crf_segmentator.onnx"),
                                os.path.join(_pkg_root, "models", "darknet53-1024_segmentator.onnx"),
                                "/opt/sloam_ws/src/models/darknet53-1024_segmentator.onnx",
                            ])
                        if not _sem_mp:
                            raise RuntimeError(
                                "semantic.model_type=sloam but semantic.model_path is empty or file not found"
                            )
                        _sm["model_path"] = _sem_mp
                        sys.stderr.write("{} [SEMANTIC] offline resolved sloam model_path={}\n".format(_LP, _sem_mp))
                        sys.stderr.flush()
                    else:
                        # LSK3DNet path resolution (lsk3dnet / lsk3dnet_hybrid)
                        _repo_root = _resolve_path(_lsk.get("repo_root"))
                        if not _repo_root:
                            _repo_cand = os.path.join(_pkg_root, "thrid_party", "LSK3DNet-main")
                            if os.path.isdir(_repo_cand):
                                _repo_root = _repo_cand

                        _lsk_ts = _resolve_path(_lsk.get("model_path"))
                        _cfg_yaml = _resolve_path(_lsk.get("config_yaml"), _repo_root if _repo_root else None)
                        _ckpt = _resolve_path(_lsk.get("checkpoint"), _repo_root if _repo_root else None)
                        _classifier_ts = _resolve_path(_lsk.get("classifier_torchscript"), _repo_root if _repo_root else None)
                        _worker_script = _resolve_path(_lsk.get("worker_script"), _repo_root if _repo_root else None)

                        if not _cfg_yaml and _repo_root:
                            _cfg_yaml = _first_existing([
                                os.path.join(_repo_root, "config", "lk-semantickitti_erk_finetune.yaml"),
                                os.path.join(_repo_root, "config", "lk-semantickitti_sub_tta.yaml"),
                            ])
                        if not _ckpt and _repo_root:
                            _ckpt = _first_existing([
                                os.path.join(_repo_root, "output_skitti", "opensource_9ks_s030_w64_0.pt"),
                            ])
                        if (not _ckpt) and _repo_root and os.path.isdir(_repo_root):
                            _pt_cands = sorted(glob.glob(os.path.join(_repo_root, "**", "*.pt"), recursive=True))
                            if _pt_cands:
                                _ckpt = _pt_cands[0]
                        if not _classifier_ts:
                            _classifier_ts = _first_existing([
                                os.path.join(_pkg_root, "models", "lsk_classifier.ts"),
                                os.path.join(_pkg_root, "models", "lsk3dnet_classifier.ts"),
                                os.path.join(_repo_root, "output_skitti", "lsk_classifier.ts") if _repo_root else "",
                            ])
                        if (not _classifier_ts) and _repo_root and os.path.isdir(_repo_root):
                            _cls_cands = sorted(glob.glob(os.path.join(_repo_root, "**", "*classifier*.ts"), recursive=True))
                            if _cls_cands:
                                _classifier_ts = _cls_cands[0]
                        if not _worker_script and _repo_root:
                            _worker_script = os.path.join(_repo_root, "scripts", "lsk3dnet_hybrid_worker.py")

                        # If lsk3dnet.ts missing but hybrid assets are present, auto switch to hybrid.
                        _has_lsk_ts = bool(_lsk_ts and os.path.isfile(_lsk_ts))
                        _hybrid_ready = bool(
                            _repo_root and os.path.isdir(_repo_root) and
                            _cfg_yaml and os.path.isfile(_cfg_yaml) and
                            _ckpt and os.path.isfile(_ckpt) and
                            _classifier_ts and os.path.isfile(_classifier_ts) and
                            _worker_script and os.path.isfile(_worker_script)
                        )
                        if _model_type == "lsk3dnet" and (not _has_lsk_ts) and _hybrid_ready:
                            _model_type = "lsk3dnet_hybrid"
                            sys.stderr.write("{} [SEMANTIC] lsk3dnet.ts missing, auto-switch model_type -> lsk3dnet_hybrid\n".format(_LP))
                            sys.stderr.flush()

                        if _model_type == "lsk3dnet":
                            if not _has_lsk_ts:
                                raise RuntimeError(
                                    "semantic.model_type=lsk3dnet but semantic.lsk3dnet.model_path not found: {}".format(_lsk_ts)
                                )
                        elif _model_type == "lsk3dnet_hybrid":
                            _missing = []
                            if not (_repo_root and os.path.isdir(_repo_root)):
                                _missing.append("repo_root")
                            if not (_cfg_yaml and os.path.isfile(_cfg_yaml)):
                                _missing.append("config_yaml")
                            if not (_ckpt and os.path.isfile(_ckpt)):
                                _missing.append("checkpoint(.pt)")
                            if not (_classifier_ts and os.path.isfile(_classifier_ts)):
                                _missing.append("classifier_torchscript(.ts)")
                            if not (_worker_script and os.path.isfile(_worker_script)):
                                _missing.append("worker_script")
                            if _missing:
                                raise RuntimeError(
                                    "semantic.model_type=lsk3dnet_hybrid missing assets: {}".format(", ".join(_missing))
                                )
                        else:
                            raise RuntimeError("unsupported semantic.model_type: {}".format(_model_type))

                        _sm["model_type"] = _model_type
                        if _repo_root:
                            _lsk["repo_root"] = _repo_root
                        if _lsk_ts:
                            _lsk["model_path"] = _lsk_ts
                        if _cfg_yaml:
                            _lsk["config_yaml"] = _cfg_yaml
                        if _ckpt:
                            _lsk["checkpoint"] = _ckpt
                        if _classifier_ts:
                            _lsk["classifier_torchscript"] = _classifier_ts
                        if _worker_script:
                            _lsk["worker_script"] = _worker_script
                        _sm["lsk3dnet"] = _lsk
                        sys.stderr.write(
                            "{} [SEMANTIC] offline resolved {} assets: ts={} ckpt={} cls_ts={} repo={}\n".format(
                                _LP, _model_type, _lsk.get("model_path", ""), _lsk.get("checkpoint", ""),
                                _lsk.get("classifier_torchscript", ""), _lsk.get("repo_root", ""))
                        )
                        sys.stderr.flush()

                    _cfg["semantic"] = _sm
            except Exception as _e_sem:
                # 语义资产缺失时不应中断离线会话补丁写入；降级后继续启动主流程。
                _log_launch_exception("resolve semantic.model_path (offline)", _e_sem)
                try:
                    _sm = _cfg.get("semantic") if isinstance(_cfg.get("semantic"), dict) else {}
                    _pkg_root = os.path.dirname(os.path.dirname(_orig_config_abs))
                    _fallback_sloam = _first_existing([
                        os.path.join(_pkg_root, "models", "squeezesegV2_segmentator.onnx"),
                        os.path.join(_pkg_root, "models", "squeezesegV2-crf_segmentator.onnx"),
                        os.path.join(_pkg_root, "models", "squeezeseg_segmentator.onnx"),
                        os.path.join(_pkg_root, "models", "squeezeseg-crf_segmentator.onnx"),
                        os.path.join(_pkg_root, "models", "darknet53-1024_segmentator.onnx"),
                        "/opt/sloam_ws/src/models/darknet53-1024_segmentator.onnx",
                    ])
                    if _fallback_sloam:
                        _sm["enabled"] = True
                        _sm["model_type"] = "sloam"
                        _sm["model_path"] = os.path.normpath(_fallback_sloam)
                        _cfg["semantic"] = _sm
                        sys.stderr.write(
                            "{} [SEMANTIC][FALLBACK] resolve failed, downgrade to sloam model_path={}\n".format(
                                _LP, _sm["model_path"])
                        )
                    else:
                        _sm["enabled"] = False
                        _cfg["semantic"] = _sm
                        sys.stderr.write(
                            "{} [SEMANTIC][FALLBACK] resolve failed and no sloam model found, disable semantic for this run\n".format(
                                _LP)
                        )
                    sys.stderr.flush()
                except Exception as _e_sem_fb:
                    _log_launch_exception("semantic fallback (offline)", _e_sem_fb)
            # Normalize section types before patching: YAML can contain `system: null`,
            # and dict.setdefault() will NOT override an existing None value.
            _system = _cfg.get("system") if isinstance(_cfg.get("system"), dict) else {}
            _base_out = (_system.get("output_dir") or "").strip() or "/data/automap_output"
            _base_out = os.path.abspath(_base_out.rstrip("/"))
            if "/run_" in _base_out:
                session_out = _base_out
            else:
                session_out = _base_out + "/run_" + datetime.now().strftime("%Y%m%d_%H%M%S")
            os.makedirs(session_out, exist_ok=True)
            os.environ["AUTOMAP_SESSION_OUTPUT_DIR"] = session_out

            # Contract flags are needed for config patching/hardening.
            _contract = _cfg.get("contract") if isinstance(_cfg.get("contract"), dict) else {}
            _strict_mode = str(_contract.get("strict_mode", False)).strip().lower() in ("1", "true", "yes", "on")

            # Hardening: ensure `system` is a dict (YAML may contain `system: null`).
            # If it's not a dict, attempt to recover it from the original config before falling back to {}.
            if not isinstance(_cfg.get("system"), dict):
                try:
                    with open(_orig_config_abs, "r", encoding="utf-8") as _f_orig:
                        _orig_cfg_reload = _yaml.safe_load(_f_orig) or {}
                    if isinstance(_orig_cfg_reload, dict) and isinstance(_orig_cfg_reload.get("system"), dict):
                        _cfg["system"] = dict(_orig_cfg_reload.get("system") or {})
                    else:
                        _cfg["system"] = {}
                except Exception:
                    _cfg["system"] = {}

            # Ensure the protocol-required key is present when strict_mode is enabled.
            # (The C++ side enforces this in strict_mode; keep launch behavior consistent.)
            if _strict_mode and not str(_cfg.get("system", {}).get("api_version", "")).strip():
                # Default to the current Automap-Pro protocol version used in shipped configs.
                _cfg["system"]["api_version"] = "3.1"
                sys.stderr.write(
                    "{} [CONFIG][WARN] strict_mode=true but system.api_version missing; defaulting to '3.1' for offline patched config\n".format(_LP)
                )
                sys.stderr.flush()

            _cfg["system"]["output_dir"] = session_out
            if not isinstance(_cfg.get("map"), dict):
                _cfg["map"] = {}
            _cfg["map"]["frame_config_path"] = os.path.join(session_out, "map_frame.cfg")

            # 钉死顶层 `gps:`：与原始 YAML 合并缺失键，避免 safe_dump/中间步骤导致补丁里丢杆臂，
            # C++ 侧 cfg_["gps"] 异常与 HBA 零杆臂（见 run_20260329_091053 + ConfigManager 诊断）。
            try:
                _orig_full_for_gps = {}
                with open(_orig_config_abs, "r", encoding="utf-8") as _ogps:
                    _orig_full_for_gps = _yaml.safe_load(_ogps) or {}
                _src_gps = _orig_full_for_gps.get("gps") if isinstance(_orig_full_for_gps.get("gps"), dict) else {}
                if not isinstance(_cfg.get("gps"), dict):
                    _cfg["gps"] = {}
                _merged_gps = dict(_cfg["gps"])
                for _gk, _gv in _src_gps.items():
                    if _gk not in _merged_gps or _merged_gps[_gk] is None:
                        _merged_gps[_gk] = _gv
                _cfg["gps"] = _merged_gps
                if _src_gps:
                    sys.stderr.write(
                        "{} [CONFIG][GPS] merged top-level gps: keys from source={}, keys in patched={}\n".format(
                            _LP, len(_src_gps), len(_merged_gps))
                    )
                    sys.stderr.flush()
            except Exception as _e_merge_gps:
                _log_launch_exception("merge top-level gps from source config (offline)", _e_merge_gps)

            # Runtime hard guard (from run_automap precheck):
            # If NVRTC arch support is insufficient, force OT to CPU to avoid process aborts.
            _force_ot_cpu = str(os.environ.get("AUTOMAP_FORCE_OT_CPU", "")).strip().lower() in ("1", "true", "yes", "on")
            if _force_ot_cpu:
                _reason = str(os.environ.get("AUTOMAP_FORCE_OT_CPU_REASON", "")).strip()
                if not isinstance(_cfg.get("loop_closure"), dict):
                    _cfg["loop_closure"] = {}
                if not isinstance(_cfg["loop_closure"].get("overlap_transformer"), dict):
                    _cfg["loop_closure"]["overlap_transformer"] = {}
                _cfg["loop_closure"]["overlap_transformer"]["use_cuda"] = False
                _red = "\033[1;31m"
                _rst = "\033[0m"
                sys.stderr.write(
                    "{} {}[CUDA_BLOCK][OT] NVRTC arch precheck failed -> force CPU OT (loop_closure.overlap_transformer.use_cuda=false){} reason={}\n".format(
                        _LP, _red, _rst, _reason if _reason else "<unspecified>"
                    )
                )
                sys.stderr.flush()

            # Preflight protocol contract: strict_mode requires system.api_version.
            _api_version = str(_cfg.get("system", {}).get("api_version", "")).strip() if isinstance(_cfg.get("system"), dict) else ""
            if _strict_mode and not _api_version:
                raise RuntimeError(
                    "offline patched config invalid: contract.strict_mode=true but missing required key system.api_version "
                    "(source_config={})".format(_orig_config_abs)
                )

            # Write patched config into the session output dir (mounted to host in offline mode),
            # so we can always inspect it after a crash (container /tmp is ephemeral and not visible on host).
            _patched_config = os.path.join(session_out, "system_config_offline_patched.yaml")
            # Atomic write: avoid readers observing partial YAML (bind-mounts/overlay can expose torn writes).
            _tmp_fd = None
            _tmp_path = None
            try:
                _tmp_fd, _tmp_path = tempfile.mkstemp(
                    prefix="system_config_offline_patched.",
                    suffix=".yaml.tmp",
                    dir=session_out,
                )
                with os.fdopen(_tmp_fd, "w", encoding="utf-8") as _f:
                    _yaml.safe_dump(_cfg, _f, default_flow_style=False, allow_unicode=True, sort_keys=False)
                    _f.flush()
                    os.fsync(_f.fileno())
                _tmp_fd = None
                os.replace(_tmp_path, _patched_config)
                _tmp_path = None
                # Ensure host user can inspect the file (mkstemp defaults to 0600).
                try:
                    os.chmod(_patched_config, 0o644)
                except Exception:
                    pass
            finally:
                try:
                    if _tmp_fd is not None:
                        os.close(_tmp_fd)
                except Exception:
                    pass
                try:
                    if _tmp_path and os.path.exists(_tmp_path):
                        os.unlink(_tmp_path)
                except Exception:
                    pass

            # Post-write verification: re-load from disk to ensure YAML serialization didn't drop keys.
            _verify_cfg = {}
            try:
                with open(_patched_config, "r", encoding="utf-8") as _f:
                    _verify_cfg = _yaml.safe_load(_f) or {}
            except Exception as _e_verify:
                raise RuntimeError("failed to re-load patched config from disk: {}".format(_patched_config)) from _e_verify

            _verify_contract = _verify_cfg.get("contract") if isinstance(_verify_cfg.get("contract"), dict) else {}
            _verify_strict = str(_verify_contract.get("strict_mode", False)).strip().lower() in ("1", "true", "yes", "on")
            _verify_api = ""
            if isinstance(_verify_cfg.get("system"), dict):
                _verify_api = str(_verify_cfg.get("system", {}).get("api_version", "")).strip()
            if _verify_strict and not _verify_api:
                # Print a short preview for debugging before failing fast.
                try:
                    _raw = _yaml.safe_dump(_verify_cfg, default_flow_style=False, allow_unicode=True, sort_keys=False)
                    _lines = _raw.splitlines()
                    _preview = "\n".join(_lines[:60]) + ("\n... (truncated)" if len(_lines) > 60 else "")
                    sys.stderr.write("{} [CONFIG][FATAL] patched YAML missing system.api_version after write. Preview:\n{}\n".format(_LP, _preview))
                    sys.stderr.flush()
                except Exception:
                    pass
                raise RuntimeError("offline patched config invalid after write: strict_mode=true but missing system.api_version (file={})".format(_patched_config))

            # sensor.gps.enabled 时补丁必须含 gps.lever_arm_imu（长度>=3），否则 HBA/GPS 几何与主配置分裂。
            _verify_sensor = _verify_cfg.get("sensor") if isinstance(_verify_cfg.get("sensor"), dict) else {}
            _verify_sg = _verify_sensor.get("gps") if isinstance(_verify_sensor.get("gps"), dict) else {}
            _verify_gps_on = str(_verify_sg.get("enabled", False)).strip().lower() in ("1", "true", "yes", "on")
            if _verify_gps_on:
                _v_gps = _verify_cfg.get("gps") if isinstance(_verify_cfg.get("gps"), dict) else None
                _la = _v_gps.get("lever_arm_imu") if _v_gps else None
                _la_ok = isinstance(_la, (list, tuple)) and len(_la) >= 3
                if not _la_ok:
                    raise RuntimeError(
                        "offline patched config missing valid gps.lever_arm_imu (need sequence length>=3) while sensor.gps.enabled=true; "
                        "HBA would use zero lever arm. file={}".format(_patched_config)
                    )
                sys.stderr.write(
                    "{} [CONFIG][VERIFY] gps.lever_arm_imu OK in patched YAML (sensor.gps.enabled=true)\n".format(_LP)
                )
                sys.stderr.flush()

            config_path = _patched_config
            sys.stderr.write("{} [OUTPUT] AUTOMAP_SESSION_OUTPUT_DIR={}\n".format(_LP, session_out))
            sys.stderr.write("{} [CONFIG] offline patched config: {}\n".format(_LP, config_path))
            sys.stderr.flush()
        except Exception as e:
            _log_launch_exception("构建离线 run_* 会话输出目录/补丁配置", e)
    # RViz 配置：前端 / 后端分两个窗口（LIO 仅前端；后端默认无 TF/current_cloud，见 rviz_backend_profile）
    rviz_frontend_config = os.path.join(pkg_share, "rviz", "automap_frontend.rviz")
    if not os.path.isfile(rviz_frontend_config):
        rviz_frontend_config = os.path.join(pkg_share, "rviz", "automap.rviz")
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    if launch_dir not in sys.path:
        sys.path.insert(0, launch_dir)
    from rviz_utils import is_rviz2_installed, resolve_automap_backend_rviz_path
    rb_cfg_arg = LaunchConfiguration("rviz_backend_config", default="").perform(context).strip()
    rb_profile = LaunchConfiguration("rviz_backend_profile", default="default").perform(context).strip()
    rviz_backend_config = resolve_automap_backend_rviz_path(pkg_share, rb_cfg_arg, rb_profile)
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
    else:
        try:
            _rp = os.path.realpath(config_path_str)
        except Exception:
            _rp = config_path_str
        sys.stderr.write("{} [CONFIG] automap_system config_file(raw)={} realpath={}\n".format(_LP, config_path_str, _rp))
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
        if not is_rviz2_installed():
            sys.stderr.write(
                "{} [WARN] use_rviz=true 但系统无可用 rviz2 可执行文件（常见于 Jazzy/Isaac 精简镜像未安装 "
                "ros-{}-rviz2）；跳过 RViz。安装该 deb 或传 use_rviz:=false\n".format(
                    _LP, os.environ.get("ROS_DISTRO", "<distro>")))
            sys.stderr.flush()
        else:
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
    ros2_info_failed = False
    ros2_info_stderr = ""
    metadata_fallback_error = ""

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
        elif result.returncode != 0:
            ros2_info_failed = True
            ros2_info_stderr = (result.stderr or "").strip()
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
        except Exception as e:
            metadata_fallback_error = str(e)
            sys.stderr.write("{} [BAG] [WARN] metadata fallback parse failed: {}\n".format(_LP, metadata_fallback_error))
    return topics, ros2_info_failed, ros2_info_stderr, metadata_fallback_error


def _log_bag_path(context, *args, **kwargs):
    """启动时打印 bag 路径、bag 内发布的话题名称与依赖提示。"""
    bag_file = LaunchConfiguration("bag_file").perform(context)
    sys.stderr.write("{} [BAG] 离线回放 bag_file={}\n".format(_LP, bag_file))

    topics, ros2_info_failed, ros2_info_stderr, metadata_fallback_error = _log_bag_topics(bag_file)
    if topics:
        sys.stderr.write("{} [BAG] rosbag2 将发布的话题 ({} 个): {}\n".format(
            _LP, len(topics), ", ".join(topics)))
    else:
        sys.stderr.write("{} [BAG] 无法获取 bag 话题列表（请确认 bag 路径正确且为 ROS2 格式）\n".format(_LP))
    if ros2_info_failed:
        msg = "{} [BAG] [FATAL] ros2 bag info 失败，启动中止；通常是 metadata.yaml 非法，ros2 bag play 也会失败。stderr={} metadata_fallback_error={}".format(
            _LP, ros2_info_stderr if ros2_info_stderr else "(empty)",
            metadata_fallback_error if metadata_fallback_error else "(none)")
        sys.stderr.write(msg + "\n")
        sys.stderr.write(
            "{} [BAG] [ACTION] 请先修复 metadata.yaml，再重试：python3 scripts/fix_ros2_bag_metadata.py <bag_dir>\n".format(
                _LP
            )
        )
        sys.stderr.flush()
        raise RuntimeError(msg)

    sys.stderr.write("{} [BAG] 若 ros2 bag play 报 Exception on parsing info file / bad conversion，将无数据；odom 来自 fast_livo，请确保 config 中 lid_topic/imu_topic 与 bag 一致\n".format(_LP))
    sys.stderr.write("{} [BAG] 离线模式默认不启动 standalone HBA 节点(use_hba=false)，后端优化由 automap_system 内 HBAOptimizer 负责\n".format(_LP))
    sys.stderr.flush()


def _handle_bag_play_exit(event, context):
    """
    仅在 bag 正常结束时触发 finish_mapping，避免空数据被误保存为成功运行。

    必须作为 OnProcessExit 的 on_exit **可调用对象**（签名 event, context），由 launch
    传入 ProcessExited。若写成 on_exit=[OpaqueFunction(...)] 列表形式，launch 的
    OnActionEventBase 只会返回子动作列表，**不会**把退出事件注入 OpaqueFunction 的 kwargs，
    导致 returncode 恒为 None，误判为异常并跳过 finish_mapping（见 ros2/launch
    event_handlers/on_action_event_base.py handle()）。
    """
    _ = context
    return_code = getattr(event, "returncode", None)
    # 0：ros2 bag play 正常播完。None：少数 launch/平台未填 returncode；仍触发收尾以免
    # 「播完却跳过 finish_mapping」（主因已修：on_exit 必须用本可调用形式，勿用 OpaqueFunction 列表）。
    if return_code == 0 or return_code is None:
        if return_code is None:
            sys.stderr.write(
                "{} [BAG] [WARN] bag play ProcessExited.returncode is None; still trigger finish_mapping "
                "(assume playback ended; if you get an empty map, check `ros2 bag play` exit code / launch version)\n".format(
                    _LP
                )
            )
            sys.stderr.flush()
        else:
            sys.stderr.write("{} [BAG] bag play exited with code 0; trigger finish_mapping\n".format(_LP))
            sys.stderr.flush()
        return [ExecuteProcess(
            cmd=["bash", "-c", "sleep 5 && ros2 service call /automap/finish_mapping std_srvs/srv/Trigger '{}'"],
            output="screen",
        )]

    sys.stderr.write(
        "{} [BAG] [FATAL] bag play exited abnormally (code={}); skip finish_mapping to avoid empty map artifacts.\n".format(
            _LP, return_code
        )
    )
    sys.stderr.write(
        "{} [BAG] [DIAG] root_cause=bag_play_failed effect=no_sensor_data action=shutdown_without_finish_mapping\n".format(
            _LP
        )
    )
    sys.stderr.flush()
    return [EmitEvent(event=Shutdown(reason="bag play failed, abort offline pipeline"))]


def generate_launch_description():
    pkg_share = get_package_share_directory("automap_pro")
    config_default = os.path.join(pkg_share, "config", "system_config.yaml")
    # bag 正常播完后调用 finish_mapping，执行最终 HBA + 保存 + shutdown（离线“播完再结束”）
    bag_play_action = ExecuteProcess(
        cmd=["bash", "-c",
             "set -e; "
             "echo '[automap_offline] [BAG] waiting /automap/ready ...' 1>&2; "
             "for i in $(seq 1 120); do "
             "  if ros2 topic list 2>/dev/null | grep -Fx '/automap/ready' >/dev/null; then break; fi; "
             "  sleep 0.5; "
             "done; "
             "if ! ros2 topic list 2>/dev/null | grep -Fx '/automap/ready' >/dev/null; then "
             "  echo '[automap_offline] [BAG] [FATAL] timeout waiting topic /automap/ready to appear' 1>&2; "
             "  exit 1; "
             "fi; "
             "if ! ros2 topic echo /automap/ready std_msgs/msg/Bool --qos-durability transient_local --once --timeout 10 >/dev/null; then "
             "  echo '[automap_offline] [BAG] [FATAL] failed to receive /automap/ready message' 1>&2; "
             "  exit 1; "
             "fi; "
             "echo '[automap_offline] [BAG] /automap/ready received, starting ros2 bag play' 1>&2; "
             "exec ros2 bag play \"${BAG_FILE}\" --rate \"${BAG_RATE}\" --clock"
             ],
        output="screen",
        additional_env={
            "BAG_FILE": LaunchConfiguration("bag_file"),
            "BAG_RATE": LaunchConfiguration("rate"),
        },
    )
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=config_default, description="Path to system_config.yaml"),
        DeclareLaunchArgument("bag_file", default_value=os.path.expanduser("~/data/mapping.db3"), description="Path to rosbag2"),
        DeclareLaunchArgument("rate", default_value="0.5", description="Playback rate (0.5=half speed, 1.0=realtime)"),
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Whether to start RViz",
        ),
        DeclareLaunchArgument(
            "rviz_backend_config", default_value="",
            description="Override backend RViz config path (empty = use rviz_backend_profile or automap_backend.rviz)",
        ),
        DeclareLaunchArgument(
            "rviz_backend_profile", default_value="default",
            description="Backend RViz preset: default | keyframes_only (global map + optimized/GPS KF paths + keyframe_poses; TF/current_cloud off in that file)",
        ),
        DeclareLaunchArgument("use_external_frontend", default_value="true", description="true=use verified fast_livo node (modular); false=use internal FastLIVO2Wrapper (ESIKF)"),
        DeclareLaunchArgument("use_external_overlap", default_value="true", description="Launch OverlapTransformer descriptor; true=使用 pretrained_overlap_transformer.pth.tar 做回环粗匹配"),
        DeclareLaunchArgument("use_hba", default_value="false", description="Launch standalone HBA node (reads pose.json at startup; offline 时默认 false 避免零位姿崩溃，优化由 automap_system 内 HBAOptimizer 负责)"),
        DeclareLaunchArgument("use_hba_cal_mme", default_value="false", description="Launch HBA cal_MME node (params from system_config.backend.hba_cal_mme)"),
        DeclareLaunchArgument("use_hba_visualize", default_value="false", description="Launch HBA visualize node (params from system_config.backend.hba_visualize)"),
        DeclareLaunchArgument("run_automap_under_gdb", default_value="false", description="Run automap_system_node under GDB; on crash prints full backtrace (requires gdb in container)"),
        DeclareLaunchArgument("gtsam_preload_path", default_value="", description="If set to path of libgtsam.so.4, set LD_PRELOAD to try avoiding lago static-init double free (see docs/FIX_GTSAM_LAGO_STATIC_INIT_DOUBLE_FREE.md)"),
        DeclareLaunchArgument("run_fast_livo_under_gdb", default_value="false", description="Run fastlivo_mapping under GDB; use when frontend SIGSEGV to get backtrace (e.g. frame=10 crash)"),
        OpaqueFunction(function=_log_bag_path),
        OpaqueFunction(function=_launch_nodes_offline),
        bag_play_action,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=bag_play_action,
                on_exit=_handle_bag_play_exit,
            )
        ),
    ])
