# Shared helpers for RViz2 availability (minimal launch-time dependency checks).
import os
import sys

from ament_index_python.packages import PackageNotFoundError, get_package_prefix


def is_rviz2_installed():
    """
    True only if ament indexes rviz2 and the rviz2 executable exists under the prefix.

    launch_ros.Node resolves via ExecutableInPackage; a bare get_package_prefix() alone
    is insufficient if the index is stale or the package is partially removed.
    """
    try:
        prefix = get_package_prefix("rviz2")
    except PackageNotFoundError:
        return False
    if not prefix:
        return False
    candidates = [
        os.path.join(prefix, "lib", "rviz2", "rviz2"),
        os.path.join(prefix, "bin", "rviz2"),
    ]
    return any(os.path.isfile(p) and os.access(p, os.X_OK) for p in candidates)


def resolve_automap_backend_rviz_path(pkg_share, config_path_arg, profile_arg):
    """
    解析后端 RViz 配置文件路径。

    - config_path_arg: 非空且为现存文件时优先使用（绝对/相对路径均可）。
    - profile_arg: default | keyframes_only | kf_only — 使用包内 automap_backend_keyframes_only.rviz
     （无 LIO 话题；仅全局图 + 优化/GPS 关键帧轨迹 + keyframe_poses）。
    """
    fallback = os.path.join(pkg_share, "rviz", "automap.rviz")
    default_backend = os.path.join(pkg_share, "rviz", "automap_backend.rviz")
    base = default_backend if os.path.isfile(default_backend) else fallback
    cfg = (config_path_arg or "").strip()
    if cfg:
        if os.path.isfile(cfg):
            return os.path.abspath(cfg)
        sys.stderr.write(
            "[rviz_utils] [WARN] rviz_backend_config is not a file: {!r}, using {}\n".format(cfg, base)
        )
        sys.stderr.flush()
        return base
    prof = (profile_arg or "default").strip().lower()
    if prof in ("keyframes_only", "kf_only"):
        kf = os.path.join(pkg_share, "rviz", "automap_backend_keyframes_only.rviz")
        if os.path.isfile(kf):
            return kf
    return base
