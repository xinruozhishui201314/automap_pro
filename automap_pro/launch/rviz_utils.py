# Shared helpers for RViz2 availability (minimal launch-time dependency checks).
import os

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
