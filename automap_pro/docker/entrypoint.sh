#!/bin/bash
source /opt/ros/noetic/setup.bash
source /automap_ws/devel/setup.bash 2>/dev/null || true
exec "$@"
