#!/usr/bin/env bash
# AutoMap-Pro：Docker 镜像与 ROS 发行版默认值（由 run_automap.sh 等 source）
#
# 默认使用 NGC Isaac ROS 镜像（Ubuntu 24.04 + ROS 2 Jazzy + 新 CUDA），便于 RTX 50 系等 Blackwell GPU。
# 若仍使用自建 automap-env:humble，请设置：
#   export AUTOMAP_DOCKER_IMAGE=automap-env:humble
#   export AUTOMAP_ROS_DISTRO=humble   # 可选，不设则随镜像名推断为 humble
#
# 覆盖镜像 tar 路径：
#   export AUTOMAP_IMAGE_ARCHIVE=/path/to/automap-env_humble.tar
# 显式不设 tar（仅 pull / build）：
#   export AUTOMAP_IMAGE_ARCHIVE=
#
# 运行时快照（由 run_automap.sh 在首次成功运行后写入 thrid_party/automap_cache/docker/，下次优先 docker load）：
#   AUTOMAP_PREFER_RUNTIME_SNAPSHOT / AUTOMAP_RUNTIME_SNAPSHOT_TAR / AUTOMAP_RUNTIME_SNAPSHOT_TAG — 见 run_automap.sh 帮助
#
# shellcheck shell=bash
: "${SCRIPT_DIR:?SCRIPT_DIR must be set before sourcing automap_docker_defaults.sh}"

AUTOMAP_DEFAULT_ISAAC_IMAGE="nvcr.io/nvidia/isaac/ros:isaac_ros_054e16b5c3a328b621af47d26009c348-amd64"

AUTOMAP_DOCKER_IMAGE="${AUTOMAP_DOCKER_IMAGE:-${AUTOMAP_DEFAULT_ISAAC_IMAGE}}"
IMAGE_NAME="${AUTOMAP_DOCKER_IMAGE}"

if [[ "${IMAGE_NAME}" == *"nvcr.io/nvidia/isaac/"* ]] || [[ "${IMAGE_NAME}" == *"/isaac/ros"* ]]; then
  if [[ -v AUTOMAP_IMAGE_ARCHIVE ]]; then
    IMAGE_ARCHIVE="${AUTOMAP_IMAGE_ARCHIVE}"
  else
    IMAGE_ARCHIVE=""
  fi
  AUTOMAP_ROS_DISTRO="${AUTOMAP_ROS_DISTRO:-jazzy}"
else
  if [[ -v AUTOMAP_IMAGE_ARCHIVE ]]; then
    IMAGE_ARCHIVE="${AUTOMAP_IMAGE_ARCHIVE}"
  else
    IMAGE_ARCHIVE="${SCRIPT_DIR}/docker/automap-env_humble.tar"
  fi
  AUTOMAP_ROS_DISTRO="${AUTOMAP_ROS_DISTRO:-humble}"
fi
