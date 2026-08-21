#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR="/home/minwoo/Desktop/LLM/ur10e"
REPOSITORY_DIR="/home/minwoo/Desktop/LLM"
ROS_WORKSPACE="/home/minwoo/Desktop/LLM/ros2_ur_ws"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

YOLO_PYTHON="${YOLO_PYTHON:-${REPOSITORY_DIR}/.venv/bin/python}"
if [ ! -x "${YOLO_PYTHON}" ]; then
  YOLO_PYTHON="python3"
fi

source /opt/ros/humble/setup.bash
if [ -f "${ROS_WORKSPACE}/install/setup.bash" ]; then
  source "${ROS_WORKSPACE}/install/setup.bash"
fi

exec "${YOLO_PYTHON}" "${PROJECT_DIR}/apps/yolo_camera_node.py" "$@"
