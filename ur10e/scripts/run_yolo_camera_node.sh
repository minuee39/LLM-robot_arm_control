#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR="/home/minwoo/Desktop/LLM/ur10e"
ROS_WORKSPACE="/home/minwoo/Desktop/LLM/ros2_ur_ws"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

source /opt/ros/humble/setup.bash
if [ -f "${ROS_WORKSPACE}/install/setup.bash" ]; then
  source "${ROS_WORKSPACE}/install/setup.bash"
fi

exec python3 "${PROJECT_DIR}/apps/yolo_camera_node.py" "$@"
