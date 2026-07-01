#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_log}"

ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 world sim_camera >/tmp/sim_camera_static_tf.log 2>&1 &
TF_PID=$!

cleanup() {
  kill "${TF_PID}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

rviz2 -d "${PROJECT_DIR}/config/rviz/sim_camera.rviz"
