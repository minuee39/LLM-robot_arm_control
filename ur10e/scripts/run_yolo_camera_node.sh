#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR="/home/minwoo/Development/LLM/ur10e"
REPOSITORY_DIR="/home/minwoo/Development/LLM"
ROS_WORKSPACE="/home/minwoo/Development/LLM/ros2_ur_ws"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

source /opt/ros/jazzy/setup.bash
if [ -f "${ROS_WORKSPACE}/install/setup.bash" ]; then
  source "${ROS_WORKSPACE}/install/setup.bash"
fi

detector_backend="yolo"
previous_argument=""
for argument in "$@"; do
  if [ "${previous_argument}" = "--detector" ]; then
    detector_backend="${argument}"
  elif [[ "${argument}" == --detector=* ]]; then
    detector_backend="${argument#--detector=}"
  fi
  previous_argument="${argument}"
done

if [ -n "${YOLO_PYTHON:-}" ]; then
  python_candidates=("${YOLO_PYTHON}")
else
  python_candidates=(
    "${REPOSITORY_DIR}/.yolo_venv/bin/python"
    "${REPOSITORY_DIR}/.venv/bin/python"
    "/usr/bin/python3"
    "python3"
  )
fi

YOLO_PYTHON=""
for candidate in "${python_candidates[@]}"; do
  required_modules='import cv2, numpy, rclpy'
  if [ "${detector_backend}" = "yolo" ]; then
    required_modules='import cv2, numpy, rclpy, ultralytics'
  fi
  if command -v "${candidate}" >/dev/null 2>&1 \
      && "${candidate}" -c "${required_modules}" >/dev/null 2>&1; then
    YOLO_PYTHON="${candidate}"
    break
  fi
done
if [ -z "${YOLO_PYTHON}" ]; then
  echo "ERROR: no Python interpreter provides the ${detector_backend} detector dependencies." >&2
  if [ "${detector_backend}" = "yolo" ]; then
    echo "Run: bash ur10e/scripts/setup_yolo_runtime.sh" >&2
  fi
  exit 1
fi

exec "${YOLO_PYTHON}" "${PROJECT_DIR}/apps/yolo_camera_node.py" "$@"
