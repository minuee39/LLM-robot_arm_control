#!/usr/bin/env bash
set -eo pipefail

cd /home/minwoo/Desktop/LLM/ros2_ur_ws
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
source /opt/ros/humble/setup.bash
source install/setup.bash

lock_file="/tmp/mtc_isaac_demo.lock"
exec 9>"${lock_file}"
if ! flock -n 9; then
  echo "ERROR: Another MTC Isaac demo is already running (${lock_file})." >&2
  echo "Stop the existing demo before starting a new one." >&2
  exit 1
fi

topic_publisher_count() {
  ros2 topic info "$1" 2>/dev/null | awk '/Publisher count:/ { print $3 }'
}

action_server_count() {
  ros2 action info "$1" 2>/dev/null | awk '/Action servers:/ { print $3 }'
}

if [ "$(topic_publisher_count /isaac_joint_states)" != "1" ]; then
  echo "ERROR: Isaac joint feedback is not available on /isaac_joint_states." >&2
  echo "Start ur10e/scripts/run_isaac_moveit_bridge.sh first." >&2
  exit 1
fi

bridge_pid=""
cleanup() {
  if [ -n "${bridge_pid}" ] && kill -0 "${bridge_pid}" 2>/dev/null; then
    kill "${bridge_pid}" 2>/dev/null || true
    wait "${bridge_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [ "$(action_server_count /ur_manipulator_controller/follow_joint_trajectory)" != "1" ]; then
  bridge_log="/tmp/isaac_moveit_action_bridge.log"
  ros2 run isaac_moveit_bridge action_bridge >"${bridge_log}" 2>&1 &
  bridge_pid=$!

  for _ in {1..50}; do
    if [ "$(action_server_count /ur_manipulator_controller/follow_joint_trajectory)" = "1" ]; then
      break
    fi
    sleep 0.1
  done

  if [ "$(action_server_count /ur_manipulator_controller/follow_joint_trajectory)" != "1" ]; then
    echo "ERROR: Isaac MoveIt action bridge did not start. See ${bridge_log}." >&2
    exit 1
  fi
fi

ros2 launch mtc_tutorial mtc_isaac_demo.launch.py "$@"
