#!/usr/bin/env bash
set -eo pipefail

cd /home/minwoo/Desktop/LLM/ros2_ur_ws
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
source /opt/ros/humble/setup.bash
source install/setup.bash

has_arg() {
  local needle="$1"
  shift
  for arg in "$@"; do
    if [ "$arg" = "$needle" ]; then
      return 0
    fi
  done
  return 1
}

if [ "$#" -eq 0 ]; then
  set -- \
    object_id:=red_block \
    object_x:=-0.30 \
    object_y:=0.30 \
    object_z:=0.05 \
    object_size_x:=0.10 \
    object_size_y:=0.0515 \
    object_size_z:=0.10 \
    place_x:=0.30 \
    place_y:=-0.30 \
    place_z:=0.05
fi

if ! has_arg "--show-args" "$@"; then
  echo "Checking /execute_task_solution action server..."
  for _ in {1..30}; do
    if ros2 action list 2>/dev/null | grep -qx "/execute_task_solution"; then
      echo "Found /execute_task_solution action server."
      break
    fi
    echo "Waiting for /execute_task_solution action server..."
    sleep 1
  done

  if ! ros2 action list 2>/dev/null | grep -qx "/execute_task_solution"; then
    echo "ERROR: /execute_task_solution action server is not available."
    echo "Start this in another terminal first:"
    echo "  /home/minwoo/Desktop/LLM/ur10e/scripts/run_mtc_isaac_demo.sh"
    exit 1
  fi
fi

launch_log="$(mktemp /tmp/mtc_pick_place_launch.XXXXXX.log)"
set +e
ros2 launch mtc_tutorial pick_place_demo.launch.py "$@" 2>&1 | tee "${launch_log}"
launch_status=${PIPESTATUS[0]}
set -e

if [ "${launch_status}" -ne 0 ]; then
  exit "${launch_status}"
fi

if grep -Eq "process has died|Task planning failed|Task execution failed" "${launch_log}"; then
  echo "ERROR: MTC pick-place failed. See launch log: ${launch_log}" >&2
  exit 1
fi
