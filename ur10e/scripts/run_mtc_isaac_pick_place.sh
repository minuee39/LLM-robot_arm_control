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
  printf '\n[CHECK] MoveIt execution server'
  for _ in {1..30}; do
    if ros2 action list 2>/dev/null | grep -qx "/execute_task_solution"; then
      printf ' : READY\n'
      break
    fi
    printf '.'
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
ros2 launch mtc_tutorial pick_place_demo.launch.py "$@" 2>&1 \
  | tee "${launch_log}" \
  | awk '
      /Preparing PlanningScene/ && !scene_started {
        print "[1/4] Planning scene     : preparing"; scene_started=1; fflush()
      }
      /PlanningScene setup complete/ && !scene_ready {
        print "[1/4] Planning scene     : READY"; scene_ready=1; fflush()
      }
      /Using OMPL planner:/ && !planner_ready {
        sub(/^.*Using OMPL planner: /, "", $0)
        print "[2/4] Motion planner     : " $0; planner_ready=1; fflush()
      }
      /Planning until/ && !planning_started {
        print "[3/4] Trajectory search  : RUNNING"; planning_started=1; fflush()
      }
      /timed out on attempt|Failed to fetch the PlanningScene|planning scene services are unavailable/ {
        line=$0
        sub(/^.*\[mtc_tutorial\]: /, "", line)
        print "[DETAIL] " line; fflush()
      }
      /Executing lowest-cost solution/ && !execution_started {
        line=$0
        sub(/^.*Executing /, "", line)
        print "[3/4] Trajectory search  : " line
        print "[4/4] Robot execution    : RUNNING"
        execution_started=1; fflush()
      }
      /Best solution cost/ {
        line=$0
        sub(/^.*Best solution cost /, "", line)
        print "[QUALITY] trajectory rejected: cost " line; fflush()
      }
      /Task planning failed|Task execution failed|execution skipped/ && !failure_reported {
        print "[FAIL] MTC execution stopped; see the detail above or full log"; fflush()
        failure_reported=1
      }
    '
launch_status=${PIPESTATUS[0]}
set -e

if grep -q "Task execution failed" "${launch_log}" || \
   { grep -q "Executing lowest-cost solution" "${launch_log}" && grep -q "process has died" "${launch_log}"; }; then
  echo "ERROR: MTC execution failed after robot motion started. See launch log: ${launch_log}" >&2
  exit 2
fi

if [ "${launch_status}" -ne 0 ]; then
  exit "${launch_status}"
fi

if grep -Eq "process has died|Task planning failed|execution skipped" "${launch_log}"; then
  echo "ERROR: MTC pick-place failed. See launch log: ${launch_log}" >&2
  exit 1
fi

echo "[4/4] Robot execution    : COMPLETE"
echo "[LOG] Full ROS log       : ${launch_log}"
