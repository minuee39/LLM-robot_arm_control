#!/usr/bin/env bash
set -euo pipefail

ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-/home/minwoo/isaacsim}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"


# rclpy 문제 해결 방법. 
# Do not inherit ROS Python 3.10 paths into Isaac Sim Python 3.11.
unset PYTHONPATH
export PYTHONPATH="${PROJECT_DIR}"

export ROS_DISTRO="${ROS_DISTRO:-humble}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export LD_LIBRARY_PATH="${ISAAC_SIM_ROOT}/exts/isaacsim.ros2.bridge/${ROS_DISTRO}/lib:${LD_LIBRARY_PATH:-}"

# apps/pick_place_with_camera.py 실행
exec "${ISAAC_SIM_ROOT}/python.sh" "${PROJECT_DIR}/apps/pick_place_with_camera.py" "$@"
