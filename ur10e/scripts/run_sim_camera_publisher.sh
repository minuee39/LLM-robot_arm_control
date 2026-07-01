#!/usr/bin/env bash
set -euo pipefail

ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-/home/minwoo/isaacsim}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Do not inherit ROS Python 3.10 paths into Isaac Sim Python 3.11.
unset PYTHONPATH
export PYTHONPATH="${PROJECT_DIR}"

export ROS_DISTRO="${ROS_DISTRO:-humble}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export LD_LIBRARY_PATH="${ISAAC_SIM_ROOT}/exts/isaacsim.ros2.bridge/${ROS_DISTRO}/lib:${LD_LIBRARY_PATH:-}"

exec "${ISAAC_SIM_ROOT}/python.sh" "${PROJECT_DIR}/apps/sim_camera_ros_publisher.py"
