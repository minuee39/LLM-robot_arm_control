#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-/home/minwoo/isaacsim}"
ISAAC_ROS2_LIB="${ISAAC_SIM_ROOT}/exts/isaacsim.ros2.bridge/humble/lib"

export ROS_DISTRO="${ROS_DISTRO:-humble}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export LD_LIBRARY_PATH="${ISAAC_ROS2_LIB}:${LD_LIBRARY_PATH:-}"

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH

# Isaac Sim uses Python 3.11, while ROS Humble system rclpy is Python 3.10.
# Do not let /opt/ros Python packages leak into Isaac's Python process.
if [[ -n "${PYTHONPATH:-}" ]]; then
  PYTHONPATH="$(printf '%s' "${PYTHONPATH}" | tr ':' '\n' | grep -v '^/opt/ros/' | paste -sd ':' -)"
  export PYTHONPATH
fi

exec "${ISAAC_SIM_ROOT}/python.sh" "${PROJECT_DIR}/apps/target_follow.py" "$@"
