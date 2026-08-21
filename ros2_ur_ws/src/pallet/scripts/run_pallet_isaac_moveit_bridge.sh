#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
isaac_root="${ISAAC_SIM_ROOT:-/home/minwoo/isaacsim}"
humble_bridge="${isaac_root}/exts/isaacsim.ros2.bridge/humble"

export ROS_DISTRO="${ROS_DISTRO:-humble}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# Isaac's bundled rclpy must resolve its matching rcl/spdlog/fmt libraries
# before any libraries inherited from a sourced /opt/ros/humble environment.
export LD_LIBRARY_PATH="${humble_bridge}/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${humble_bridge}/rclpy:${PYTHONPATH:-}"

exec "${isaac_root}/python.sh" "${script_dir}/pallet_isaac_moveit_bridge.py" "$@"
