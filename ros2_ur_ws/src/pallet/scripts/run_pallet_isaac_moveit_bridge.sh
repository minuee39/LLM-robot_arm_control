#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
isaac_root="${ISAAC_SIM_ROOT:-/home/minwoo/isaacsim}"
isaac_ros_distro="${ISAAC_ROS_DISTRO:-humble}"
isaac_bridge="${isaac_root}/exts/isaacsim.ros2.bridge/${isaac_ros_distro}"

export ROS_DISTRO="${isaac_ros_distro}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# Isaac's bundled rclpy must resolve its matching rcl/spdlog/fmt libraries
# before any libraries inherited from a sourced /opt/ros/humble environment.
export LD_LIBRARY_PATH="${isaac_bridge}/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${isaac_bridge}/rclpy:${PYTHONPATH:-}"

exec "${isaac_root}/python.sh" "${script_dir}/pallet_isaac_moveit_bridge.py" "$@"
