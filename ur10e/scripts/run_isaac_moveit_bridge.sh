#!/usr/bin/env bash
set -eo pipefail

ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-/home/minwoo/isaacsim}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ISAAC_ROS2_LIB="${ISAAC_SIM_ROOT}/exts/isaacsim.ros2.bridge/${ROS_DISTRO}/lib"
ISAAC_ROS2_RCLPY="${ISAAC_SIM_ROOT}/exts/isaacsim.ros2.bridge/${ROS_DISTRO}/rclpy"

# Isaac Sim uses its bundled Python 3.11 ROS client and matching ROS 2 C
# libraries. A sourced system Humble environment uses Python 3.10 and a
# different spdlog/fmt ABI, so do not let that overlay leak into this process.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
export ROS_DISTRO
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export LD_LIBRARY_PATH="${ISAAC_ROS2_LIB}:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${ISAAC_ROS2_RCLPY}"
set -u

exec "${ISAAC_SIM_ROOT}/python.sh" \
  /home/minwoo/Desktop/LLM/ur10e/apps/isaac_moveit_bridge.py "$@"
