#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
isaac_env="${PALLET_ISAAC_ENV:-/home/minwoo/Development/isaac_env}"
bridge="${isaac_env}/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy"
# Use Isaac's Python 3.11 ROS libraries, not system Jazzy's Python 3.12 modules.
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="${bridge}/lib"
export PYTHONPATH="${bridge}/rclpy"
exec "${isaac_env}/bin/python" "${script_dir}/isaac_sim.py" "$@"
