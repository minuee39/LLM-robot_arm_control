#!/usr/bin/env bash
set -eo pipefail

ISAAC_ROS2_HUMBLE_LIB=/home/minwoo/isaacsim/exts/isaacsim.ros2.bridge/humble/lib
ISAAC_ROS2_HUMBLE_RCLPY=/home/minwoo/isaacsim/exts/isaacsim.ros2.bridge/humble/rclpy
export ROS_DISTRO=${ROS_DISTRO:-humble}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:$ISAAC_ROS2_HUMBLE_LIB
export PYTHONPATH=$ISAAC_ROS2_HUMBLE_RCLPY
set -u

/home/minwoo/isaacsim/python.sh /home/minwoo/Desktop/LLM/ur10e/apps/isaac_moveit_bridge.py "$@"
