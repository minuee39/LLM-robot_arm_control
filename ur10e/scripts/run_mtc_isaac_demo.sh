#!/usr/bin/env bash
set -eo pipefail

cd /home/minwoo/Desktop/LLM/ros2_ur_ws
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mtc_tutorial mtc_isaac_demo.launch.py "$@"
