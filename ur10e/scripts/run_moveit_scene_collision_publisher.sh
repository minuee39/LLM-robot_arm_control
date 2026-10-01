#!/usr/bin/env bash
set -eo pipefail

cd /home/minwoo/Development/LLM/ros2_ur_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run isaac_moveit_bridge scene_collision_publisher "$@"
