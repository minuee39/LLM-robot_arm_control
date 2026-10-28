#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /home/minwoo/Development/LLM/ros2_ur_ws/install/setup.bash

exec python3 /home/minwoo/Development/LLM/ros2_ur_ws/scripts/run_pallet_scenario_matrix.py \
  --scenario-file /home/minwoo/Development/LLM/ros2_ur_ws/scripts/pallet_scenarios_regression_12.json \
  --repeat 3 \
  --task-timeout 240 \
  --output-dir /home/minwoo/Development/LLM/ros2_ur_ws/regression_results/pallet-regression-12x3-20261001
