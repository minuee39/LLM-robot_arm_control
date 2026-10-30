#!/usr/bin/env bash
set -o pipefail

source /opt/ros/jazzy/setup.bash
source /home/minwoo/Development/LLM/ros2_ur_ws/install/setup.bash

RUNNER=/home/minwoo/Development/LLM/ros2_ur_ws/scripts/run_pallet_scenario_matrix.py
SCENARIOS=/home/minwoo/Development/LLM/ros2_ur_ws/scripts/pallet_scenarios_regression_12.json
RESULT_ROOT=/home/minwoo/Development/LLM/ros2_ur_ws/regression_results/pallet-regression-final-7-20261001

run_group() {
  local output_dir=$1
  shift
  local resume_args=()
  if [[ -f "${output_dir}/report.json" ]]; then
    resume_args=(--resume)
  fi
  python3 "${RUNNER}" \
    --scenario-file "${SCENARIOS}" \
    --task-timeout 480 \
    --monitor-seconds 520 \
    --slip-threshold 0.006 \
    --inter-run-cooldown 10 \
    --output-dir "${output_dir}" \
    "${resume_args[@]}" \
    "$@"
}

# Overall matrix runs 30-36: run 30 once, followed by the final two scenarios x3.
run_group "${RESULT_ROOT}/run-30" \
  --scenario m500_offset_w30_v35 --repeat 1
run_30_status=$?
run_group "${RESULT_ROOT}/corrected-runs-31-36" \
  --scenario m500_base_w25_v20 \
  --scenario m500_offset_w40_v20 \
  --repeat 3
runs_31_36_status=$?

if (( run_30_status != 0 || runs_31_36_status != 0 )); then
  exit 1
fi
