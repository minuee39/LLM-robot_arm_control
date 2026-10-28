#!/usr/bin/env bash
set -Eeo pipefail

workspace="/home/minwoo/Development/LLM/ros2_ur_ws"
scenario_file="${workspace}/scripts/pallet_scenarios_soak_500g.json"
repeat=3
output_dir="${workspace}/regression_results/chapter-10-soak-$(date +%Y%m%d-%H%M%S)"

usage() {
  cat <<'EOF'
Usage: scripts/run_500g_soak_regression.sh [options]

Run the three validated 500 g geometry/speed profiles repeatedly. Results are
written atomically after every run and an interrupted output directory can be
resumed without repeating passed runs.

Options:
  --output-dir PATH  Campaign directory (required to resume an old campaign)
  --repeat N         Repetitions per profile (default: 3; total 9 runs)
  --dry-run          Validate the complete matrix without starting Isaac Sim
  -h, --help         Show this help
EOF
}

dry_run=false
while (($#)); do
  case "$1" in
    --output-dir) output_dir="$2"; shift 2 ;;
    --repeat) repeat="$2"; shift 2 ;;
    --dry-run) dry_run=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "${repeat}" =~ ^[1-9][0-9]*$ ]] || { echo "--repeat must be a positive integer" >&2; exit 2; }
mkdir -p "$(dirname "${output_dir}")"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
source /opt/ros/jazzy/setup.bash
source "${workspace}/install/setup.bash"

arguments=(
  --scenario-file "${scenario_file}"
  --repeat "${repeat}"
  --task-timeout 300
  --monitor-seconds 330
  --startup-timeout 240
  --inter-run-cooldown 10
  --slip-threshold 0.006
  --output-dir "${output_dir}"
)
[[ -f "${output_dir}/report.json" ]] && arguments+=(--resume)
[[ "${dry_run}" == true ]] && arguments+=(--dry-run)

exec python3 "${workspace}/scripts/run_pallet_scenario_matrix.py" "${arguments[@]}"
