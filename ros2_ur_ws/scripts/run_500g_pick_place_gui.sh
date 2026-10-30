#!/usr/bin/env bash
set -Eeo pipefail

WORKSPACE="/home/minwoo/Development/LLM/ros2_ur_ws"
SPEED_PROFILE="validated-fast"
STARTUP_TIMEOUT="240"
OUTPUT_DIR=""

usage() {
  cat <<'EOF'
Usage: scripts/run_500g_pick_place_gui.sh [options]

Run one 500 g pick-and-place with both Isaac Sim and RViz visible.

Options:
  --speed-profile validated-fast  Passed regression profile (default, 0.35 velocity)
  --speed-profile experimental-05 Experimental 0.50 velocity profile; may slip
  --startup-timeout SECONDS        GUI startup timeout (default: 240)
  --output-dir PATH                Result directory (default: timestamped regression_results)
  -h, --help                       Show this help
EOF
}

while (($#)); do
  case "$1" in
    --speed-profile)
      [[ $# -ge 2 ]] || { echo "--speed-profile requires a value" >&2; exit 2; }
      SPEED_PROFILE="$2"
      shift 2
      ;;
    --startup-timeout)
      [[ $# -ge 2 ]] || { echo "--startup-timeout requires a value" >&2; exit 2; }
      STARTUP_TIMEOUT="$2"
      shift 2
      ;;
    --output-dir)
      [[ $# -ge 2 ]] || { echo "--output-dir requires a value" >&2; exit 2; }
      OUTPUT_DIR="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "$SPEED_PROFILE" in
  validated-fast)
    ;;
  experimental-05)
    echo "WARNING: 0.50 profile has shown slip with some 500 g block widths." >&2
    ;;
  *)
    echo "Invalid speed profile: $SPEED_PROFILE" >&2
    exit 2
    ;;
esac

if ! [[ "$STARTUP_TIMEOUT" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "--startup-timeout must be a positive number" >&2
  exit 2
fi

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

if [[ -z "$OUTPUT_DIR" ]]; then
  OUTPUT_DIR="$WORKSPACE/regression_results/500g-gui-$(date +%Y%m%d-%H%M%S)"
fi

SCENARIO_FILE="$WORKSPACE/scripts/pallet_scenarios_500g_gui.json"
SCENARIO="m500_offset_w30_v35"
if [[ "$SPEED_PROFILE" == "experimental-05" ]]; then
  SCENARIO="m500_offset_w30_v50"
fi

echo "Starting measured Isaac Sim + RViz run ($SPEED_PROFILE)..."
python3 "$WORKSPACE/scripts/run_pallet_scenario_matrix.py" \
  --scenario-file "$SCENARIO_FILE" \
  --scenario "$SCENARIO" \
  --repeat 1 \
  --gui \
  --rviz \
  --startup-timeout "$STARTUP_TIMEOUT" \
  --task-timeout 480 \
  --monitor-seconds 520 \
  --slip-threshold 0.006 \
  --output-dir "$OUTPUT_DIR"

echo "Measured result: $OUTPUT_DIR/report.json"
