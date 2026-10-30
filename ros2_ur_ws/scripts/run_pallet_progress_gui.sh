#!/usr/bin/env bash
set -Eeo pipefail

WORKSPACE="/home/minwoo/Development/LLM/ros2_ur_ws"
CHAPTER="normal"
STARTUP_TIMEOUT="240"
OUTPUT_ROOT=""

usage() {
  cat <<'EOF'
Usage: scripts/run_pallet_progress_gui.sh [options]

Show the validated Pallet project behavior in both Isaac Sim and RViz while
recording the same physical metrics used by the regression runner.

Options:
  --chapter normal    500 g optimized pick/place (default)
  --chapter recovery  Transfer telemetry fault + staged safety recovery
  --chapter both      Run normal first, then recovery in a fresh GUI session
  --output-root PATH  Result root (default: timestamped regression_results path)
  --startup-timeout S GUI startup timeout (default: 240)
  -h, --help           Show this help
EOF
}

while (($#)); do
  case "$1" in
    --chapter)
      [[ $# -ge 2 ]] || { echo "--chapter requires a value" >&2; exit 2; }
      CHAPTER="$2"
      shift 2
      ;;
    --output-root)
      [[ $# -ge 2 ]] || { echo "--output-root requires a value" >&2; exit 2; }
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    --startup-timeout)
      [[ $# -ge 2 ]] || { echo "--startup-timeout requires a value" >&2; exit 2; }
      STARTUP_TIMEOUT="$2"
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

case "$CHAPTER" in
  normal|recovery|both) ;;
  *) echo "--chapter must be normal, recovery, or both" >&2; exit 2 ;;
esac
if ! [[ "$STARTUP_TIMEOUT" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "--startup-timeout must be a positive number" >&2
  exit 2
fi

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

if [[ -z "$OUTPUT_ROOT" ]]; then
  OUTPUT_ROOT="$WORKSPACE/regression_results/gui-progress-$(date +%Y%m%d-%H%M%S)"
fi

run_normal() {
  local result_dir="$OUTPUT_ROOT/normal"
  echo "[GUI] 500 g optimized pick/place: arm=0.50, Cartesian=0.35, accel=0.20"
  python3 "$WORKSPACE/scripts/run_pallet_scenario_matrix.py" \
    --scenario-file "$WORKSPACE/scripts/pallet_scenarios_500g_speed_sweep.json" \
    --scenario m500_offset_w30_arm50_cart35 \
    --repeat 1 --gui --rviz \
    --startup-timeout "$STARTUP_TIMEOUT" \
    --task-timeout 480 --monitor-seconds 520 \
    --slip-threshold 0.006 --min-clearance 0.020 \
    --output-dir "$result_dir"
  echo "[RESULT] $result_dir/report.json"
}

run_recovery() {
  local result_dir="$OUTPUT_ROOT/recovery"
  echo "[GUI] 500 g transfer TELEMETRY_STALE + staged recovery"
  python3 "$WORKSPACE/scripts/run_pallet_scenario_matrix.py" \
    --scenario-file "$WORKSPACE/scripts/pallet_scenarios_500g.json" \
    --scenario mass_500_release_stable \
    --repeat 1 --gui --rviz \
    --safety-cancel-enabled --expect-safety-stop \
    --inject-sim-fault-phase transfer \
    --inject-sim-fault-code TELEMETRY_STALE \
    --pose-stream-timeout 0.75 \
    --startup-timeout "$STARTUP_TIMEOUT" \
    --task-timeout 420 --monitor-seconds 420 \
    --recovery-timeout 120 --settle-seconds 3 \
    --min-clearance 0.020 \
    --output-dir "$result_dir"
  echo "[RESULT] $result_dir/report.json"
}

case "$CHAPTER" in
  normal) run_normal ;;
  recovery) run_recovery ;;
  both)
    run_normal
    run_recovery
    ;;
esac

echo "GUI verification completed. Results: $OUTPUT_ROOT"
