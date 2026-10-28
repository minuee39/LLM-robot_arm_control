#!/usr/bin/env bash
set -Eeo pipefail

repository_dir="/home/minwoo/Development/LLM"
workspace_dir="${repository_dir}/ros2_ur_ws"
calibration_file="/tmp/pallet_camera_calibration.json"
vision_scene_file="/tmp/pallet_vision_scene.json"
block_x="-0.32"
block_y="0.26"
place_x="0.30"
place_y="-0.30"
place_z="0.025"
block_mass_kg="0.5"
startup_timeout="240"
detection_timeout="45"
headless="false"
use_rviz="true"
output_dir="${workspace_dir}/regression_results/vision-pick-place-$(date +%Y%m%d-%H%M%S)"

usage() {
  cat <<'EOF'
Usage: scripts/run_pallet_vision_pick_place_gui.sh [options]

Detect the 500 g red block from Isaac Sim RGB-D topics, transform the stable
camera detection into base_footprint, and execute the resulting MTC pick/place.
Isaac Sim and RViz are visible by default.

Options:
  --block-x M           Initial block X in base_footprint (default: -0.32)
  --block-y M           Initial block Y in base_footprint (default: 0.26)
  --place-x M           Placement X (default: 0.30)
  --place-y M           Placement Y (default: -0.30)
  --place-z M           Placement center Z (default: 0.025)
  --mass-kg KG          Block mass (default: 0.5)
  --startup-timeout S   Isaac/MoveIt startup timeout (default: 240)
  --detection-timeout S Stable camera-detection timeout (default: 45)
  --output-dir PATH     Logs and captured scene directory
  --headless             Hide Isaac Sim and RViz (CI/debug use)
  -h, --help             Show this help
EOF
}

while (($#)); do
  case "$1" in
    --block-x) block_x="$2"; shift 2 ;;
    --block-y) block_y="$2"; shift 2 ;;
    --place-x) place_x="$2"; shift 2 ;;
    --place-y) place_y="$2"; shift 2 ;;
    --place-z) place_z="$2"; shift 2 ;;
    --mass-kg) block_mass_kg="$2"; shift 2 ;;
    --startup-timeout) startup_timeout="$2"; shift 2 ;;
    --detection-timeout) detection_timeout="$2"; shift 2 ;;
    --output-dir) output_dir="$2"; shift 2 ;;
    --headless) headless="true"; use_rviz="false"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

mkdir -p "${output_dir}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
source /opt/ros/jazzy/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u

launch_pid=""
vision_pid=""

stop_group() {
  local pid="$1"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill -INT -- "-${pid}" 2>/dev/null || true
    for _ in {1..30}; do
      kill -0 "${pid}" 2>/dev/null || return 0
      sleep 0.2
    done
    kill -TERM -- "-${pid}" 2>/dev/null || true
  fi
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  stop_group "${vision_pid}"
  stop_group "${launch_pid}"
  wait "${vision_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
  exit "${status}"
}
trap cleanup EXIT INT TERM

# Reject stale calibration/detection files from earlier simulator sessions.
rm -f -- "${calibration_file}" "${vision_scene_file}"

if [[ "${headless}" == true ]]; then
  echo "[1/4] Starting headless Isaac Sim and MoveIt"
else
  echo "[1/4] Starting Isaac Sim, MoveIt, and RViz"
fi
setsid ros2 launch mtc_tutorial pallet_mtc_isaac_demo.launch.py \
  "headless:=${headless}" "use_rviz:=${use_rviz}" \
  "startup_timeout:=${startup_timeout}" \
  "block_x:=${block_x}" "block_y:=${block_y}" \
  "block_mass_kg:=${block_mass_kg}" \
  >"${output_dir}/simulation.log" 2>&1 &
launch_pid=$!

ready=false
deadline=$((SECONDS + startup_timeout))
while ((SECONDS < deadline)); do
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "Simulator stack exited before becoming ready. See ${output_dir}/simulation.log" >&2
    exit 1
  fi
  if [[ -s "${calibration_file}" ]] \
      && ros2 action list 2>/dev/null | grep -qx '/execute_task_solution' \
      && ros2 topic list 2>/dev/null | grep -qx '/sim_camera/rgb' \
      && ros2 topic list 2>/dev/null | grep -qx '/sim_camera/depth' \
      && ros2 topic list 2>/dev/null | grep -qx '/sim_camera/camera_info'; then
    ready=true
    break
  fi
  sleep 1
done
if [[ "${ready}" != true ]]; then
  echo "Timed out waiting for Isaac RGB-D and MoveIt. See ${output_dir}/simulation.log" >&2
  exit 1
fi

echo "[2/4] Detecting the block from calibrated RGB-D data"
setsid "${repository_dir}/ur10e/scripts/run_yolo_camera_node.sh" \
  --detector yolo \
    --model "${repository_dir}/runs/detect/train/weights/best.pt" \
    --label-mode color --coordinate-mode pallet \
    --pallet-block-clearance 0.0 \
    --scene-state-file "${calibration_file}" \
  --vision-scene-file "${vision_scene_file}" \
  --stability-min-samples 3 --detection-ttl 2.0 --publish-period 0.2 \
  >"${output_dir}/vision.log" 2>&1 &
vision_pid=$!

command_json=$(printf \
  '{"action":"pick_place","pick_object":"red_block","place_xyz":[%s,%s,%s]}' \
  "${place_x}" "${place_y}" "${place_z}")
detected=false
deadline=$((SECONDS + detection_timeout))
while ((SECONDS < deadline)); do
  if ! kill -0 "${vision_pid}" 2>/dev/null; then
    echo "Vision node exited before producing a stable scene. See ${output_dir}/vision.log" >&2
    exit 1
  fi
  if python3 "${repository_dir}/ur10e/mtc_command_runner.py" \
      --robot-profile pallet_gripper \
      --pallet-scene-file "${vision_scene_file}" \
      --vision-max-age 2.0 --dry-run --once "${command_json}" \
      >"${output_dir}/scene-validation.log" 2>&1; then
    detected=true
    break
  fi
  sleep 0.5
done
if [[ "${detected}" != true ]]; then
  echo "Timed out waiting for a stable red-block detection. See ${output_dir}/vision.log" >&2
  exit 1
fi
cp -- "${vision_scene_file}" "${output_dir}/detected-scene.json"

echo "[3/4] Executing camera-derived MTC pick/place"
set +e
python3 "${repository_dir}/ur10e/mtc_command_runner.py" \
  --robot-profile pallet_gripper \
  --pallet-execution-mode standard \
  --pallet-scene-file "${vision_scene_file}" \
  --vision-max-age 5.0 --once "${command_json}" \
  2>&1 | tee "${output_dir}/command.log"
command_status=${PIPESTATUS[0]}
set -e
if ((command_status != 0)); then
  echo "Camera-derived pick/place failed with code ${command_status}." >&2
  exit "${command_status}"
fi

echo "[4/4] PASS: camera detection, MTC execution, and physical placement check"
echo "[RESULT] ${output_dir}"
