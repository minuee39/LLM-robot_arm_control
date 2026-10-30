#!/usr/bin/env bash
set -Eeo pipefail

repository_dir="/home/minwoo/Development/LLM"
workspace_dir="${repository_dir}/ros2_ur_ws"
calibration_file="/tmp/pallet_camera_calibration.json"
vision_scene_file="/tmp/pallet_vision_scene.json"
startup_timeout=240
detection_timeout=45
output_root="${workspace_dir}/regression_results/llm-gui-$(date +%Y%m%d-%H%M%S)"
speed_levels=(1.0 0.7 0.5)

usage() {
  cat <<'EOF'
Usage: scripts/run_pallet_llm_gui.sh [options]

Start Isaac Sim, MoveIt/RViz with the annotated camera image, and an
interactive Gemini palletizing console. Motion starts at speed/acceleration
1.0. Only an execution-dynamics failure restarts a fresh simulator at 0.7,
then 0.5.

Options:
  --output-root PATH      Logs, TaskSpecs, journals, and reports
  --startup-timeout S     Isaac/MoveIt startup timeout (default: 240)
  --detection-timeout S   Stable detection timeout (default: 45)
  -h, --help              Show this help
EOF
}

while (($#)); do
  case "$1" in
    --output-root) output_root="$2"; shift 2 ;;
    --startup-timeout) startup_timeout="$2"; shift 2 ;;
    --detection-timeout) detection_timeout="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ "${output_root}" != /* ]]; then
  output_root="${repository_dir}/${output_root}"
fi

if [[ -z "${GEMINI_API_KEY:-}" ]]; then
  echo "ERROR: GEMINI_API_KEY is not configured in this shell." >&2
  echo "Run: gemini_api" >&2
  exit 2
fi

mkdir -p "${output_root}"
pending_retry_spec="${output_root}/pending-speed-retry.json"
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

stop_stack() {
  stop_group "${vision_pid}"
  stop_group "${launch_pid}"
  wait "${vision_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
  vision_pid=""
  launch_pid=""
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  stop_stack
  exit "${status}"
}
trap cleanup EXIT INT TERM

for speed_index in "${!speed_levels[@]}"; do
  speed="${speed_levels[$speed_index]}"
  attempt_dir="${output_root}/speed-${speed}"
  mkdir -p "${attempt_dir}"
  rm -f -- "${calibration_file}" "${vision_scene_file}"

  echo "[1/4] Isaac Sim + MoveIt + RViz 시작 (속도/가속도 ${speed})"
  # Keep the three-block row near the robot: red/green/blue y positions are
  # 0.10, 0.02, and -0.06 m while preserving the validated 8 cm spacing.
  setsid ros2 launch mtc_tutorial pallet_mtc_isaac_demo.launch.py \
    headless:=false use_rviz:=true "startup_timeout:=${startup_timeout}" \
    block_layout:=three block_x:=-0.32 block_y:=0.10 block_mass_kg:=0.5 \
    >"${attempt_dir}/simulation.log" 2>&1 &
  launch_pid=$!

  ready=false
  deadline=$((SECONDS + startup_timeout))
  while ((SECONDS < deadline)); do
    kill -0 "${launch_pid}" 2>/dev/null || break
    if [[ -s "${calibration_file}" ]] \
        && ros2 action list 2>/dev/null | grep -qx '/execute_task_solution' \
        && ros2 topic list 2>/dev/null | grep -qx '/sim_camera/rgb' \
        && ros2 topic list 2>/dev/null | grep -qx '/sim_camera/depth'; then
      ready=true
      break
    fi
    sleep 1
  done
  if [[ "${ready}" != true ]]; then
    echo "ERROR: simulator stack did not become ready; see ${attempt_dir}/simulation.log" >&2
    exit 1
  fi

  echo "[2/4] RGB-D 물체 인식 시작"
  setsid "${repository_dir}/ur10e/scripts/run_yolo_camera_node.sh" \
    --detector yolo \
    --model "${repository_dir}/runs/detect/train/weights/best.pt" \
    --label-mode color --coordinate-mode pallet \
    --pallet-block-clearance 0.0 \
    --scene-state-file "${calibration_file}" \
    --vision-scene-file "${vision_scene_file}" \
    --debug-image-file "${attempt_dir}/vision-annotated.png" \
    --stability-min-samples 3 --detection-ttl 2.0 --publish-period 0.2 \
    >"${attempt_dir}/vision.log" 2>&1 &
  vision_pid=$!

  echo "[3/4] Gemini 명령 콘솔 시작"
  console_args=(
    --scene-file "${vision_scene_file}"
    --output-root "${attempt_dir}/commands"
    --speed-scaling "${speed}"
    --pending-retry-spec "${pending_retry_spec}"
    --detection-timeout "${detection_timeout}"
  )
  if [[ -s "${pending_retry_spec}" ]]; then
    console_args+=(--retry-spec "${pending_retry_spec}")
  fi

  set +e
  python3 "${repository_dir}/ur10e/pallet_llm_console.py" "${console_args[@]}"
  console_status=$?
  set -e
  stop_stack

  if ((console_status == 75)); then
    if ((speed_index + 1 < ${#speed_levels[@]})); then
      echo "[SPEED] 새 시뮬레이션에서 다음 속도로 재시작합니다."
      continue
    fi
    echo "ERROR: 속도/가속도 0.5에서도 동역학 실행이 실패했습니다." >&2
    exit 2
  fi
  if ((console_status != 0)); then
    exit "${console_status}"
  fi
  echo "[4/4] 통합 세션 종료. 결과: ${output_root}"
  exit 0
done
