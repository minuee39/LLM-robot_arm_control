#!/usr/bin/env bash
set -Eeo pipefail

repository_dir="/home/minwoo/Development/LLM"
workspace_dir="${repository_dir}/ros2_ur_ws"
calibration_file="/tmp/pallet_camera_calibration.json"
vision_scene_file="/tmp/pallet_vision_scene.json"
startup_timeout=240
detection_timeout=45
headless=false
use_rviz=true
task_spec=""
motion_velocity=1.0
motion_acceleration=1.0
gripper_close_position=""
release_open_position=""
place_yaw=""
object_place_yaw_arguments=()
grasp_angle_delta=""
cartesian_transfer=false
upright_transfer=false
scene_obstacles=false
camera_view="source"
rgbd_resolution="640x480"
rgbd_clock="simulation"
transfer_height=""
block_contact_offset="-1.0"
block_static_friction="-1.0"
jaw_friction="-1.0"
contact_geometry_log=false
arm_settle_seconds="0.0"
arm_goal_dwell_seconds="0.0"
solver_velocity_iterations="-1"
external_forces_every_iteration="default"
totg_path_tolerance=""
output_dir="${workspace_dir}/regression_results/palletizing-$(date +%Y%m%d-%H%M%S)"

usage() {
  cat <<'EOF'
Usage: scripts/run_palletizing_gui.sh [options]

Detect and sequentially palletize red, green, and blue 500 g blocks. Isaac Sim
and RViz are visible by default. The job stops at the first failed item.

Options:
  --output-dir PATH      Logs, task journals, and report directory
  --startup-timeout S    Isaac/MoveIt startup timeout (default: 240)
  --detection-timeout S  Per-object detection timeout (default: 45)
  --totg-path-tolerance R
                        Optional Pallet TOTG path deviation experiment in rad
  --headless             Hide Isaac Sim and RViz
  --task-spec PATH       Validated declarative job JSON
  --velocity-scaling V   Arm/Cartesian velocity scaling (default: 1.0)
  --acceleration-scaling A
                         Arm/Cartesian acceleration scaling (default: 1.0)
  --gripper-close-position M
                         Optional jaw target in metres
  --release-open-position M
                         Final release travel per jaw in metres; pre-grasp unchanged
  --place-yaw RAD        Optional fixed object placement yaw
  --object-place-yaw OBJECT RAD
                         Per-object execution yaw in [-pi, pi]; repeatable
  --grasp-angle-delta RAD
                         Optional angular spacing for grasp candidates
  --cartesian-transfer   Keep the loaded move-to-place path straight
  --upright-transfer     Constrain loaded-transfer roll/pitch, allow yaw
  --scene-obstacles      Require all three camera blocks and add non-picked collision envelopes
  --camera-view VIEW     Fixed RGB-D view: source (default) or workcell
  --rgbd-resolution SIZE RGB-D resolution: 640x480 (default) or 1280x960
  --rgbd-clock CLOCK     RGB-D clock: simulation (default) or system
  --transfer-height M    Override both lift and lower distance
  --block-contact-offset-m M
                         Experimental block contact offset (-1 uses Isaac default)
  --block-static-friction V
                         Experimental static friction (-1 uses Isaac default)
  --jaw-friction V       Experimental jaw static/dynamic friction (-1 preserves binding)
  --contact-geometry-log Log actual pad poses and red block geometry at 1 Hz
  --arm-settle-seconds S Require low arm velocity at each goal for S seconds (0 disables)
  --arm-goal-dwell-seconds S
                         Timed position-goal hold experiment (not velocity settling)
  --solver-velocity-iterations N
                         Experimental articulation velocity iterations (-1 preserves default)
  --external-forces-every-iteration
                         Apply TGS external forces every position iteration (experimental)
  -h, --help             Show this help
EOF
}

while (($#)); do
  case "$1" in
    --output-dir) output_dir="$2"; shift 2 ;;
    --startup-timeout) startup_timeout="$2"; shift 2 ;;
    --detection-timeout) detection_timeout="$2"; shift 2 ;;
    --headless) headless=true; use_rviz=false; shift ;;
    --task-spec) task_spec="$2"; shift 2 ;;
    --velocity-scaling) motion_velocity="$2"; shift 2 ;;
    --acceleration-scaling) motion_acceleration="$2"; shift 2 ;;
    --gripper-close-position) gripper_close_position="$2"; shift 2 ;;
    --release-open-position) release_open_position="$2"; shift 2 ;;
    --place-yaw) place_yaw="$2"; shift 2 ;;
    --object-place-yaw)
      if (($# < 3)); then
        echo "--object-place-yaw requires OBJECT RAD" >&2; exit 2
      fi
      object_place_yaw_arguments+=(--object-place-yaw "$2" "$3"); shift 3 ;;
    --grasp-angle-delta) grasp_angle_delta="$2"; shift 2 ;;
    --cartesian-transfer) cartesian_transfer=true; shift ;;
    --upright-transfer) upright_transfer=true; shift ;;
    --scene-obstacles) scene_obstacles=true; shift ;;
    --camera-view) camera_view="$2"; shift 2 ;;
    --rgbd-resolution) rgbd_resolution="$2"; shift 2 ;;
    --rgbd-clock) rgbd_clock="$2"; shift 2 ;;
    --transfer-height) transfer_height="$2"; shift 2 ;;
    --block-contact-offset-m) block_contact_offset="$2"; shift 2 ;;
    --block-static-friction) block_static_friction="$2"; shift 2 ;;
    --jaw-friction) jaw_friction="$2"; shift 2 ;;
    --contact-geometry-log) contact_geometry_log=true; shift ;;
    --arm-settle-seconds) arm_settle_seconds="$2"; shift 2 ;;
    --arm-goal-dwell-seconds) arm_goal_dwell_seconds="$2"; shift 2 ;;
    --solver-velocity-iterations) solver_velocity_iterations="$2"; shift 2 ;;
    --external-forces-every-iteration) external_forces_every_iteration="true"; shift ;;
    --totg-path-tolerance) totg_path_tolerance="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

job_arguments=()
case "${camera_view}" in
  source|workcell) ;;
  *) echo "--camera-view must be source or workcell" >&2; exit 2 ;;
esac
case "${rgbd_resolution}" in
  640x480|1280x960) ;;
  *) echo "--rgbd-resolution must be 640x480 or 1280x960" >&2; exit 2 ;;
esac
case "${rgbd_clock}" in
  simulation|system) ;;
  *) echo "--rgbd-clock must be simulation or system" >&2; exit 2 ;;
esac
if [[ "${scene_obstacles}" == true ]]; then
  job_arguments+=(--scene-obstacles)
fi
if [[ -n "${totg_path_tolerance}" ]]; then
  job_arguments+=(--totg-path-tolerance "${totg_path_tolerance}")
fi
if [[ -n "${task_spec}" ]]; then
  python3 "${repository_dir}/ur10e/pallet_task_spec.py" "${task_spec}"
  job_arguments+=(--task-spec "${task_spec}")
fi
if [[ -n "${gripper_close_position}" ]]; then
  job_arguments+=(--gripper-close-position "${gripper_close_position}")
fi
if [[ -n "${release_open_position}" ]]; then
  job_arguments+=(--release-open-position "${release_open_position}")
fi
if [[ -n "${place_yaw}" ]]; then
  job_arguments+=(--place-yaw "${place_yaw}")
fi
if [[ -n "${grasp_angle_delta}" ]]; then
  job_arguments+=(--grasp-angle-delta "${grasp_angle_delta}")
fi
if [[ "${cartesian_transfer}" == true ]]; then
  job_arguments+=(--cartesian-transfer)
fi
if [[ "${upright_transfer}" == true ]]; then
  job_arguments+=(--upright-transfer)
fi
if [[ -n "${transfer_height}" ]]; then
  job_arguments+=(--transfer-height "${transfer_height}")
fi
job_arguments+=("${object_place_yaw_arguments[@]}")
# Validate execution overrides against the actual job before any simulator starts.
python3 "${repository_dir}/ur10e/palletizing_job.py" \
  --output-dir "${output_dir}/job" --validate-only "${job_arguments[@]}"

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
rm -f -- "${calibration_file}" "${vision_scene_file}"

echo "[1/4] Starting three-block Isaac workcell and MoveIt"
# Keep red/green/blue at y=0.10, 0.02, and -0.06 m near the robot.
setsid ros2 launch mtc_tutorial pallet_mtc_isaac_demo.launch.py \
  "headless:=${headless}" "use_rviz:=${use_rviz}" \
  "startup_timeout:=${startup_timeout}" safety_cancel_enabled:=true block_layout:=three \
  block_x:=-0.32 block_y:=0.10 block_mass_kg:=0.5 \
  "block_contact_offset_m:=${block_contact_offset}" \
  "block_static_friction:=${block_static_friction}" \
  "jaw_friction:=${jaw_friction}" \
  "contact_geometry_log:=${contact_geometry_log}" \
  "arm_settle_seconds:=${arm_settle_seconds}" \
  "arm_goal_dwell_seconds:=${arm_goal_dwell_seconds}" \
  "solver_velocity_iterations:=${solver_velocity_iterations}" \
  "external_forces_every_iteration:=${external_forces_every_iteration}" \
  "camera_view:=${camera_view}" \
  "rgbd_resolution:=${rgbd_resolution}" \
  "rgbd_clock:=${rgbd_clock}" \
  >"${output_dir}/simulation.log" 2>&1 &
launch_pid=$!

ready=false
deadline=$((SECONDS + startup_timeout))
while ((SECONDS < deadline)); do
  kill -0 "${launch_pid}" 2>/dev/null || break
  if [[ -s "${calibration_file}" ]] \
      && ros2 action list 2>/dev/null | grep -qx '/execute_task_solution' \
      && ros2 topic list 2>/dev/null | grep -qx '/sim_camera/rgb'; then
    ready=true
    break
  fi
  sleep 1
done
[[ "${ready}" == true ]] || { echo "Workcell startup failed" >&2; exit 1; }

echo "[2/4] Starting calibrated three-color detection"
setsid "${repository_dir}/ur10e/scripts/run_yolo_camera_node.sh" \
  --detector yolo \
    --model "${repository_dir}/runs/detect/train/weights/best.pt" \
    --label-mode color --coordinate-mode pallet \
    --pallet-block-clearance 0.0 \
    --scene-state-file "${calibration_file}" \
  --vision-scene-file "${vision_scene_file}" \
  --debug-image-file "${output_dir}/vision-annotated.png" \
  --stability-min-samples 3 --detection-ttl 2.0 --publish-period 0.2 \
  >"${output_dir}/vision.log" 2>&1 &
vision_pid=$!

echo "[3/4] Palletizing red, green, and blue blocks"
python3 "${repository_dir}/ur10e/palletizing_job.py" \
  --scene-file "${vision_scene_file}" --output-dir "${output_dir}/job" \
  --detection-timeout "${detection_timeout}" \
  --arm-velocity-scaling "${motion_velocity}" \
  --arm-acceleration-scaling "${motion_acceleration}" \
  --cartesian-velocity-scaling "${motion_velocity}" \
  --cartesian-acceleration-scaling "${motion_acceleration}" \
  "${job_arguments[@]}" | tee "${output_dir}/job.log"

echo "[4/4] PASS: requested job completed"
echo "[RESULT] ${output_dir}/job/report.json"
