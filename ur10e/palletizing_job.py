#!/usr/bin/env python3
"""Run a fail-closed sequence of camera-derived Pallet pick/place tasks."""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

from mtc_command_runner import load_pallet_scene
from pallet_task_spec import load_tasks


DEFAULT_PLACEMENTS = {
    # Keep every destination close to the single-block validated pose
    # (0.30, -0.30) while leaving 10 mm clearance between 30 mm blocks.
    # Pick from the inside of the source row outward so the arm cannot brush
    # an unhandled inner block while reaching the outer ones.
    "blue_block": [0.28, -0.28, 0.025],
    "green_block": [0.28, -0.24, 0.025],
    "red_block": [0.24, -0.28, 0.025],
}
BLOCK_MONITOR = Path(__file__).resolve().parent.parent / "ros2_ur_ws/scripts/watch_pallet_block.py"
STACK_PLACE_YAWS = (0.0, 0.7853981633974483, -0.7853981633974483)


def block_pose_topic(object_id: str) -> str:
    return ("/isaac_calibration_block_pose" if object_id == "red_block"
            else f"/isaac_{object_id}_pose")


def start_motion_monitor(args, object_id: str, target: list[float], output_path: Path,
                         log_path: Path):
    if not getattr(args, "motion_monitor", False):
        return None, None
    command = [
        "/usr/bin/python3", str(getattr(args, "block_monitor", BLOCK_MONITOR)),
        "--pose-topic", block_pose_topic(object_id),
        "--seconds", str(getattr(args, "motion_monitor_seconds", 300.0)),
        "--target", *(str(value) for value in target),
        "--slip-threshold", str(getattr(args, "slip_threshold", 0.006)),
        "--slip-confirm-samples", "3", "--output", str(output_path),
        "--supervised", "--monitor-id", uuid.uuid4().hex,
    ]
    stream = log_path.open("w", encoding="utf-8")
    try:
        process = subprocess.Popen(
            command, cwd=args.repository_dir, stdout=stream,
            stderr=subprocess.STDOUT,
        )
        process.monitor_id = command[-1]
    except Exception:
        stream.close()
        raise
    return process, stream


def stop_motion_monitor(process, stream, output_path: Path) -> dict | None:
    if process is None:
        return None
    alive_at_stop = process.poll() is None
    try:
        if alive_at_stop:
            process.send_signal(signal.SIGINT)
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
    finally:
        stream.close()
    try:
        payload = json.loads(output_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    summary = payload.get("summary") if isinstance(payload, dict) else None
    if (not isinstance(summary, dict) or not alive_at_stop
            or process.returncode not in (0, 1)):
        return None
    monitoring = summary.get("monitoring", {})
    if (not isinstance(monitoring, dict) or not getattr(process, "monitor_id", None)
            or monitoring.get("monitor_id") != process.monitor_id
            or monitoring.get("supervised") is not True
            or monitoring.get("end_reason") != "supervisor_stop"):
        return None
    return summary


def request_monitor_safety_stop(args, log):
    safety_script = Path(__file__).with_name("pallet_monitor_safety.py")
    try:
        stop = subprocess.run(["/usr/bin/python3", str(safety_script)],
                              cwd=args.repository_dir, stdout=log,
                              stderr=subprocess.STDOUT, check=False, timeout=4.0)
        return stop.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def stop_command_group(process, grace_seconds=2.0):
    """Bounded cleanup of the exact session created for this command."""
    if process is None:
        return True
    for stop_signal in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, stop_signal)
        except ProcessLookupError:
            process.wait(timeout=1)
            return True
        deadline = time.monotonic() + grace_seconds
        while True:
            process.poll()  # Reap the leader, but still check its children.
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                return True
            if time.monotonic() >= deadline:
                break
            time.sleep(0.1)
    return False


def run_supervised_command(invocation, args, log, monitor_process):
    """Stop execution if its safety monitor disappears; never retry that run."""
    command_environment = os.environ.copy()
    output_dir = getattr(args, 'output_dir', None)
    if output_dir is not None:
        command_environment['MTC_LAUNCH_LOG_DIR'] = str((Path(output_dir) / 'ros').resolve())
    if monitor_process is None:
        return subprocess.run(invocation, cwd=args.repository_dir, stdout=log,
                              stderr=subprocess.STDOUT, check=False,
                              env=command_environment), None
    process = None
    try:
        if monitor_process.poll() is None:
            process = subprocess.Popen(invocation, cwd=args.repository_dir, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True,
                                       env=command_environment)
        while process is not None:
            if monitor_process.poll() is not None:
                break
            code = process.poll()
            if code is not None:
                return subprocess.CompletedProcess(invocation, code), None
            time.sleep(0.1)
        acknowledgement = request_monitor_safety_stop(args, log)
        cleaned = stop_command_group(process)
    except BaseException:
        try:
            request_monitor_safety_stop(args, log)
        finally:
            stop_command_group(process)
            if monitor_process.poll() is None:
                monitor_process.send_signal(signal.SIGINT)
        raise
    detail = "acknowledged" if acknowledgement else "not acknowledged"
    error = f"motion monitor terminated early; safety stop {detail}"
    if not cleaned:
        error += "; command group cleanup unconfirmed"
    return subprocess.CompletedProcess(invocation, 2), error


def speed_failure_class(runs: list[dict]) -> str | None:
    """Classify only failures that justify a fresh lower-speed simulation."""
    if not runs or all(run.get("passed") is True for run in runs):
        return None
    failed = runs[-1]
    if failed.get("monitoring_failure"):
        return "monitoring"
    # Return code 1 is emitted before robot execution for validation,
    # planning, or startup failures. The concurrently started motion monitor
    # naturally reports no release in that case; that is not a dynamics fault.
    if failed.get("returncode") == 1:
        return "planning_or_startup"
    if failed.get("returncode") == 2:
        return "execution_dynamics"
    if failed.get("error") == "motion monitor result unavailable":
        return "monitoring"
    monitor = failed.get("motion_summary")
    if isinstance(monitor, dict) and (
            monitor.get("slip_detected") is True
            or monitor.get("runtime_fault_detected") is True
            or monitor.get("safety_event_published") is True
            or monitor.get("release_detected") is not True
            or monitor.get("post_release_recontact_detected") is True):
        return "execution_dynamics"
    if failed.get("error") == "fresh stable camera detection unavailable":
        return "vision"
    return "unknown"


def fresh_detected_objects(path: Path, max_age: float, now=None) -> set[str]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return set()
    if not isinstance(payload, dict):
        return set()
    timestamp = time.time() if now is None else now
    updated_at = payload.get("updated_at")
    if (payload.get("frame") != "base_footprint" or payload.get("unit") != "m"
            or type(updated_at) not in (int, float) or not math.isfinite(updated_at)
            or abs(timestamp - updated_at) > max_age
            or not isinstance(payload.get("objects"), dict)):
        return set()
    return set(payload["objects"])


def wait_for_objects(path: Path, object_ids: set[str], timeout: float, max_age: float,
                     stable_samples: int = 3) -> bool:
    """Require one fresh scene to contain every object for consecutive polls."""
    if stable_samples <= 0:
        raise ValueError("stable_samples must be positive")
    deadline = time.monotonic() + timeout
    consecutive = 0
    while time.monotonic() < deadline:
        if object_ids.issubset(fresh_detected_objects(path, max_age)):
            consecutive += 1
            if consecutive >= stable_samples:
                return True
        else:
            consecutive = 0
        time.sleep(0.2)
    return False


def job_tasks(args) -> list[dict]:
    tasks = getattr(args, "tasks", None)
    if tasks is not None:
        return tasks
    return [
        {"action": "pick_place", "pick_object": object_id, "place_xyz": target}
        for object_id, target in args.placements.items()
    ]


def resolve_task_target(args, task: dict) -> list[float]:
    if "place_xyz" in task:
        return list(task["place_xyz"])
    manager = load_pallet_scene(
        args.scene_file,
        max_age=args.vision_max_age,
    )
    target = manager.resolve_target_position(task)
    return [float(value) for value in target]


def validate_object_place_yaws(entries, tasks):
    """Execution-layer radian tuning, separate from declarative task intent."""
    allowed = {task['pick_object'] for task in tasks}
    result = {}
    for entry in entries or []:
        if not isinstance(entry, (list, tuple)) or len(entry) != 2:
            raise ValueError('object place yaw requires object and radians')
        name, value = entry
        if not isinstance(name, str) or name not in allowed or name in result:
            raise ValueError('object place yaw requires a unique object in this job')
        if isinstance(value, bool):
            raise ValueError('object place yaw must be finite radians within [-pi,pi]')
        try:
            angle = float(value)
        except (ValueError, TypeError) as error:
            raise ValueError('object place yaw must be finite radians within [-pi,pi]') from error
        if not math.isfinite(angle) or not -math.pi <= angle <= math.pi:
            raise ValueError('object place yaw must be finite radians within [-pi,pi]')
        result[name] = angle
    return result


def run_job(args) -> dict:
    tasks = job_tasks(args)
    object_place_yaws = getattr(args, 'object_place_yaws', {})
    if not isinstance(object_place_yaws, dict):
        raise ValueError('object place yaws must be a validated map')
    object_place_yaws = validate_object_place_yaws(list(object_place_yaws.items()), tasks)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    runs = []
    for index, task in enumerate(tasks, start=1):
        object_id = task["pick_object"]
        required_objects = {object_id}
        if getattr(args, 'scene_obstacles', False):
            required_objects.update(DEFAULT_PLACEMENTS)
        if "target_object" in task:
            required_objects.add(task["target_object"])
        if not wait_for_objects(
                args.scene_file, required_objects, args.detection_timeout,
                args.vision_max_age):
            runs.append({
                "object_id": object_id, "target": None, "passed": False,
                "error": "fresh stable camera detection unavailable",
            })
            break
        try:
            target = resolve_task_target(args, task)
        except ValueError as error:
            runs.append({
                "object_id": object_id, "target": None, "passed": False,
                "error": f"relative target resolution failed: {error}",
            })
            break
        command = dict(task)
        started = time.monotonic()
        motion_summary = None
        monitoring_failed = False
        attempts = []
        retry_error = None
        monitoring_error = None
        planning_attempts = max(1, int(getattr(args, "planning_attempts", 3)))
        for attempt in range(1, planning_attempts + 1):
            # Planning can take long enough for the camera scene to change.
            # Never start another planner against a missing/stale object.
            if attempt > 1 and not wait_for_objects(
                    args.scene_file, required_objects, args.detection_timeout,
                    args.vision_max_age):
                retry_error = "fresh stable camera detection unavailable before retry"
                break
            log_path = args.output_dir / f"{index:02d}-{object_id}-attempt-{attempt:02d}.log"
            task_state = args.output_dir / f"{index:02d}-{object_id}-attempt-{attempt:02d}-task.json"
            invocation = [
                sys.executable, str(args.command_runner),
                "--robot-profile", "pallet_gripper",
                "--pallet-execution-mode", "standard",
                "--pallet-scene-file", str(args.scene_file),
                "--vision-max-age", str(args.vision_max_age),
                "--move-to-pick-timeout", "10.0",
                "--move-to-place-timeout", "10.0",
                "--return-home-timeout", "5.0",
                "--task-state-file", str(task_state),
                "--task-id", f"{args.job_id}-{index:02d}-{object_id}-attempt-{attempt:02d}",
                "--arm-velocity-scaling", str(getattr(args, "arm_velocity_scaling", 1.0)),
                "--arm-acceleration-scaling", str(getattr(args, "arm_acceleration_scaling", 1.0)),
                "--cartesian-velocity-scaling", str(
                    getattr(args, "cartesian_velocity_scaling", 1.0)),
                "--cartesian-acceleration-scaling", str(
                    getattr(args, "cartesian_acceleration_scaling", 1.0)),
            ]
            gripper_close_position = getattr(args, "gripper_close_position", None)
            if gripper_close_position is not None:
                invocation.extend([
                    "--gripper-close-position", str(gripper_close_position),
                ])
            grasp_angle_delta = getattr(args, "grasp_angle_delta", None)
            release_open_position = getattr(args, 'release_open_position', None)
            if release_open_position is not None:
                invocation.extend(['--release-open-position', str(release_open_position)])
            if grasp_angle_delta is not None:
                invocation.extend(["--grasp-angle-delta", str(grasp_angle_delta)])
            if getattr(args, "cartesian_transfer", False):
                invocation.append("--cartesian-transfer")
            if getattr(args, "upright_transfer", False):
                invocation.append("--upright-transfer")
            if getattr(args, 'scene_obstacles', False):
                invocation.append('--scene-obstacles')
            transfer_height = getattr(args, "transfer_height", None)
            if transfer_height is not None:
                invocation.extend(["--transfer-height", str(transfer_height)])
            place_yaw = object_place_yaws.get(object_id, getattr(args, "place_yaw", None))
            if place_yaw is not None:
                invocation.extend(["--place-yaw", str(place_yaw)])
            elif command.get("relation") == "on":
                place_yaw = STACK_PLACE_YAWS[(attempt - 1) % len(STACK_PLACE_YAWS)]
                invocation.extend(["--place-yaw", str(place_yaw)])
            invocation.extend(["--once", json.dumps(command, separators=(",", ":"))])
            totg_path_tolerance = getattr(args, 'totg_path_tolerance', None)
            if totg_path_tolerance is not None:
                invocation.extend(['--totg-path-tolerance', str(totg_path_tolerance)])
            # A pre-motion retry gets its own complete monitoring lifetime.
            # Preserve the legacy first-attempt path and never overwrite its
            # evidence with a later attempt's measurements.
            prefix = f"{index:02d}-{object_id}"
            if attempt > 1:
                prefix += f"-attempt-{attempt:02d}"
            monitor_output = args.output_dir / f"{prefix}-motion.json"
            monitor_log = args.output_dir / f"{prefix}-motion.log"
            with log_path.open("w", encoding="utf-8") as log:
                monitor_process, monitor_stream = start_motion_monitor(
                    args, object_id, target, monitor_output, monitor_log)
                try:
                    if monitor_process is not None:
                        time.sleep(0.5)
                    completed, monitoring_error = run_supervised_command(
                        invocation, args, log, monitor_process)
                finally:
                    motion_summary = stop_motion_monitor(
                        monitor_process, monitor_stream, monitor_output)
            monitoring_failed = (
                getattr(args, "motion_monitor", False) and motion_summary is None)
            monitor_safety_failed = isinstance(motion_summary, dict) and any(
                motion_summary.get(key) is True for key in (
                    "slip_detected", "runtime_fault_detected", "safety_event_published"))
            attempts.append({
                "attempt": attempt, "returncode": completed.returncode,
                "log": str(log_path), "task_state": str(task_state),
                "place_yaw": place_yaw,
                "motion_output": str(monitor_output),
                "motion_summary": motion_summary,
            })
            # Code 1 means planning/startup failed before robot motion.  It is
            # safe to retry.  Code 2 requires operator recovery and must stop.
            if completed.returncode != 1 or monitoring_failed or monitor_safety_failed:
                break
        dynamics_failed = completed.returncode == 0 and isinstance(motion_summary, dict) and (
            motion_summary.get("placement_pass") is not True
            or motion_summary.get("fresh_feedback") is not True
            or motion_summary.get("slip_detected") is True
            or motion_summary.get("runtime_fault_detected") is True
            or motion_summary.get("safety_event_published") is True
            or motion_summary.get("release_detected") is not True
            or motion_summary.get("post_release_recontact_detected") is True)
        run = {
            "object_id": object_id, "target": target,
            "returncode": completed.returncode,
            "elapsed_seconds": time.monotonic() - started,
            "passed": completed.returncode == 0 and not dynamics_failed and not monitoring_failed,
            "log": str(log_path), "task_state": str(task_state),
            "motion_summary": motion_summary,
            "motion_output": str(monitor_output),
            "attempts": attempts,
        }
        if monitoring_failed:
            run["error"] = "motion monitor result unavailable"
            run["monitoring_failure"] = True
        if monitoring_error is not None:
            run["error"] = monitoring_error
            run["monitoring_failure"] = True
        if retry_error is not None:
            run["retry_blocked"] = retry_error
        runs.append(run)
        if not run["passed"]:
            break
    report = {
        "job_id": args.job_id,
        "created_at": time.time(),
        "requested_count": len(tasks),
        "completed_count": sum(run.get("passed") is True for run in runs),
        "passed": len(runs) == len(tasks) and all(
            run.get("passed") is True for run in runs),
        "failure_class": speed_failure_class(runs),
        "motion_scaling": {
            "arm_velocity": getattr(args, "arm_velocity_scaling", 1.0),
            "arm_acceleration": getattr(args, "arm_acceleration_scaling", 1.0),
            "cartesian_velocity": getattr(args, "cartesian_velocity_scaling", 1.0),
            "cartesian_acceleration": getattr(args, "cartesian_acceleration_scaling", 1.0),
        },
        "planning_tuning": {
            "object_place_yaws_rad": object_place_yaws,
            "totg_path_tolerance_rad": getattr(args, 'totg_path_tolerance', None),
            "scene_obstacles": bool(getattr(args, 'scene_obstacles', False)),
            "release_open_position_m": getattr(args, 'release_open_position', None),
        },
        "runs": runs,
    }
    (args.output_dir / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    repository_dir = Path(__file__).resolve().parent.parent
    parser.add_argument("--scene-file", type=Path, default=Path("/tmp/pallet_vision_scene.json"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument('--validate-only', action='store_true',
                        help='Validate job options without creating output or starting motion.')
    parser.add_argument("--job-id", default=f"palletize-{int(time.time())}")
    parser.add_argument("--detection-timeout", type=float, default=30.0)
    parser.add_argument("--vision-max-age", type=float, default=5.0)
    parser.add_argument("--planning-attempts", type=int, default=3)
    parser.add_argument('--object-place-yaw', nargs=2, action='append',
                        metavar=('OBJECT', 'RAD'), help='Optional per-object execution yaw override.')
    parser.add_argument('--totg-path-tolerance', type=float,
                        help='Optional Pallet TOTG path deviation experiment in rad.')
    parser.add_argument("--disable-motion-monitor", action="store_true")
    parser.add_argument("--motion-monitor-seconds", type=float, default=300.0)
    parser.add_argument("--slip-threshold", type=float, default=0.006)
    parser.add_argument("--arm-velocity-scaling", type=float, default=1.0)
    parser.add_argument("--arm-acceleration-scaling", type=float, default=1.0)
    parser.add_argument("--cartesian-velocity-scaling", type=float, default=1.0)
    parser.add_argument("--cartesian-acceleration-scaling", type=float, default=1.0)
    parser.add_argument(
        "--gripper-close-position", type=float,
        help="Optional Pallet jaw target in metres for controlled preload experiments")
    parser.add_argument('--release-open-position', type=float,
                        help='Optional final release travel per jaw in metres; pre-grasp unchanged.')
    parser.add_argument(
        "--place-yaw", type=float,
        help="Optional fixed object placement yaw in radians")
    parser.add_argument(
        "--grasp-angle-delta", type=float,
        help="Optional generated grasp angular spacing in radians")
    parser.add_argument(
        "--cartesian-transfer", action="store_true",
        help="Use a straight Cartesian loaded transfer to reduce swing and tilt")
    parser.add_argument(
        "--upright-transfer", action="store_true",
        help="Constrain loaded-transfer roll/pitch while allowing yaw")
    parser.add_argument(
        "--transfer-height", type=float,
        help="Optional common lift/lower distance in metres for transfer clearance")
    parser.add_argument("--task-spec", type=Path,
                        help="Versioned declarative pick/place job JSON")
    parser.add_argument('--scene-obstacles', action='store_true',
                        help='Require fresh three-block scene and include non-picked collision obstacles.')
    parser.add_argument(
        "--command-runner", type=Path,
        default=repository_dir / "ur10e/mtc_command_runner.py")
    args = parser.parse_args()
    args.motion_monitor = not args.disable_motion_monitor
    args.block_monitor = BLOCK_MONITOR
    for name in (
            "arm_velocity_scaling", "arm_acceleration_scaling",
            "cartesian_velocity_scaling", "cartesian_acceleration_scaling"):
        value = getattr(args, name)
        if not 0.0 < value <= 1.0:
            parser.error(f"--{name.replace('_', '-')} must be within (0, 1]")
    args.repository_dir = repository_dir
    if args.totg_path_tolerance is not None and (
            not math.isfinite(args.totg_path_tolerance) or not 0 < args.totg_path_tolerance <= 0.1):
        parser.error('--totg-path-tolerance must be finite and within (0, 0.1] rad')
    for name in ("motion_monitor_seconds", "detection_timeout", "vision_max_age"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    if (args.gripper_close_position is not None and
            not 0.0 <= args.gripper_close_position <= 0.0335):
        parser.error("--gripper-close-position must be within [0, 0.0335]")
    if args.release_open_position is not None and (
            not math.isfinite(args.release_open_position) or not 0 < args.release_open_position <= 0.0335):
        parser.error('--release-open-position must be finite and within (0, 0.0335] m')
    if args.place_yaw is not None and not math.isfinite(args.place_yaw):
        parser.error("--place-yaw must be finite")
    if (args.grasp_angle_delta is not None and
            (not math.isfinite(args.grasp_angle_delta) or
             not 0.0 < args.grasp_angle_delta <= 2.0 * math.pi)):
        parser.error("--grasp-angle-delta must be within (0, 2*pi]")
    if (args.transfer_height is not None and
            (not math.isfinite(args.transfer_height) or
             not 0.0 < args.transfer_height <= 0.40)):
        parser.error("--transfer-height must be within (0, 0.40]")
    if not 0.0 < args.slip_threshold < 1.0:
        parser.error("--slip-threshold must be within (0, 1)")
    if args.task_spec:
        args.tasks = load_tasks(args.task_spec)
        args.placements = None
    else:
        args.tasks = None
        args.placements = DEFAULT_PLACEMENTS
    try:
        args.object_place_yaws = validate_object_place_yaws(args.object_place_yaw, job_tasks(args))
    except ValueError as error:
        parser.error(str(error))
    if args.validate_only:
        return 0
    report = run_job(args)
    print(json.dumps(report, ensure_ascii=False), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
