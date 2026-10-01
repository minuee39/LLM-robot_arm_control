#!/usr/bin/env python3
"""Run repeatable Pallet MTC/Isaac scenarios and record physical outcomes."""

import argparse
import datetime as dt
import json
import math
import os
from pathlib import Path
import re
import signal
import statistics
import subprocess
import sys
import tempfile
import time

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
ROS_WORKSPACE = PROJECT_ROOT / 'ros2_ur_ws'
MONITOR = ROS_WORKSPACE / 'scripts/watch_pallet_block.py'
PALLET_CONFIG = ROS_WORKSPACE / 'src/pallet_gripper_moveit_config'
sys.path.insert(0, str(PALLET_CONFIG / 'scripts'))
from workcell_scenario import block_scenario

WORKCELL = yaml.safe_load((PALLET_CONFIG / 'config/workcell.yaml').read_text(encoding='utf-8'))
READY_TEXT = 'Pallet workcell ready for direct-pose MTC execution.'
TASK_COMPLETED_TEXT = 'Task execution completed'

DEFAULT_SCENARIOS = {
    'baseline': {'center_xy': [-0.30, 0.30], 'size': [0.030, 0.030, 0.050], 'mass_kg': 0.050},
    'offset': {'center_xy': [-0.27, 0.28], 'size': [0.030, 0.030, 0.050], 'mass_kg': 0.050},
    'narrow': {'center_xy': [-0.30, 0.30], 'size': [0.030, 0.025, 0.050], 'mass_kg': 0.050},
    'mass_065': {'center_xy': [-0.27, 0.28], 'size': [0.030, 0.030, 0.050], 'mass_kg': 0.065},
}

SCENARIO_SCALING_PARAMETERS = (
    'arm_velocity_scaling', 'arm_acceleration_scaling',
    'cartesian_velocity_scaling', 'cartesian_acceleration_scaling',
)


def validate_scenario(name, raw):
    center = tuple(raw.get('center_xy', ()))
    size = tuple(raw.get('size', ()))
    mass = raw.get('mass_kg')
    if not name or len(center) != 2 or len(size) != 3:
        raise ValueError(f'{name or "scenario"}: center_xy needs 2 values and size needs 3 values')
    try:
        scenario = block_scenario(WORKCELL, center, size, mass)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name}: {exc}') from exc
    result = {
        'name': name, 'center_xy': scenario['center_xy_base'],
        'size': scenario['size'], 'mass_kg': float(scenario['mass_kg']),
    }
    gripper_close = raw.get('gripper_close_position')
    if gripper_close is not None:
        gripper_close = float(gripper_close)
        if not math.isfinite(gripper_close) or not 0.0 <= gripper_close <= 0.0335:
            raise ValueError(f'{name}: gripper_close_position must be within 0..0.0335 m')
        result['gripper_close_position'] = gripper_close
    for parameter in SCENARIO_SCALING_PARAMETERS:
        value = raw.get(parameter)
        if value is None:
            continue
        value = float(value)
        if not math.isfinite(value) or not 0.0 < value <= 1.0:
            raise ValueError(f'{name}: {parameter} must be within (0, 1]')
        result[parameter] = value
    return result


def scenario_commands(scenario, result_file, monitor_seconds, slip_threshold=0.005,
                      slip_confirm_samples=3, headless=True,
                      safety_cancel_enabled=False,
                      inject_safety_event_after_contact=-1.0,
                      event_file=None, inject_safety_code='SLIP_DETECTED',
                      drop_clearance_threshold=0.005,
                      tilt_threshold_deg=20.0, fault_confirm_samples=3,
                      pose_stream_timeout=0.75,
                      inject_sim_fault_phase='none',
                      inject_sim_fault_code='BLOCK_DROP_DETECTED',
                      inject_sim_fault_delay=0.0):
    center = scenario['center_xy']
    size = scenario['size']
    target = [0.30, -0.30, size[2] / 2]
    simulation = [
        'ros2', 'launch', 'mtc_tutorial', 'pallet_mtc_isaac_demo.launch.py',
        f'headless:={str(headless).lower()}', 'use_rviz:=false', 'startup_timeout:=180',
        f'block_x:={center[0]:.6f}', f'block_y:={center[1]:.6f}',
        f'block_size_x:={size[0]:.6f}', f'block_size_y:={size[1]:.6f}',
        f'block_size_z:={size[2]:.6f}', f'block_mass_kg:={scenario["mass_kg"]:.6f}',
        f'safety_cancel_enabled:={str(safety_cancel_enabled).lower()}',
        f'fault_injection_enabled:={str(inject_sim_fault_phase != "none").lower()}',
    ]
    monitor = [
        '/usr/bin/python3', str(MONITOR), '--seconds', str(monitor_seconds),
        '--target', *(f'{value:.6f}' for value in target),
        '--block-size', *(f'{value:.6f}' for value in size),
        '--slip-threshold', f'{slip_threshold:.6f}',
        '--slip-confirm-samples', str(slip_confirm_samples),
        '--drop-clearance-threshold', f'{drop_clearance_threshold:.6f}',
        '--tilt-threshold-deg', f'{tilt_threshold_deg:.6f}',
        '--fault-confirm-samples', str(fault_confirm_samples),
        '--pose-stream-timeout', f'{pose_stream_timeout:.6f}',
        '--output', str(result_file),
    ]
    if event_file is not None:
        monitor.extend(['--event-output', str(event_file)])
    if inject_safety_event_after_contact >= 0.0:
        monitor.extend([
            '--inject-safety-event-after-contact',
            f'{inject_safety_event_after_contact:.6f}',
            '--inject-safety-code', inject_safety_code,
        ])
    if inject_sim_fault_phase != 'none':
        monitor.extend([
            '--inject-sim-fault-phase', inject_sim_fault_phase,
            '--inject-sim-fault-code', inject_sim_fault_code,
            '--inject-sim-fault-delay', f'{inject_sim_fault_delay:.6f}',
        ])
    task = [
        'ros2', 'launch', 'mtc_tutorial', 'pick_place_demo.launch.py',
        'robot_profile:=pallet_gripper', 'execution_mode:=standard',
        f'object_x:={center[0]:.6f}', f'object_y:={center[1]:.6f}',
        f'object_z:={size[2] / 2:.6f}',
        f'object_size_x:={size[0]:.6f}', f'object_size_y:={size[1]:.6f}',
        f'object_size_z:={size[2]:.6f}', f'place_z:={target[2]:.6f}',
    ]
    if 'gripper_close_position' in scenario:
        close = scenario['gripper_close_position']
        task.extend([
            f'gripper_close_min:={close:.6f}',
            f'gripper_close_max:={close:.6f}',
        ])
    for parameter in SCENARIO_SCALING_PARAMETERS:
        if parameter in scenario:
            task.append(f'{parameter}:={scenario[parameter]:.6f}')
    return simulation, monitor, task


def stop_process(process, timeout=20):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                return
            process.wait(timeout=5)


def process_group_alive(process):
    if process is None:
        return False
    try:
        os.killpg(process.pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def wait_for_text(process, log_path, expected, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f'simulation exited with code {process.returncode}; see {log_path}')
        if log_path.exists() and expected in log_path.read_text(encoding='utf-8', errors='replace'):
            return
        time.sleep(0.5)
    raise TimeoutError(f'simulation readiness timed out after {timeout}s; see {log_path}')


def read_summary(path):
    payload = json.loads(path.read_text(encoding='utf-8'))
    summary = payload.get('summary')
    if not isinstance(summary, dict):
        raise ValueError(f'missing summary in {path}')
    return summary


def task_completed(log_path):
    """Return whether the MTC child reported successful task execution.

    ``ros2 launch`` can exit with code zero even when a launched node exits with
    code one, so its wrapper return code alone is not a reliable success signal.
    """
    try:
        return TASK_COMPLETED_TEXT in log_path.read_text(encoding='utf-8', errors='replace')
    except OSError:
        return False


def planning_seconds(log_path):
    try:
        text = log_path.read_text(encoding='utf-8', errors='replace')
    except OSError:
        return None
    match = re.search(r'Task planning (?:failed after|completed in) ([0-9.]+) s', text)
    return float(match.group(1)) if match else None


def execution_seconds(log_path):
    try:
        text = log_path.read_text(encoding='utf-8', errors='replace')
    except OSError:
        return None
    match = re.search(r'Task execution (?:failed after|completed in) ([0-9.]+) s', text)
    return float(match.group(1)) if match else None


def classify_failure(run):
    """Return a stable primary failure category for matrix triage."""
    if run.get('passed') is True:
        return None
    error = str(run.get('error') or '').lower()
    summary = run.get('summary') if isinstance(run.get('summary'), dict) else {}
    if 'readiness timed out' in error or 'simulation exited' in error:
        return 'simulation_startup'
    if 'task timed out' in error or run.get('task_returncode') == 124:
        return 'task_timeout'
    if not run.get('process_cleanup_ok', True):
        return 'process_cleanup'
    if summary.get('safety_event_published') is True and not run.get('safety_stop_expected'):
        return 'safety_false_positive'
    if not run.get('task_execution_completed'):
        return 'planning_or_execution'
    if not summary:
        return 'monitor_result_missing'
    if summary.get('slip_detected') is True:
        return 'slip'
    if summary.get('release_detected') is not True:
        return 'release_not_detected'
    if summary.get('post_release_recontact_detected') is True:
        return 'post_release_recontact'
    clearance = summary.get('transfer_min_bottom_clearance')
    if (isinstance(clearance, (int, float)) and
            clearance < run.get('min_clearance_required', 0.0)):
        return 'transfer_clearance'
    if summary.get('placement_pass') is not True:
        return 'placement'
    if run.get('safety_stop_expected'):
        return 'safety_recovery'
    return 'unknown'


def write_json_atomic(path, payload):
    """Keep the latest long-run report readable if the process is interrupted."""
    path = Path(path)
    with tempfile.NamedTemporaryFile(
            mode='w', encoding='utf-8', dir=path.parent,
            prefix=f'.{path.name}.', suffix='.tmp', delete=False) as output:
        temporary = Path(output.name)
        json.dump(payload, output, indent=2)
        output.write('\n')
    os.replace(temporary, path)


def numeric_statistics(values):
    finite_values = [float(value) for value in values if math.isfinite(float(value))]
    if not finite_values:
        return None
    return {
        'count': len(finite_values),
        'mean': statistics.fmean(finite_values),
        'stddev': statistics.pstdev(finite_values),
        'min': min(finite_values),
        'max': max(finite_values),
    }


def summarize_runs(runs):
    """Aggregate success and physical metrics for each scenario."""
    grouped = {}
    for run in runs:
        scenario = run.get('scenario') or {}
        name = scenario.get('name')
        if name:
            grouped.setdefault(name, []).append(run)

    result = {}
    summary_metrics = (
        'transfer_min_bottom_clearance', 'final_xy_error', 'final_height_error',
        'max_height', 'long_axis_vertical_cosine',
        'max_contact_relative_displacement_m', 'max_loaded_jaw_asymmetry_m',
        'max_post_release_displacement_m')
    for name, scenario_runs in grouped.items():
        success_count = sum(run.get('passed') is True for run in scenario_runs)
        successful_runs = [run for run in scenario_runs if run.get('passed') is True]
        scenario_result = {
            'run_count': len(scenario_runs),
            'success_count': success_count,
            'failure_count': len(scenario_runs) - success_count,
            'success_rate': success_count / len(scenario_runs),
            'elapsed_seconds': numeric_statistics(
                run['elapsed_seconds'] for run in scenario_runs
                if isinstance(run.get('elapsed_seconds'), (int, float))),
            'task_planning_seconds': numeric_statistics(
                run['task_planning_seconds'] for run in scenario_runs
                if isinstance(run.get('task_planning_seconds'), (int, float))),
            'task_execution_seconds': numeric_statistics(
                run['task_execution_seconds'] for run in scenario_runs
                if isinstance(run.get('task_execution_seconds'), (int, float))),
            'safety_false_positive_count': sum(
                classify_failure(run) == 'safety_false_positive'
                for run in scenario_runs),
            'process_cleanup_failure_count': sum(
                run.get('process_cleanup_ok') is False for run in scenario_runs),
            'failure_categories': {},
        }
        for run in scenario_runs:
            category = classify_failure(run)
            if category:
                scenario_result['failure_categories'][category] = (
                    scenario_result['failure_categories'].get(category, 0) + 1)
        for metric in summary_metrics:
            scenario_result[metric] = numeric_statistics(
                run['summary'][metric] for run in successful_runs
                if isinstance(run.get('summary'), dict) and
                isinstance(run['summary'].get(metric), (int, float)))
        result[name] = scenario_result
    return result


def summarize_matrix(runs):
    completed = [run for run in runs if run.get('status') != 'dry_run']
    success_count = sum(run.get('passed') is True for run in completed)
    return {
        'run_count': len(completed),
        'success_count': success_count,
        'failure_count': len(completed) - success_count,
        'success_rate': success_count / len(completed) if completed else None,
        'safety_false_positive_count': sum(
            classify_failure(run) == 'safety_false_positive' for run in completed),
        'process_cleanup_failure_count': sum(
            run.get('process_cleanup_ok') is False for run in completed),
        'task_planning_seconds': numeric_statistics(
            run['task_planning_seconds'] for run in completed
            if isinstance(run.get('task_planning_seconds'), (int, float))),
        'task_execution_seconds': numeric_statistics(
            run['task_execution_seconds'] for run in completed
            if isinstance(run.get('task_execution_seconds'), (int, float))),
        'elapsed_seconds': numeric_statistics(
            run['elapsed_seconds'] for run in completed
            if isinstance(run.get('elapsed_seconds'), (int, float))),
    }


def recovery_candidates(scenario, requested=None):
    """Return bounded base-frame retreat candidates in deterministic order."""
    source = scenario['center_xy']
    target = (0.30, -0.30)
    reverse = (source[0] - target[0], source[1] - target[1])
    norm = math.hypot(*reverse)
    reverse = (reverse[0] / norm, reverse[1] / norm)
    candidates = {
        'vertical': (0.0, 0.0, 1.0),
        'reverse_transfer': (reverse[0], reverse[1], 0.5),
        'lateral_left': (-reverse[1], reverse[0], 0.5),
        'lateral_right': (reverse[1], -reverse[0], 0.5),
    }
    names = requested or list(candidates)
    return [(name, candidates[name]) for name in names]


def parse_recovery_diagnostics(candidate, text, returncode):
    """Extract stable, machine-readable recovery failure evidence from MTC logs."""
    planning_match = re.search(
        r'Task planning (?:failed after|completed in) ([0-9.]+) s', text)
    detail_lines = []
    for raw_line in text.splitlines():
        line = re.sub(r'^\[[^]]+\]\s*', '', raw_line).strip()
        if ('adapter' not in line.lower() and any(token in line.lower() for token in (
                'min_fraction not met', 'collision', 'joint bound',
                'joint limit', 'invalid start state'))):
            detail_lines.append(line)
    lowered = '\n'.join(detail_lines).lower()
    if 'min_fraction not met' in lowered:
        classification = 'cartesian_path_incomplete'
    elif 'collision' in lowered:
        classification = 'collision'
    elif 'joint bound' in lowered or 'joint limit' in lowered:
        classification = 'joint_limit'
    elif 'Task planning failed' in text:
        classification = 'planning_failed_unknown'
    else:
        classification = None
    return {
        'candidate': candidate,
        'returncode': returncode,
        'planning_seconds': float(planning_match.group(1)) if planning_match else None,
        'execution_started': 'Executing lowest-cost solution' in text,
        'task_completed': TASK_COMPLETED_TEXT in text,
        'safe_refusal': (
            'Task planning failed' in text and
            'Executing lowest-cost solution' not in text),
        'failure_classification': classification,
        'failure_evidence': detail_lines,
    }


def run_once(scenario, run_dir, args):
    result_path = run_dir / 'block_motion.json'
    event_path = run_dir / 'safety_event.json'
    sim_log = run_dir / 'simulation.log'
    task_log = run_dir / 'task.log'
    recovery_log = run_dir / 'recovery.log'
    monitor_log = run_dir / 'monitor.log'
    simulation_command, monitor_command, task_command = scenario_commands(
        scenario, result_path, args.monitor_seconds, args.slip_threshold,
        args.slip_confirm_samples, args.headless, args.safety_cancel_enabled,
        args.inject_safety_event_after_contact, event_path,
        args.inject_safety_code, args.drop_clearance_threshold,
        args.tilt_threshold_deg, args.fault_confirm_samples,
        args.pose_stream_timeout, args.inject_sim_fault_phase,
        args.inject_sim_fault_code, args.inject_sim_fault_delay)
    if args.dry_run:
        return {
            'scenario': scenario, 'simulation_command': simulation_command,
            'monitor_command': monitor_command, 'task_command': task_command,
            'status': 'dry_run',
        }

    simulation = monitor = task_process = recovery = None
    recovery_returncode = None
    recovery_completed = False
    recovery_safe_refusal = False
    recovery_attempts = []
    fallback_hold_latched = False
    task_returncode = None
    error = None
    started = time.monotonic()
    ros_log_dir = run_dir / 'ros-logs'
    ros_log_dir.mkdir()
    run_environment = os.environ.copy()
    run_environment['ROS_LOG_DIR'] = str(ros_log_dir)
    try:
        with sim_log.open('w', encoding='utf-8') as output:
            simulation = subprocess.Popen(
                simulation_command, cwd=ROS_WORKSPACE, stdout=output,
                stderr=subprocess.STDOUT, start_new_session=True, text=True,
                env=run_environment)
        wait_for_text(simulation, sim_log, READY_TEXT, args.startup_timeout)
        with monitor_log.open('w', encoding='utf-8') as output:
            monitor = subprocess.Popen(
                monitor_command, cwd=PROJECT_ROOT, stdout=output,
                stderr=subprocess.STDOUT, start_new_session=True, text=True,
                env=run_environment)
        with task_log.open('w', encoding='utf-8') as output:
            task_process = subprocess.Popen(
                task_command, cwd=ROS_WORKSPACE, stdout=output,
                stderr=subprocess.STDOUT, start_new_session=True, text=True,
                env=run_environment)
            try:
                task_returncode = task_process.wait(timeout=args.task_timeout)
            except subprocess.TimeoutExpired:
                task_returncode = 124
                error = f'MTC task timed out after {args.task_timeout}s'
                stop_process(task_process)
        if (args.expect_safety_stop and event_path.exists() and
                not task_completed(task_log)):
            reset_output = []
            for _attempt in range(5):
                reset = subprocess.run(
                    ['ros2', 'service', 'call', '/pallet/reset_safety_stop',
                     'std_srvs/srv/Trigger', '{}'],
                    cwd=ROS_WORKSPACE, env=run_environment, text=True,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=15)
                reset_output.append(reset.stdout)
                if 'success=True' in reset.stdout or 'success: true' in reset.stdout:
                    break
                time.sleep(0.5)
            recovery_log.write_text(''.join(reset_output), encoding='utf-8')
            for candidate, direction in recovery_candidates(
                    scenario, args.recovery_candidate):
                attempt_log = run_dir / f'recovery-{candidate}.log'
                candidate_command = [
                    *task_command, 'recovery_only:=true',
                    f'recovery_candidate_id:={candidate}',
                    f'recovery_direction_x:={direction[0]:.9f}',
                    f'recovery_direction_y:={direction[1]:.9f}',
                    f'recovery_direction_z:={direction[2]:.9f}',
                ]
                with attempt_log.open('w', encoding='utf-8') as output:
                    recovery = subprocess.Popen(
                        candidate_command, cwd=ROS_WORKSPACE,
                        stdout=output, stderr=subprocess.STDOUT,
                        start_new_session=True, text=True, env=run_environment)
                    try:
                        recovery_returncode = recovery.wait(timeout=args.recovery_timeout)
                    except subprocess.TimeoutExpired:
                        recovery_returncode = 124
                        stop_process(recovery)
                attempt_text = attempt_log.read_text(encoding='utf-8', errors='replace')
                attempt = parse_recovery_diagnostics(
                    candidate, attempt_text, recovery_returncode)
                attempt['direction_base'] = list(direction)
                attempt['log'] = str(attempt_log)
                recovery_attempts.append(attempt)
                with recovery_log.open('a', encoding='utf-8') as output:
                    output.write(f'\n--- recovery candidate {candidate} ---\n')
                    output.write(attempt_text)
                if attempt['task_completed']:
                    recovery_completed = True
                    break
                if not attempt['safe_refusal']:
                    break
            recovery_safe_refusal = bool(recovery_attempts) and all(
                attempt['safe_refusal'] for attempt in recovery_attempts)
            if not recovery_completed:
                event = json.loads(event_path.read_text(encoding='utf-8'))
                event['event_id'] = f'{event.get("event_id", "safety")}-recovery-fallback'
                event['published_at_unix'] = time.time()
                event['recovery_failed'] = True
                payload = json.dumps(event, separators=(',', ':'))
                message = "{data: '" + payload.replace("'", "''") + "'}"
                relatch = subprocess.run(
                    ['ros2', 'topic', 'pub', '--once',
                     '--qos-durability', 'transient_local',
                     '--qos-reliability', 'reliable',
                     '/pallet/safety_events', 'std_msgs/msg/String', message],
                    cwd=ROS_WORKSPACE, env=run_environment, text=True,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=15)
                fallback_hold_latched = relatch.returncode == 0
                with recovery_log.open('a', encoding='utf-8') as output:
                    output.write('\n--- fallback safety relatch ---\n')
                    output.write(relatch.stdout)
                time.sleep(0.5)
        time.sleep(args.settle_seconds)
    except Exception as exc:
        error = str(exc)
    finally:
        stop_process(task_process, timeout=10)
        stop_process(recovery, timeout=10)
        stop_process(monitor, timeout=10)
        stop_process(simulation)

    lingering_process_groups = [
        name for name, process in (
            ('task', task_process), ('recovery', recovery),
            ('monitor', monitor), ('simulation', simulation))
        if process_group_alive(process)
    ]

    summary = None
    if result_path.exists():
        try:
            summary = read_summary(result_path)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            error = error or str(exc)
    task_execution_completed = task_completed(task_log)
    clearance = summary.get('transfer_min_bottom_clearance') if summary else None
    if args.expect_safety_stop:
        physical_fault_verified = (
            args.inject_sim_fault_phase == 'none' or
            (summary and summary.get('sim_fault_command_published') is True and
             summary.get('safety_event', {}).get('code') == args.inject_sim_fault_code and
             summary.get('safety_event', {}).get('injected') is False))
        passed = bool(
            summary and summary.get('safety_event_published') is True and
            not task_execution_completed and
            physical_fault_verified and
            (recovery_completed or (recovery_safe_refusal and fallback_hold_latched)))
    else:
        passed = bool(
            task_returncode == 0 and task_execution_completed and summary and
            summary.get('placement_pass') is True and
            summary.get('slip_detected') is False and
            summary.get('release_detected') is True and
            summary.get('post_release_recontact_detected') is False and
            clearance is not None and clearance >= args.min_clearance)
    if lingering_process_groups:
        passed = False
        error = error or (
            'process cleanup left groups alive: ' + ', '.join(lingering_process_groups))
    if not passed and error is None:
        error = (
            'safety stop/recovery requirement failed' if args.expect_safety_stop else
            'task, placement, transfer-clearance, slip, or release-stability requirement failed')
    recovery_outcome = (
        'completed' if recovery_completed else
        'refused_relatched' if recovery_safe_refusal and fallback_hold_latched else
        'failed' if args.expect_safety_stop else 'not_requested')
    return {
        'scenario': scenario, 'task_returncode': task_returncode,
        'task_execution_completed': task_execution_completed,
        'task_planning_seconds': planning_seconds(task_log),
        'task_execution_seconds': execution_seconds(task_log),
        'safety_stop_expected': args.expect_safety_stop,
        'recovery_returncode': recovery_returncode,
        'recovery_completed': recovery_completed,
        'recovery_safe_refusal': recovery_safe_refusal,
        'recovery_attempts': recovery_attempts,
        'fallback_hold_latched': fallback_hold_latched,
        'recovery_outcome': recovery_outcome,
        'summary': summary, 'min_clearance_required': args.min_clearance,
        'process_cleanup_ok': not lingering_process_groups,
        'lingering_process_groups': lingering_process_groups,
        'passed': passed, 'error': error,
        'elapsed_seconds': time.monotonic() - started,
        'logs': {
            'simulation': str(sim_log), 'task': str(task_log),
            'monitor': str(monitor_log), 'recovery': str(recovery_log),
        },
    }


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--scenario', action='append',
        help='scenario name from the defaults or --scenario-file (repeatable)')
    parser.add_argument('--scenario-file', type=Path,
                        help='JSON object mapping scenario names to center_xy, size, and mass_kg')
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument(
        '--resume', action='store_true',
        help='resume an existing output directory; passed runs are skipped and failed runs retried')
    parser.add_argument('--startup-timeout', type=float, default=210)
    parser.add_argument('--task-timeout', type=float, default=150)
    parser.add_argument('--monitor-seconds', type=float, default=180)
    parser.add_argument('--settle-seconds', type=float, default=3)
    parser.add_argument('--min-clearance', type=float, default=0.020)
    parser.add_argument('--slip-threshold', type=float, default=0.005)
    parser.add_argument('--slip-confirm-samples', type=int, default=3)
    parser.add_argument('--drop-clearance-threshold', type=float, default=0.005)
    parser.add_argument('--tilt-threshold-deg', type=float, default=20.0)
    parser.add_argument('--fault-confirm-samples', type=int, default=3)
    parser.add_argument('--pose-stream-timeout', type=float, default=0.75)
    parser.add_argument(
        '--inject-safety-code', default='SLIP_DETECTED', choices=(
            'SLIP_DETECTED', 'BLOCK_DROP_DETECTED', 'TILT_DETECTED',
            'TELEMETRY_STALE'))
    parser.add_argument('--safety-cancel-enabled', action='store_true')
    parser.add_argument('--expect-safety-stop', action='store_true')
    parser.add_argument('--inject-safety-event-after-contact', type=float, default=-1.0)
    parser.add_argument(
        '--inject-sim-fault-phase', default='none', choices=(
            'none', 'after_contact', 'after_lift', 'transfer', 'lower',
            'pre_release'))
    parser.add_argument(
        '--inject-sim-fault-code', default='BLOCK_DROP_DETECTED', choices=(
            'BLOCK_DROP_DETECTED', 'TILT_DETECTED', 'TELEMETRY_STALE'))
    parser.add_argument('--inject-sim-fault-delay', type=float, default=0.0)
    parser.add_argument('--recovery-timeout', type=float, default=180.0)
    parser.add_argument(
        '--recovery-candidate', action='append', choices=(
            'vertical', 'reverse_transfer', 'lateral_left', 'lateral_right'),
        help='bounded recovery candidate to try; repeat to override default order')
    parser.add_argument(
        '--gui', action='store_false', dest='headless',
        help='show the Isaac Sim GUI for each repetition')
    parser.set_defaults(headless=True)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    if args.repeat <= 0:
        parser.error('--repeat must be positive')
    for name in ('startup_timeout', 'task_timeout', 'monitor_seconds', 'settle_seconds'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'--{name.replace("_", "-")} must be finite and positive')
    if not math.isfinite(args.min_clearance) or args.min_clearance < 0:
        parser.error('--min-clearance must be finite and non-negative')
    if not math.isfinite(args.slip_threshold) or not 0 < args.slip_threshold < 1:
        parser.error('--slip-threshold must be finite and within (0, 1)')
    if (not math.isfinite(args.drop_clearance_threshold) or
            not 0 <= args.drop_clearance_threshold < 1):
        parser.error('--drop-clearance-threshold must be finite and within [0, 1)')
    if not math.isfinite(args.tilt_threshold_deg) or not 0 < args.tilt_threshold_deg < 90:
        parser.error('--tilt-threshold-deg must be finite and within (0, 90)')
    if (not math.isfinite(args.pose_stream_timeout) or
            not 0.1 <= args.pose_stream_timeout <= 10.0):
        parser.error('--pose-stream-timeout must be finite and within [0.1, 10]')
    if (not math.isfinite(args.inject_safety_event_after_contact) or
            args.inject_safety_event_after_contact < -1.0):
        parser.error('--inject-safety-event-after-contact must be -1 or non-negative')
    if not math.isfinite(args.inject_sim_fault_delay) or args.inject_sim_fault_delay < 0:
        parser.error('--inject-sim-fault-delay must be finite and non-negative')
    if (args.inject_safety_event_after_contact >= 0.0 and
            args.inject_sim_fault_phase != 'none'):
        parser.error('synthetic safety injection and physical fault injection are exclusive')
    if not math.isfinite(args.recovery_timeout) or args.recovery_timeout <= 0:
        parser.error('--recovery-timeout must be finite and positive')
    if args.expect_safety_stop and not args.safety_cancel_enabled:
        parser.error('--expect-safety-stop requires --safety-cancel-enabled')
    if args.slip_confirm_samples <= 0 or args.fault_confirm_samples <= 0:
        parser.error('confirmation sample counts must be positive')
    return args


def main():
    args = parse_args()
    raw_scenarios = dict(DEFAULT_SCENARIOS)
    loaded_names = []
    if args.scenario_file:
        loaded = json.loads(args.scenario_file.read_text(encoding='utf-8'))
        if not isinstance(loaded, dict):
            raise ValueError('--scenario-file must contain a JSON object')
        loaded_names = list(loaded)
        raw_scenarios.update(loaded)
    names = args.scenario or loaded_names or list(raw_scenarios)
    unknown = sorted(set(names) - set(raw_scenarios))
    if unknown:
        raise ValueError(
            f'unknown scenario(s): {", ".join(unknown)}; '
            f'available: {", ".join(sorted(raw_scenarios))}')
    scenarios = [validate_scenario(name, raw_scenarios[name]) for name in names]
    stamp = dt.datetime.now().strftime('%Y%m%d-%H%M%S')
    output_dir = args.output_dir or Path('/tmp') / f'pallet-scenario-matrix-{stamp}'
    report_path = output_dir / 'report.json'
    if args.resume:
        if not report_path.exists():
            raise ValueError(f'--resume requires an existing report: {report_path}')
        report = json.loads(report_path.read_text(encoding='utf-8'))
        if not isinstance(report.get('runs'), list):
            raise ValueError(f'invalid resume report: {report_path}')
        report['resumed_at'] = dt.datetime.now(dt.timezone.utc).isoformat()
    else:
        output_dir.mkdir(parents=True, exist_ok=False)
        report = {'created_at': dt.datetime.now(dt.timezone.utc).isoformat(), 'runs': []}
    existing_runs = {
        (run.get('scenario', {}).get('name'), run.get('repetition')): index
        for index, run in enumerate(report['runs'])
    }
    for scenario in scenarios:
        for repetition in range(1, args.repeat + 1):
            key = (scenario['name'], repetition)
            existing_index = existing_runs.get(key)
            if existing_index is not None and report['runs'][existing_index].get('passed') is True:
                print(
                    f'Skipping passed {scenario["name"]} repetition {repetition}/{args.repeat}',
                    flush=True)
                continue
            base_run_dir = output_dir / f'{scenario["name"]}-{repetition:02d}'
            run_dir = base_run_dir
            retry = 1
            while run_dir.exists():
                run_dir = output_dir / f'{base_run_dir.name}-retry-{retry:02d}'
                retry += 1
            run_dir.mkdir()
            print(f'Running {scenario["name"]} repetition {repetition}/{args.repeat}', flush=True)
            try:
                result = run_once(scenario, run_dir, args)
            except Exception as exc:
                result = {
                    'scenario': scenario,
                    'status': 'runner_error',
                    'passed': False,
                    'error': f'unhandled run error: {exc}',
                    'process_cleanup_ok': None,
                }
            result['repetition'] = repetition
            result['failure_category'] = classify_failure(result)
            if existing_index is None:
                report['runs'].append(result)
                existing_runs[key] = len(report['runs']) - 1
            else:
                previous = report['runs'][existing_index]
                result['prior_attempts'] = previous.get('prior_attempts', []) + [{
                    'passed': previous.get('passed'),
                    'error': previous.get('error'),
                    'failure_category': previous.get('failure_category'),
                    'task_planning_seconds': previous.get('task_planning_seconds'),
                    'task_execution_seconds': previous.get('task_execution_seconds'),
                    'logs': previous.get('logs'),
                }]
                report['runs'][existing_index] = result
            write_json_atomic(report_path, report)
            print(json.dumps({
                'scenario': scenario['name'], 'repetition': repetition,
                'status': result.get('status'), 'passed': result.get('passed'),
                'error': result.get('error'),
            }), flush=True)
    report['passed'] = all(run.get('passed', run.get('status') == 'dry_run') for run in report['runs'])
    if not args.dry_run:
        report['statistics'] = summarize_runs(report['runs'])
        report['overall'] = summarize_matrix(report['runs'])
    write_json_atomic(report_path, report)
    print(f'Report: {report_path}', flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
