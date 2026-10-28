import json
import pytest
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import palletizing_job


def test_validate_only_accepts_overrides_without_scene_or_output(tmp_path):
    output = tmp_path / 'job'
    result = palletizing_job.subprocess.run([
        sys.executable, palletizing_job.__file__, '--output-dir', str(output),
        '--scene-file', str(tmp_path / 'missing-scene'), '--validate-only',
        '--object-place-yaw', 'red_block', '0',
        '--object-place-yaw', 'green_block', '1.5707963267948966',
    ], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert not output.exists()


@pytest.mark.parametrize('extra', [
    ['red_block'], ['red_block', 'nan'], ['unknown_block', '0'],
    ['red_block', '4'], ['red_block', '0', '--object-place-yaw', 'red_block', '1'],
])
def test_gui_wrapper_rejects_invalid_override_before_startup(tmp_path, extra):
    output = tmp_path / 'run'
    wrapper = Path(palletizing_job.__file__).resolve().parents[1] / 'ros2_ur_ws/scripts/run_palletizing_gui.sh'
    result = palletizing_job.subprocess.run([
        'bash', str(wrapper), '--output-dir', str(output), '--object-place-yaw', *extra,
    ], capture_output=True, text=True, timeout=10)
    assert result.returncode == 2, result.stderr
    assert not output.exists()


def test_fresh_detected_objects_validates_contract(tmp_path):
    scene = tmp_path / "scene.json"
    scene.write_text(json.dumps({
        "frame": "base_footprint", "unit": "m", "updated_at": 100.0,
        "objects": {"red_block": {}, "green_block": {}},
    }), encoding="utf-8")
    assert palletizing_job.fresh_detected_objects(scene, 2.0, now=101.0) == {
        "red_block", "green_block"}
    assert palletizing_job.fresh_detected_objects(scene, 0.5, now=101.0) == set()


def test_wait_for_objects_requires_consecutive_complete_scenes(monkeypatch):
    snapshots = iter((
        {"red_block"},
        {"red_block", "blue_block"},
        {"blue_block"},
        {"red_block", "blue_block"},
        {"red_block", "blue_block"},
        {"red_block", "blue_block"},
    ))
    monkeypatch.setattr(
        palletizing_job, "fresh_detected_objects",
        lambda *unused: next(snapshots),
    )
    with patch.object(palletizing_job.time, "sleep", return_value=None):
        assert palletizing_job.wait_for_objects(
            None, {"red_block", "blue_block"}, 10.0, 5.0) is True


def test_fresh_detected_objects_rejects_invalid_root_and_timestamps(tmp_path):
    scene = tmp_path / "scene.json"
    for payload in ([], None, "invalid", *(
            {"frame": "base_footprint", "unit": "m", "updated_at": stamp,
             "objects": {"red_block": {}}}
            for stamp in (float("nan"), float("inf"), -float("inf"), True, False))):
        scene.write_text(json.dumps(payload), encoding="utf-8")
        assert palletizing_job.fresh_detected_objects(scene, 2.0, now=1.0) == set()


def test_job_rejects_nonfinite_or_unbounded_wait_options_before_execution(tmp_path):
    for option in ("--motion-monitor-seconds", "--detection-timeout", "--vision-max-age"):
        for value in ("nan", "inf", "0"):
            result = palletizing_job.subprocess.run(
                [sys.executable, palletizing_job.__file__, "--output-dir", str(tmp_path / "out"),
                 option, value], capture_output=True, text=True, check=False)
            assert result.returncode == 2
            assert f"{option} must be finite and positive" in result.stderr
    assert not (tmp_path / "out").exists()


def test_job_rejects_invalid_release_opening_before_execution(tmp_path):
    for value in ('nan', 'inf', '0', '-1', '0.034'):
        result = palletizing_job.subprocess.run(
            [sys.executable, palletizing_job.__file__, '--output-dir', str(tmp_path/'out'),
             '--release-open-position', value], capture_output=True, text=True, check=False)
        assert result.returncode == 2
        assert '--release-open-position must be finite' in result.stderr
    assert not (tmp_path/'out').exists()


def test_object_place_yaw_validation_and_job_scope():
    tasks = [{'pick_object': 'red_block'}, {'pick_object': 'green_block'}]
    assert palletizing_job.validate_object_place_yaws(None, tasks) == {}
    assert palletizing_job.validate_object_place_yaws([['red_block', '0'], ['green_block', '1.5708']], tasks) == {'red_block': 0., 'green_block': 1.5708}
    for entries in ([['blue_block', 0]], [['red_block', True]], [['red_block', 'nan']],
                    [['red_block', 4]], [['red_block', 0], ['red_block', 1]], [['red_block']]):
        with pytest.raises(ValueError):
            palletizing_job.validate_object_place_yaws(entries, tasks)


def test_job_invalid_object_yaw_is_rejected_before_execution(tmp_path):
    for name, value in [('unknown', '0'), ('red_block', 'nan'), ('red_block', '4')]:
        result = palletizing_job.subprocess.run(
            [sys.executable, palletizing_job.__file__, '--output-dir', str(tmp_path/'out'),
             '--object-place-yaw', name, value], capture_output=True, text=True, check=False)
        assert result.returncode == 2
    assert not (tmp_path/'out').exists()


def test_wait_for_objects_rejects_invalid_stable_sample_count():
    try:
        palletizing_job.wait_for_objects(None, {"red_block"}, 1.0, 1.0, 0)
    except ValueError as error:
        assert "stable_samples" in str(error)
    else:
        raise AssertionError("expected stable_samples validation")


def test_run_job_stops_after_first_failed_object(tmp_path, monkeypatch):
    args = SimpleNamespace(
        output_dir=tmp_path / "out", scene_file=tmp_path / "scene.json",
        detection_timeout=1.0, vision_max_age=5.0,
        placements={"red_block": [0.1, 0.1, 0.025],
                    "green_block": [0.2, 0.1, 0.025]},
        command_runner=tmp_path / "runner.py", repository_dir=tmp_path,
        job_id="job-1", gripper_close_position=0.002, place_yaw=-1.5708,
        grasp_angle_delta=6.283185307179586, cartesian_transfer=True,
        upright_transfer=True, transfer_height=0.10,
        totg_path_tolerance=0.01, scene_obstacles=True, release_open_position=0.020,
    )
    required = []
    def wait(*arguments):
        required.append(arguments[1])
        return True
    monkeypatch.setattr(palletizing_job, "wait_for_objects", wait)
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=2)

    monkeypatch.setattr(palletizing_job.subprocess, "run", run)
    report = palletizing_job.run_job(args)
    assert report["passed"] is False
    assert report["failure_class"] == "execution_dynamics"
    assert report["completed_count"] == 0
    assert len(report["runs"]) == 1
    assert len(calls) == 1
    assert "--arm-velocity-scaling" in calls[0]
    assert calls[0][calls[0].index("--arm-velocity-scaling") + 1] == "1.0"
    assert calls[0][calls[0].index("--gripper-close-position") + 1] == "0.002"
    assert calls[0][calls[0].index("--place-yaw") + 1] == "-1.5708"
    assert calls[0][calls[0].index("--grasp-angle-delta") + 1] == "6.283185307179586"
    assert "--cartesian-transfer" in calls[0]
    assert "--upright-transfer" in calls[0]
    assert '--scene-obstacles' in calls[0]
    assert required == [{'red_block', 'green_block', 'blue_block'}]
    assert calls[0][calls[0].index("--totg-path-tolerance") + 1] == "0.01"
    assert calls[0][calls[0].index("--transfer-height") + 1] == "0.1"
    assert calls[0][calls[0].index('--release-open-position') + 1] == '0.02'
    assert report['planning_tuning']['release_open_position_m'] == 0.020
    assert json.loads((args.output_dir / "report.json").read_text())["passed"] is False


def test_run_job_retries_only_pre_motion_failure(tmp_path, monkeypatch):
    args = SimpleNamespace(
        output_dir=tmp_path / "out", scene_file=tmp_path / "scene.json",
        detection_timeout=1.0, vision_max_age=5.0,
        placements={"blue_block": [0.28, -0.28, 0.025]},
        command_runner=tmp_path / "runner.py", repository_dir=tmp_path,
        job_id="job-retry", planning_attempts=3,
    )
    waits = []
    monkeypatch.setattr(palletizing_job, "wait_for_objects",
                        lambda *unused: waits.append(unused[1]) or True)
    returncodes = iter((1, 0))
    monkeypatch.setattr(
        palletizing_job.subprocess, "run",
        lambda *args, **kwargs: SimpleNamespace(returncode=next(returncodes)),
    )

    report = palletizing_job.run_job(args)

    assert report["passed"] is True
    assert report["failure_class"] is None
    assert [attempt["returncode"] for attempt in report["runs"][0]["attempts"]] == [1, 0]
    assert waits == [{"blue_block"}, {"blue_block"}]


def test_planning_retry_stops_when_camera_detection_is_unavailable(tmp_path, monkeypatch):
    args = SimpleNamespace(
        output_dir=tmp_path / "out", scene_file=tmp_path / "scene.json",
        detection_timeout=1.0, vision_max_age=5.0,
        placements={"red_block": [0.24, -0.28, 0.025]},
        command_runner=tmp_path / "runner.py", repository_dir=tmp_path,
        job_id="retry-camera", planning_attempts=3,
    )
    available = iter((True, False))
    monkeypatch.setattr(palletizing_job, "wait_for_objects", lambda *unused: next(available))
    calls = []
    monkeypatch.setattr(palletizing_job.subprocess, "run",
                        lambda *command, **kwargs: calls.append(command) or SimpleNamespace(returncode=1))
    report = palletizing_job.run_job(args)
    run = report["runs"][0]
    assert report["passed"] is False
    assert report["failure_class"] == "planning_or_startup"
    assert len(calls) == len(run["attempts"]) == 1
    assert run["returncode"] == 1
    assert run["retry_blocked"] == "fresh stable camera detection unavailable before retry"
    assert run["log"] == run["attempts"][0]["log"]


def test_pre_motion_retry_uses_separate_complete_monitor_lifetimes(tmp_path, monkeypatch):
    args = SimpleNamespace(
        output_dir=tmp_path / "out", scene_file=tmp_path / "scene.json",
        detection_timeout=1.0, vision_max_age=5.0, motion_monitor=True,
        placements={"red_block": [0.24, -0.28, 0.025]},
        command_runner=tmp_path / "runner.py", repository_dir=tmp_path,
        job_id="monitor-retry", planning_attempts=3,
    )
    monkeypatch.setattr(palletizing_job, "wait_for_objects", lambda *unused: True)
    monkeypatch.setattr(palletizing_job.time, "sleep", lambda *unused: None)
    started, stopped = [], []
    def start(args, object_id, target, output, log):
        monitor = SimpleNamespace(identity=len(started) + 1)
        started.append((monitor, output))
        return monitor, None
    def stop(monitor, stream, output):
        stopped.append((monitor, output))
        return {"placement_pass": monitor.identity == 2, "fresh_feedback": True,
                "release_detected": monitor.identity == 2,
                "post_release_recontact_detected": False}
    monkeypatch.setattr(palletizing_job, "start_motion_monitor", start)
    monkeypatch.setattr(palletizing_job, "stop_motion_monitor", stop)
    codes = iter((1, 0))
    monkeypatch.setattr(palletizing_job, "run_supervised_command",
                        lambda *unused: (SimpleNamespace(returncode=next(codes)), None))
    report = palletizing_job.run_job(args)
    assert report["passed"] is True
    assert started == stopped
    assert len(started) == 2 and started[0][1] != started[1][1]
    run = report["runs"][0]
    assert run["motion_output"] == str(started[1][1])
    assert run["attempts"][0]["motion_summary"]["release_detected"] is False
    assert run["attempts"][1]["motion_summary"]["release_detected"] is True


def test_missing_monitor_result_blocks_pre_motion_retry(tmp_path, monkeypatch):
    args = SimpleNamespace(
        output_dir=tmp_path / "out", scene_file=tmp_path / "scene.json",
        detection_timeout=1.0, vision_max_age=5.0, motion_monitor=True,
        placements={"red_block": [0.24, -0.28, 0.025]},
        command_runner=tmp_path / "runner.py", repository_dir=tmp_path,
        job_id="monitor-failed", planning_attempts=3,
    )
    monkeypatch.setattr(palletizing_job, "wait_for_objects", lambda *unused: True)
    monkeypatch.setattr(palletizing_job.time, "sleep", lambda *unused: None)
    start = Mock(return_value=(object(), None))
    monkeypatch.setattr(palletizing_job, "start_motion_monitor", start)
    monkeypatch.setattr(palletizing_job, "stop_motion_monitor", lambda *unused: None)
    monkeypatch.setattr(palletizing_job, "run_supervised_command",
                        lambda *unused: (SimpleNamespace(returncode=1), None))
    report = palletizing_job.run_job(args)
    assert report["passed"] is False
    assert report["failure_class"] == "monitoring"
    start.assert_called_once()
    assert len(report["runs"][0]["attempts"]) == 1


def test_run_job_preserves_relative_command_for_command_runner(tmp_path, monkeypatch):
    relative_task = {
        "action": "pick_place", "pick_object": "green_block",
        "target_object": "blue_block", "relation": "near", "confidence": 1.0,
    }
    args = SimpleNamespace(
        output_dir=tmp_path / "out", scene_file=tmp_path / "scene.json",
        detection_timeout=1.0, vision_max_age=5.0,
        tasks=[relative_task], placements=None,
        command_runner=tmp_path / "runner.py", repository_dir=tmp_path,
        job_id="job-relative", planning_attempts=1,
    )
    monkeypatch.setattr(palletizing_job, "wait_for_objects", lambda *unused: True)
    manager = SimpleNamespace(
        resolve_target_position=lambda unused: [0.24, 0.30, 0.025])
    monkeypatch.setattr(palletizing_job, "load_pallet_scene", lambda *args, **kwargs: manager)
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(palletizing_job.subprocess, "run", run)

    report = palletizing_job.run_job(args)

    assert report["passed"] is True
    invocation = calls[0]
    encoded_command = invocation[invocation.index("--once") + 1]
    assert json.loads(encoded_command) == relative_task
    assert report["runs"][0]["target"] == [0.24, 0.30, 0.025]


@pytest.mark.parametrize('override', [None, 0., 1.5708])
def test_stack_planning_retries_use_distinct_place_yaws(tmp_path, monkeypatch, override):
    task = {
        "action": "pick_place", "pick_object": "blue_block",
        "target_object": "red_block", "relation": "on", "confidence": 1.0,
    }
    args = SimpleNamespace(
        output_dir=tmp_path / "out", scene_file=tmp_path / "scene.json",
        detection_timeout=1.0, vision_max_age=5.0,
        tasks=[task], placements=None,
        command_runner=tmp_path / "runner.py", repository_dir=tmp_path,
        job_id="job-stack", planning_attempts=3,
        object_place_yaws={} if override is None else {'blue_block': override},
        place_yaw=None if override is None else -.4,
    )
    monkeypatch.setattr(palletizing_job, "wait_for_objects", lambda *unused: True)
    manager = SimpleNamespace(resolve_target_position=lambda unused: [-0.32, 0.26, 0.075])
    monkeypatch.setattr(palletizing_job, "load_pallet_scene", lambda *args, **kwargs: manager)
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=1)

    monkeypatch.setattr(palletizing_job.subprocess, "run", run)

    report = palletizing_job.run_job(args)

    assert report["passed"] is False
    expected = list(palletizing_job.STACK_PLACE_YAWS) if override is None else [override]*3
    assert [
        float(call[call.index("--place-yaw") + 1]) for call in calls
    ] == expected
    assert [
        attempt["place_yaw"] for attempt in report["runs"][0]["attempts"]
    ] == expected


def test_slip_summary_is_classified_as_execution_dynamics():
    runs = [{
        "passed": False, "returncode": 0,
        "motion_summary": {
            "slip_detected": True,
            "release_detected": True,
            "post_release_recontact_detected": False,
        },
    }]
    assert palletizing_job.speed_failure_class(runs) == "execution_dynamics"


def test_successful_runner_requires_monitor_result_when_enabled(tmp_path, monkeypatch):
    args = SimpleNamespace(
        output_dir=tmp_path / "out", scene_file=tmp_path / "scene.json",
        detection_timeout=1.0, vision_max_age=5.0,
        placements={"red_block": [0.24, -0.28, 0.025]},
        command_runner=tmp_path / "runner.py", repository_dir=tmp_path,
        job_id="missing-monitor", motion_monitor=True,
    )
    monkeypatch.setattr(palletizing_job, "wait_for_objects", lambda *unused: True)
    monkeypatch.setattr(palletizing_job, "start_motion_monitor", lambda *unused: (object(), None))
    monkeypatch.setattr(palletizing_job, "stop_motion_monitor", lambda *unused: None)
    monkeypatch.setattr(palletizing_job, "run_supervised_command",
                        lambda *unused: (SimpleNamespace(returncode=0), None))
    monkeypatch.setattr(palletizing_job.time, "sleep", lambda *unused: None)
    monkeypatch.setattr(palletizing_job.subprocess, "run",
                        lambda *unused, **kwargs: SimpleNamespace(returncode=0))
    report = palletizing_job.run_job(args)
    assert report["passed"] is False
    assert report["completed_count"] == 0
    assert report["failure_class"] == "monitoring"


def test_runtime_fault_summary_is_classified_as_execution_dynamics():
    runs = [{
        "passed": False, "returncode": 0,
        "motion_summary": {
            "slip_detected": False,
            "runtime_fault_detected": True,
            "safety_event_published": True,
            "release_detected": True,
            "post_release_recontact_detected": False,
        },
    }]
    assert palletizing_job.speed_failure_class(runs) == "execution_dynamics"


def test_supervisor_stops_running_command_when_monitor_exits(tmp_path, monkeypatch):
    monitor = Mock()
    monitor.poll.side_effect = [None, 1]
    runner = Mock(pid=12345)
    runner.poll.return_value = None
    monkeypatch.setattr(palletizing_job.subprocess, "Popen", lambda *args, **kwargs: runner)
    monkeypatch.setattr(palletizing_job.subprocess, "run",
                        lambda *args, **kwargs: SimpleNamespace(returncode=0))
    cleanup = Mock(return_value=True)
    monkeypatch.setattr(palletizing_job, "stop_command_group", cleanup)
    completed, error = palletizing_job.run_supervised_command(
        ["runner"], SimpleNamespace(repository_dir=tmp_path), None, monitor)
    assert completed.returncode == 2
    assert error == "motion monitor terminated early; safety stop acknowledged"
    cleanup.assert_called_once_with(runner)
    assert palletizing_job.speed_failure_class([
        {"passed": False, "returncode": 2, "monitoring_failure": True}]) == "monitoring"


def test_supervisor_never_launches_runner_if_monitor_already_exited(tmp_path, monkeypatch):
    monitor = Mock()
    monitor.poll.return_value = 1
    launch = Mock()
    monkeypatch.setattr(palletizing_job.subprocess, "Popen", launch)
    monkeypatch.setattr(palletizing_job.subprocess, "run",
                        lambda *args, **kwargs: SimpleNamespace(returncode=1))
    completed, error = palletizing_job.run_supervised_command(
        ["runner"], SimpleNamespace(repository_dir=tmp_path), None, monitor)
    launch.assert_not_called()
    assert completed.returncode == 2
    assert "not acknowledged" in error


def test_monitor_result_requires_current_identity_and_supervisor_shutdown(tmp_path):
    output = tmp_path / "motion.json"
    for end_reason, identity, alive, expected in (
            ("supervisor_stop", "new", True, True),
            ("timeout", "new", True, False),
            ("supervisor_stop", "old", True, False),
            ("supervisor_stop", "new", False, False)):
        monitor = Mock(monitor_id="new", returncode=0)
        monitor.poll.return_value = None if alive else 0
        summary = {"monitoring": {
            "monitor_id": identity, "end_reason": end_reason, "supervised": True}}
        output.write_text(json.dumps({"summary": summary}), encoding="utf-8")
        result = palletizing_job.stop_motion_monitor(monitor, Mock(), output)
        assert (result is not None) is expected


def test_supervisor_keeps_monitor_running_until_runner_completes(tmp_path, monkeypatch):
    monitor = Mock()
    monitor.poll.return_value = None
    runner = Mock()
    runner.poll.side_effect = [None, 0]
    monkeypatch.setattr(palletizing_job.subprocess, "Popen", lambda *args, **kwargs: runner)
    stop = Mock()
    monkeypatch.setattr(palletizing_job.subprocess, "run", stop)
    monkeypatch.setattr(palletizing_job.time, "sleep", lambda *unused: None)
    completed, error = palletizing_job.run_supervised_command(
        ["runner"], SimpleNamespace(repository_dir=tmp_path), None, monitor)
    assert completed.returncode == 0 and error is None
    stop.assert_not_called()


def test_supervisor_cleans_isolated_runner_on_interrupt(tmp_path, monkeypatch):
    monitor, runner = Mock(), Mock()
    monitor.poll.return_value = None
    runner.poll.side_effect = KeyboardInterrupt
    monkeypatch.setattr(palletizing_job.subprocess, "Popen", lambda *args, **kwargs: runner)
    safety, cleanup = Mock(return_value=True), Mock(return_value=True)
    monkeypatch.setattr(palletizing_job, "request_monitor_safety_stop", safety)
    monkeypatch.setattr(palletizing_job, "stop_command_group", cleanup)
    try:
        palletizing_job.run_supervised_command(
            ["runner"], SimpleNamespace(repository_dir=tmp_path), None, monitor)
    except KeyboardInterrupt:
        pass
    else:
        raise AssertionError("interrupt was hidden")
    safety.assert_called_once()
    cleanup.assert_called_once_with(runner)
    monitor.send_signal.assert_called_once_with(palletizing_job.signal.SIGINT)


def test_group_cleanup_escalates_after_leader_exits(monkeypatch):
    process = Mock(pid=12345)
    process.poll.return_value = 0
    signals = []
    def killpg(pid, stop_signal):
        if stop_signal:
            signals.append(stop_signal)
        elif palletizing_job.signal.SIGKILL in signals:
            raise ProcessLookupError
    monkeypatch.setattr(palletizing_job.os, "killpg", killpg)
    assert palletizing_job.stop_command_group(process, grace_seconds=0)
    assert signals == [palletizing_job.signal.SIGINT,
                       palletizing_job.signal.SIGTERM, palletizing_job.signal.SIGKILL]


def test_monitor_cleanup_kills_after_two_wait_timeouts(tmp_path):
    monitor, stream = Mock(returncode=-9), Mock()
    monitor.poll.return_value = None
    monitor.wait.side_effect = [
        palletizing_job.subprocess.TimeoutExpired("monitor", 15),
        palletizing_job.subprocess.TimeoutExpired("monitor", 5), None]
    assert palletizing_job.stop_motion_monitor(
        monitor, stream, tmp_path / "missing.json") is None
    monitor.terminate.assert_called_once()
    monitor.kill.assert_called_once()
    stream.close.assert_called_once()


def test_job_preserves_full_ros_logs_in_output_directory(tmp_path, monkeypatch):
    args = SimpleNamespace(repository_dir=tmp_path, output_dir=tmp_path / "job")
    run = Mock(return_value=SimpleNamespace(returncode=0))
    monkeypatch.setattr(palletizing_job.subprocess, "run", run)
    completed, error = palletizing_job.run_supervised_command(["runner"], args, None, None)
    assert completed.returncode == 0 and error is None
    assert run.call_args.kwargs["env"]["MTC_LAUNCH_LOG_DIR"] == str(tmp_path / "job" / "ros")


def test_pre_motion_failure_is_not_misclassified_from_missing_release():
    runs = [{
        "passed": False,
        "returncode": 1,
        "motion_summary": {
            "slip_detected": False,
            "release_detected": False,
            "post_release_recontact_detected": None,
        },
    }]

    assert palletizing_job.speed_failure_class(runs) == "planning_or_startup"
