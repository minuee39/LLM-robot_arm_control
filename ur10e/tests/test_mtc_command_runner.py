import numpy as np
import json
import time
from types import SimpleNamespace

import mtc_command_runner
from mtc_command_runner import (
    RECOVERY_MTC_ARGS,
    MTC_EXECUTION_FAILURE_RETURN_CODE,
    build_mtc_invocation,
    command_to_mtc_args,
    execution_tuning_args,
    execute_user_command,
    read_vision_scene_objects,
    restore_scene_memory,
    run_grasp_failure_recovery,
    save_scene_memory,
    sync_scene_manager_from_isaac,
    sync_scene_manager_from_vision,
    run_interactive,
)
from scene_manager import SceneManager


def write_vision_scene(path, *, updated_at=None, overrides=None):
    if updated_at is None:
        updated_at = time.time()
    objects = {
        "red_block": {"confidence": 0.9, "position": [0.11, 0.22, 0.05]},
        "green_block": {"confidence": 0.91, "position": [0.31, 0.42, 0.05]},
        "blue_block": {"confidence": 0.92, "position": [0.51, 0.62, 0.05]},
    }
    if overrides:
        objects.update(overrides)
    path.write_text(
        json.dumps({"updated_at": updated_at, "frame": "world", "objects": objects}),
        encoding="utf-8",
    )


def test_build_mtc_invocation_from_local_command():
    command, mtc_args = build_mtc_invocation("빨간 블럭을 파란 블럭 옆에 둬", provider="local")

    assert command["pick_object"] == "red_block"
    assert command["target_object"] == "blue_block"
    assert command["relation"] == "near"
    assert "object_id:=red_block" in mtc_args
    assert "object_x:=-0.300000" in mtc_args
    assert "object_y:=0.300000" in mtc_args
    assert "place_x:=0.420000" in mtc_args
    assert "place_y:=0.420000" in mtc_args


def test_build_mtc_invocation_resolves_bare_left_and_right_without_llm(monkeypatch):
    def fail_if_called(*args, **kwargs):
        raise AssertionError("spatial object commands must use the deterministic parser")

    monkeypatch.setattr(mtc_command_runner, "parse_user_command_with_llm", fail_if_called)

    command, mtc_args = build_mtc_invocation(
        "왼쪽 물체를 오른쪽 물체 위로 옮겨",
        provider="gemini",
    )

    assert command["pick_object"] == "blue_block"
    assert command["target_object"] == "red_block"
    assert command["relation"] == "on"
    assert "object_id:=blue_block" in mtc_args
    assert "place_x:=-0.300000" in mtc_args


def test_command_to_mtc_args_clamps_z_to_object_center_height():
    scene_manager = SceneManager.from_defaults()
    command = {
        "action": "pick_place",
        "pick_object": "red_block",
        "target_object": "blue_block",
        "relation": "near",
        "confidence": 1.0,
    }

    mtc_args = command_to_mtc_args(command, scene_manager)

    assert "object_id:=red_block" in mtc_args
    assert "object_size_x:=0.100000" in mtc_args
    assert "object_size_y:=0.051500" in mtc_args
    assert "object_size_z:=0.100000" in mtc_args
    assert "object_z:=0.050000" in mtc_args
    assert "place_z:=0.050000" in mtc_args


def test_execution_tuning_args_selects_requested_rrt_variant():
    args = execution_tuning_args(
        "RRTstarkConfigDefault",
        5,
        25.0,
        5.0,
        8.0,
        2.0,
        8.0,
        1.0,
        5.0,
        0.02,
        0.45,
        0.08,
    )

    assert args[0] == "planner_id:=RRTstarkConfigDefault"
    assert "max_solution_cost:=25.000" in args
    assert "move_to_place_max_path_length:=8.000" in args
    assert "return_home_max_path_length:=5.000" in args


def test_scene_manager_defaults_exclude_legacy_object():
    scene_manager = SceneManager.from_defaults()

    assert "object" not in scene_manager.names()


def test_execute_user_command_dry_run_keeps_memory_unchanged(tmp_path):
    scene_manager = SceneManager.from_defaults()
    vision_scene = tmp_path / "vision.json"
    write_vision_scene(vision_scene)

    result = execute_user_command(
        "빨간 블럭을 파란 블럭 옆에 둬",
        scene_manager=scene_manager,
        provider="local",
        run_script="/unused",
        dry_run=True,
        vision_scene_file=vision_scene,
        vision_max_age=2.0,
    )

    assert result == 0
    assert scene_manager.memory.last_moved_object is None


def test_read_vision_scene_objects_validates_complete_fresh_world_scene(tmp_path):
    vision_scene = tmp_path / "vision.json"
    write_vision_scene(vision_scene, updated_at=100.0)

    objects = read_vision_scene_objects(
        vision_scene,
        max_age=2.0,
        min_confidence=0.8,
        now=101.0,
    )

    assert set(objects) == {"red_block", "green_block", "blue_block"}
    np.testing.assert_allclose(objects["red_block"]["position"], [0.11, 0.22, 0.05])
    assert objects["blue_block"]["confidence"] == 0.92


def test_read_vision_scene_objects_rejects_stale_scene(tmp_path):
    vision_scene = tmp_path / "vision.json"
    write_vision_scene(vision_scene, updated_at=100.0)

    try:
        read_vision_scene_objects(vision_scene, max_age=2.0, now=103.0)
    except ValueError as error:
        assert "stale" in str(error)
    else:
        raise AssertionError("stale vision scene should be rejected")


def test_read_vision_scene_objects_rejects_low_confidence_block(tmp_path):
    vision_scene = tmp_path / "vision.json"
    write_vision_scene(
        vision_scene,
        updated_at=100.0,
        overrides={"green_block": {"confidence": 0.4, "position": [0.31, 0.42, 0.05]}},
    )

    try:
        read_vision_scene_objects(
            vision_scene,
            max_age=2.0,
            min_confidence=0.6,
            now=101.0,
        )
    except ValueError as error:
        assert "green_block" in str(error)
        assert "confidence" in str(error)
    else:
        raise AssertionError("low-confidence detection should be rejected")


def test_read_vision_scene_objects_accepts_partial_scene(tmp_path):
    vision_scene = tmp_path / "vision.json"
    vision_scene.write_text(
        json.dumps(
            {
                "updated_at": 100.0,
                "frame": "world",
                "objects": {
                    "red_block": {"confidence": 0.9, "position": [0.11, 0.22, 0.05]},
                    "blue_block": {"confidence": 0.92, "position": [0.51, 0.62, 0.05]},
                },
            }
        ),
        encoding="utf-8",
    )

    objects = read_vision_scene_objects(vision_scene, max_age=2.0, now=101.0)

    assert set(objects) == {"red_block", "blue_block"}


def test_sync_scene_manager_from_vision_updates_detected_positions(tmp_path, monkeypatch):
    scene_manager = SceneManager.from_defaults()
    vision_scene = tmp_path / "vision.json"
    write_vision_scene(vision_scene, updated_at=100.0)
    monkeypatch.setattr("mtc_command_runner.time.time", lambda: 101.0)

    assert sync_scene_manager_from_vision(scene_manager, vision_scene) == {
        "red_block",
        "green_block",
        "blue_block",
    }
    np.testing.assert_allclose(scene_manager.get_object("red_block").position, [0.11, 0.22, 0.05])
    assert scene_manager.get_object("red_block").confidence == 0.9


def test_execute_user_command_allows_unrelated_block_to_be_missing(tmp_path):
    scene_manager = SceneManager.from_defaults()
    vision_scene = tmp_path / "vision.json"
    vision_scene.write_text(
        json.dumps(
            {
                "updated_at": time.time(),
                "frame": "world",
                "objects": {
                    "red_block": {"confidence": 0.9, "position": [0.11, 0.22, 0.05]},
                    "blue_block": {"confidence": 0.92, "position": [0.51, 0.62, 0.05]},
                },
            }
        ),
        encoding="utf-8",
    )

    result = execute_user_command(
        "빨간 블럭을 파란 블럭 옆에 둬",
        scene_manager=scene_manager,
        provider="local",
        run_script="/unused",
        dry_run=True,
        vision_scene_file=vision_scene,
    )

    assert result == 0


def test_execute_user_command_uses_remembered_position_for_missing_required_target(tmp_path, capsys):
    scene_manager = SceneManager.from_defaults()
    scene_manager.update_object("blue_block", [0.61, 0.52, 0.15], confidence=0.92)
    vision_scene = tmp_path / "vision.json"
    vision_scene.write_text(
        json.dumps(
            {
                "updated_at": time.time(),
                "frame": "world",
                "objects": {
                    "red_block": {"confidence": 0.9, "position": [0.11, 0.22, 0.05]},
                    "green_block": {"confidence": 0.91, "position": [0.31, 0.42, 0.05]},
                },
            }
        ),
        encoding="utf-8",
    )

    result = execute_user_command(
        "빨간 블럭을 파란 블럭 옆에 둬",
        scene_manager=scene_manager,
        provider="local",
        run_script="/unused",
        dry_run=True,
        vision_scene_file=vision_scene,
    )

    assert result == 0
    output = capsys.readouterr().out
    assert "camera occluded; using remembered positions" in output
    assert "blue_block=(+0.610, +0.520, +0.150) m" in output


def test_sync_scene_manager_from_isaac_updates_actual_positions(tmp_path):
    scene_manager = SceneManager.from_defaults()
    scene_state = tmp_path / "scene.json"
    scene_state.write_text(
        """
        {
          "objects": [
            {"name": "red_block", "position": [0.11, 0.22, 0.033], "size": [0.1, 0.05, 0.1]},
            {"name": "unknown", "position": [9, 9, 9]}
          ]
        }
        """,
        encoding="utf-8",
    )

    assert sync_scene_manager_from_isaac(scene_manager, scene_state) is True
    np.testing.assert_allclose(scene_manager.get_object("red_block").position, np.array([0.11, 0.22, 0.033]))


def test_scene_memory_round_trip_preserves_hidden_object_position(tmp_path):
    memory_file = tmp_path / "scene_memory.json"
    original = SceneManager.from_defaults()
    original.update_object("blue_block", [0.31, 0.29, 0.15], confidence=0.92)
    save_scene_memory(original, memory_file)

    restored = SceneManager.from_defaults()
    assert restore_scene_memory(restored, memory_file) is True
    np.testing.assert_allclose(restored.get_object("blue_block").position, [0.31, 0.29, 0.15])


def test_execute_user_command_updates_scene_after_success(monkeypatch):
    scene_manager = SceneManager.from_defaults()
    calls = []

    class Completed:
        returncode = 0

    def fake_run(args, check):
        calls.append(args)
        assert check is False
        return Completed()

    monkeypatch.setattr("mtc_command_runner.subprocess.run", fake_run)
    sync_count = 0

    def fake_sync(scene_manager, *args, **kwargs):
        nonlocal sync_count
        sync_count += 1
        position = [0.11, 0.22, 0.05] if sync_count == 1 else [0.51, 0.52, 0.053]
        scene_manager.update_object("red_block", position, confidence=0.9)
        return {"red_block", "green_block", "blue_block"}

    monkeypatch.setattr("mtc_command_runner.sync_scene_manager_from_vision", fake_sync)
    monkeypatch.setattr("mtc_command_runner.vision_scene_mtime", lambda path: 1)
    monkeypatch.setattr("mtc_command_runner.wait_for_vision_scene_update", lambda *args, **kwargs: True)

    class SuccessfulGraspMonitor:
        def __init__(self, *args, **kwargs):
            pass

        def start(self):
            pass

        def stop(self):
            return SimpleNamespace(
                success=True,
                reason="verified",
                sample_count=4,
                lifted_sample_count=3,
                max_lift=0.12,
                max_relative_deviation=0.004,
            )

    monkeypatch.setattr("mtc_command_runner.GraspMonitor", SuccessfulGraspMonitor)

    result = execute_user_command(
        "빨간 블럭을 파란 블럭 옆에 둬",
        scene_manager=scene_manager,
        provider="local",
        run_script="/run_mtc",
    )

    assert result == 0
    assert len(calls) == 1
    assert calls[0][0] == "/run_mtc"
    assert "planner_id:=RRTConnectkConfigDefault" in calls[0]
    assert "max_solutions:=1" in calls[0]
    assert "max_solution_cost:=60.000" in calls[0]
    assert "move_to_pick_timeout:=5.000" in calls[0]
    assert "move_to_pick_max_path_length:=-1.000" in calls[0]
    assert "move_to_place_timeout:=4.000" in calls[0]
    assert "move_to_place_max_path_length:=-1.000" in calls[0]
    assert "return_home_timeout:=1.000" in calls[0]
    assert "return_home_max_path_length:=-1.000" in calls[0]
    assert "gripper_close_min:=0.020" in calls[0]
    assert "gripper_close_max:=0.500" in calls[0]
    assert "gripper_close_step:=0.080" in calls[0]
    assert "object_x:=0.110000" in calls[0]
    assert "object_y:=0.220000" in calls[0]
    assert scene_manager.memory.last_moved_object == "red_block"
    np.testing.assert_allclose(scene_manager.get_object("red_block").position, np.array([0.51, 0.52, 0.053]))


def test_execute_user_command_updates_memory_from_plan_when_pick_is_occluded_after_success(monkeypatch):
    scene_manager = SceneManager.from_defaults()

    class Completed:
        returncode = 0

    monkeypatch.setattr("mtc_command_runner.subprocess.run", lambda *args, **kwargs: Completed())
    sync_count = 0

    def fake_sync(manager, *args, **kwargs):
        nonlocal sync_count
        sync_count += 1
        manager.update_object("blue_block", [0.30, 0.30, 0.15], confidence=0.9)
        if sync_count == 1:
            manager.update_object("red_block", [-0.30, 0.30, 0.05], confidence=0.9)
            return {"red_block", "blue_block", "green_block"}
        return {"blue_block", "green_block"}

    monkeypatch.setattr("mtc_command_runner.sync_scene_manager_from_vision", fake_sync)
    monkeypatch.setattr("mtc_command_runner.vision_scene_mtime", lambda path: 1)
    monkeypatch.setattr("mtc_command_runner.wait_for_vision_scene_update", lambda *args, **kwargs: True)

    class SuccessfulGraspMonitor:
        def __init__(self, *args, **kwargs):
            pass

        def start(self):
            pass

        def stop(self):
            return SimpleNamespace(
                success=True,
                reason="verified",
                sample_count=4,
                lifted_sample_count=3,
                max_lift=0.12,
                max_relative_deviation=0.004,
            )

    monkeypatch.setattr("mtc_command_runner.GraspMonitor", SuccessfulGraspMonitor)

    result = execute_user_command(
        "빨간 블럭을 파란 블럭 위에 둬",
        scene_manager=scene_manager,
        provider="local",
        run_script="/run_mtc",
    )

    assert result == 0
    assert scene_manager.memory.last_moved_object == "red_block"
    np.testing.assert_allclose(
        scene_manager.get_object("red_block").position,
        np.array([0.30, 0.30, 0.255]),
    )


def test_execute_user_command_rejects_unverified_physical_grasp(monkeypatch):
    scene_manager = SceneManager.from_defaults()

    class Completed:
        returncode = 0

    calls = []

    def fake_run(args, check):
        calls.append(args)
        return Completed()

    monkeypatch.setattr("mtc_command_runner.subprocess.run", fake_run)
    monkeypatch.setattr(
        "mtc_command_runner.sync_scene_manager_from_vision",
        lambda *args, **kwargs: {"red_block", "green_block", "blue_block"},
    )

    class FailedGraspMonitor:
        def __init__(self, *args, **kwargs):
            pass

        def start(self):
            pass

        def stop(self):
            return SimpleNamespace(
                success=False,
                reason="object did not remain lifted",
                sample_count=5,
                lifted_sample_count=0,
                max_lift=0.002,
                max_relative_deviation=None,
            )

    monkeypatch.setattr("mtc_command_runner.GraspMonitor", FailedGraspMonitor)

    result = execute_user_command(
        "빨간 블럭을 파란 블럭 위에 둬",
        scene_manager=scene_manager,
        provider="local",
        run_script="/run_mtc",
    )

    assert result == 1
    assert calls[0][0] == "/run_mtc"
    assert len(calls) == 1
    assert scene_manager.memory.last_moved_object is None


def test_execute_user_command_does_not_update_scene_after_failure(monkeypatch):
    scene_manager = SceneManager.from_defaults()

    mtc_call_count = 0

    def fake_run(args, check):
        nonlocal mtc_call_count
        assert check is False
        mtc_call_count += 1
        return SimpleNamespace(
            returncode=MTC_EXECUTION_FAILURE_RETURN_CODE if mtc_call_count == 1 else 0
        )

    monkeypatch.setattr("mtc_command_runner.subprocess.run", fake_run)
    monkeypatch.setattr(
        "mtc_command_runner.sync_scene_manager_from_vision",
        lambda *args, **kwargs: {"red_block", "blue_block"},
    )

    result = execute_user_command(
        "빨간 블럭을 파란 블럭 위에 둬",
        scene_manager=scene_manager,
        provider="local",
        run_script="/run_mtc",
    )

    assert result == MTC_EXECUTION_FAILURE_RETURN_CODE
    assert mtc_call_count == 2
    assert scene_manager.memory.last_moved_object is None
    np.testing.assert_allclose(scene_manager.get_object("red_block").position, np.array([-0.30, 0.30, 0.02575]))


def test_execute_user_command_skips_recovery_after_planning_failure(monkeypatch):
    scene_manager = SceneManager.from_defaults()
    calls = []

    def fake_run(args, check):
        calls.append(args)
        return SimpleNamespace(returncode=1)

    monkeypatch.setattr("mtc_command_runner.subprocess.run", fake_run)
    monkeypatch.setattr(
        "mtc_command_runner.sync_scene_manager_from_vision",
        lambda *args, **kwargs: {"red_block", "blue_block"},
    )

    result = execute_user_command(
        "빨간 블럭을 파란 블럭 위에 둬",
        scene_manager=scene_manager,
        provider="local",
        run_script="/run_mtc",
    )

    assert result == 1
    assert len(calls) == 1
    assert scene_manager.memory.last_moved_object is None


def test_run_grasp_failure_recovery_appends_recovery_mode_after_task_args(monkeypatch):
    calls = []

    def fake_run(args, check):
        calls.append((args, check))
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr("mtc_command_runner.subprocess.run", fake_run)

    result = run_grasp_failure_recovery(
        "/run_mtc",
        ["object_id:=red_block", "max_solutions:=2"],
    )

    assert result == 0
    assert calls == [
        (
            [
                "/run_mtc",
                "object_id:=red_block",
                "max_solutions:=2",
                *RECOVERY_MTC_ARGS,
            ],
            False,
        )
    ]


def test_run_interactive_executes_initial_command_then_waits_for_next(monkeypatch):
    commands = []
    scene_manager_ids = []
    inputs = iter(["방금 옮긴 블럭을 초록 블럭 위에 올려", "exit"])

    def fake_execute_user_command(user_text, scene_manager, provider, run_script, dry_run=False, **kwargs):
        commands.append(user_text)
        scene_manager_ids.append(id(scene_manager))
        return 0

    monkeypatch.setattr("mtc_command_runner.execute_user_command", fake_execute_user_command)

    def fake_input(prompt):
        assert commands
        return next(inputs)

    monkeypatch.setattr("builtins.input", fake_input)

    result = run_interactive(
        provider="local",
        run_script="/run_mtc",
        initial_command="빨간 블럭을 파란 블럭 위에 둬",
    )

    assert result == 0
    assert commands == [
        "빨간 블럭을 파란 블럭 위에 둬",
        "방금 옮긴 블럭을 초록 블럭 위에 올려",
    ]
    assert len(set(scene_manager_ids)) == 1


def test_run_interactive_prints_next_command_prompt_after_initial_execution(monkeypatch, capsys):
    def fake_execute_user_command(*args, **kwargs):
        print("[TEST] initial command finished", flush=True)
        return 0

    monkeypatch.setattr("mtc_command_runner.execute_user_command", fake_execute_user_command)
    monkeypatch.setattr("builtins.input", lambda prompt: "exit")

    result = run_interactive(
        provider="local",
        run_script="/run_mtc",
        initial_command="빨간 블럭을 파란 블럭 위에 놓아",
    )

    output = capsys.readouterr().out
    assert result == 0
    assert output.index("[TEST] initial command finished") < output.index("[READY] 다음 pick/place 명령")
