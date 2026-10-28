import numpy as np
import json
import time
import pytest
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
    pallet_block_pose_topic,
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


def test_pallet_scene_uses_its_camera_and_object_dimensions(tmp_path):
    scene_path = tmp_path / "pallet.json"
    scene_path.write_text(json.dumps({
        "frame": "base_footprint", "unit": "m", "updated_at": 100.0,
        "camera": {
            "eye": [0.0, -1.0, 1.0], "target": [0.0, 0.0, 0.0],
            "up": [0.0, 0.0, 1.0],
        },
        "objects": {
            "red_block": {"position": [-0.3, 0.3, 0.025], "size": [0.03, 0.03, 0.05], "confidence": 0.97},
            "blue_block": {"position": [0.3, 0.3, 0.025], "size": [0.03, 0.03, 0.05], "confidence": 0.96},
        },
    }), encoding="utf-8")
    manager = mtc_command_runner.load_pallet_scene(scene_path, now=101.0)
    command, args = build_mtc_invocation(
        "왼쪽 물체를 오른쪽 물체 왼쪽에 놓아", provider="local",
        scene_manager=manager,
    )
    assert command["pick_object"] == "red_block"
    assert command["target_object"] == "blue_block"
    assert "object_size_x:=0.030000" in args
    assert "place_x:=0.150000" in args
    assert "place_z:=0.025000" in args

    command, args = build_mtc_invocation(
        "빨간 블록을 파란 블록 옆에 놓아", provider="local",
        scene_manager=manager,
    )
    assert command["relation"] == "near"
    assert "place_x:=0.240000" in args
    assert "place_y:=0.300000" in args

    command, args = build_mtc_invocation(
        "빨간 블록을 파란 블록 위에 놓아", provider="local",
        scene_manager=manager,
    )
    assert command["relation"] == "on"
    assert "place_z:=0.075000" in args
    assert "lower_min_distance:=0.080" in args
    assert "lower_max_distance:=0.080" in args
    assert "retreat_min_distance:=0.080" in args
    assert "retreat_max_distance:=0.080" in args
    assert "place_max_ik_solutions:=8" in args
    assert "place_ik_min_solution_distance:=0.100" in args

    command, args = build_mtc_invocation(
        json.dumps({
            "action": "pick_place", "pick_object": "red_block",
            "target_object": "blue_block", "relation": "right_of",
            "confidence": 1.0,
        }),
        provider="local", scene_manager=manager,
    )
    assert command["relation"] == "right_of"
    assert "place_x:=0.450000" in args
    assert not any(arg.startswith("lower_min_distance:=") for arg in args)
    assert not any(arg.startswith("place_max_ik_solutions:=") for arg in args)


def test_pallet_scene_rejects_stale_or_wrong_frame(tmp_path):
    scene_path = tmp_path / "pallet.json"
    scene_path.write_text(json.dumps({
        "frame": "world", "unit": "m", "updated_at": 100.0,
    }), encoding="utf-8")
    import pytest
    with pytest.raises(ValueError, match="frame"):
        mtc_command_runner.load_pallet_scene(scene_path, now=101.0)
    data = json.loads(scene_path.read_text(encoding="utf-8"))
    data["frame"] = "base_footprint"
    scene_path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError, match="age"):
        mtc_command_runner.load_pallet_scene(scene_path, now=105.0)


def test_pallet_direct_place_uses_only_camera_detected_pick(tmp_path):
    scene_path = tmp_path / "pallet.json"
    scene_path.write_text(json.dumps({
        "frame": "base_footprint", "unit": "m", "updated_at": 100.0,
        "camera": {
            "eye": [-0.55, 0.65, 0.85], "target": [-0.3, 0.3, 0.025],
            "up": [0.0, 0.0, 1.0],
        },
        "objects": {
            "red_block": {"position": [-0.3, 0.3, 0.028], "size": [0.03, 0.03, 0.05], "confidence": 0.97},
        },
    }), encoding="utf-8")
    manager = mtc_command_runner.load_pallet_scene(scene_path, now=101.0)
    command, args = build_mtc_invocation(
        json.dumps({"action": "pick_place", "pick_object": "red_block",
                    "place_xyz": [0.3, -0.3, 0.025]}),
        provider="local", scene_manager=manager,
    )
    assert command["pick_object"] == "red_block"
    assert "object_z:=0.028000" in args
    assert "place_x:=0.300000" in args
    assert "place_y:=-0.300000" in args
    import pytest
    with pytest.raises(ValueError, match="finite"):
        build_mtc_invocation(
            json.dumps({"action": "pick_place", "pick_object": "red_block",
                        "place_xyz": [float("nan"), -0.3, 0.025]}),
            provider="local", scene_manager=manager,
        )
    data = json.loads(scene_path.read_text(encoding="utf-8"))
    data["objects"]["red_block"]["confidence"] = 0.3
    scene_path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError, match="confidence"):
        mtc_command_runner.load_pallet_scene(scene_path, now=101.0)


@pytest.mark.parametrize('release', [None, 0.020])
def test_pallet_cli_dry_run_prints_planning_only_launch(tmp_path, monkeypatch, capsys, release):
    import sys
    scene_path = tmp_path / "pallet.json"
    scene_path.write_text(json.dumps({
        "frame": "base_footprint", "unit": "m", "updated_at": time.time(),
        "camera": {
            "eye": [0.0, -1.0, 1.0], "target": [0.0, 0.0, 0.0],
            "up": [0.0, 0.0, 1.0],
        },
        "objects": {
            "red_block": {"position": [-0.3, 0.3, 0.025], "size": [0.03, 0.03, 0.05], "confidence": 0.97},
            "blue_block": {"position": [0.3, 0.3, 0.025], "size": [0.03, 0.03, 0.05], "confidence": 0.96},
        },
    }), encoding="utf-8")
    monkeypatch.setattr(sys, "argv", [
        "mtc_command_runner.py", "--robot-profile", "pallet_gripper",
        "--pallet-scene-file", str(scene_path), "--dry-run", "--once",
        "빨간 블록을 파란 블록 왼쪽에 놓아",
    ])
    if release is not None:
        monkeypatch.setattr(sys, 'argv', sys.argv + ['--release-open-position', str(release)])
    assert mtc_command_runner.main() == 0
    output = capsys.readouterr().out
    assert "robot_profile:=pallet_gripper" in output
    assert "planning_only:=true" in output
    if release is None:
        assert 'release_open_position:=' not in output
    else:
        assert 'release_open_position:=0.020000' in output
    assert "object_size_x:=0.030000" in output
    assert "place_x:=0.150000" in output


def test_release_open_cli_rejects_invalid_or_unrelated_profile(monkeypatch):
    import sys
    for profile, value in [('pallet_gripper', x) for x in ('nan', 'inf', '0', '-1', '0.034')] + [
            ('ur10e_robotiq', '0.020')]:
        monkeypatch.setattr(sys, 'argv', ['runner', '--robot-profile', profile,
                                         '--release-open-position', value])
        with pytest.raises(SystemExit) as error:
            mtc_command_runner.main()
        assert error.value.code == 2


def test_pallet_obstacles_enclose_unknown_orientation_and_require_complete_scene():
    import pytest
    manager = SceneManager.from_command_scene({
        name: {'position': [-0.32, y, 0.025], 'size': [0.03, 0.03, 0.05]}
        for name, y in [('red_block', 0.1), ('green_block', 0.02), ('blue_block', -0.06)]
    })
    items = mtc_command_runner.pallet_obstacle_description(manager, 'red_block')
    assert {x['id'] for x in items} == {'green_block', 'blue_block'}
    assert all(np.isclose(x['radius_m'], np.sqrt(0.03**2*2 + 0.05**2)/2) for x in items)
    incomplete = SceneManager.from_command_scene({'red_block': manager.as_command_scene()['red_block']})
    with pytest.raises(ValueError, match='all three fresh'):
        mtc_command_runner.pallet_obstacle_description(incomplete, 'red_block')


def test_pallet_cli_obstacles_forward_validated_complete_scene(tmp_path, monkeypatch, capsys):
    import sys
    scene_path = tmp_path / 'pallet.json'
    objects = {
        name: {'position': [-0.32, y, 0.025], 'size': [0.03, 0.03, 0.05], 'confidence': 0.95}
        for name, y in [('red_block', 0.1), ('green_block', 0.02), ('blue_block', -0.06)]
    }
    data = {'frame': 'base_footprint', 'unit': 'm', 'updated_at': time.time(),
            'camera': {'eye': [-0.55, 0.65, 0.85], 'target': [-0.3, 0.3, 0.025],
                       'up': [0, 0, 1]}, 'objects': objects}
    scene_path.write_text(json.dumps(data))
    monkeypatch.setattr(sys, 'argv', ['mtc_command_runner.py', '--robot-profile', 'pallet_gripper',
        '--pallet-scene-file', str(scene_path), '--scene-obstacles', '--dry-run',
        json.dumps({'action': 'pick_place', 'pick_object': 'red_block', 'place_xyz': [0.24, -0.28, 0.025]})])
    assert mtc_command_runner.main() == 0
    output = capsys.readouterr().out
    description = json.loads(output.split('pallet_scene_obstacles:=', 1)[1].strip())
    assert {x['id'] for x in description} == {'green_block', 'blue_block'}
    del data['objects']['green_block']
    scene_path.write_text(json.dumps(data))
    assert mtc_command_runner.main() == 1
    assert 'all three fresh' in capsys.readouterr().out


def test_pallet_cli_can_request_sim_execution(tmp_path, monkeypatch, capsys):
    import sys
    scene_path = tmp_path / "pallet.json"
    scene_path.write_text(json.dumps({
        "frame": "base_footprint", "unit": "m", "updated_at": time.time(),
        "camera": {"eye": [-0.55, 0.65, 0.85], "target": [-0.3, 0.3, 0.025],
                   "up": [0.0, 0.0, 1.0]},
        "objects": {"red_block": {"position": [-0.3, 0.3, 0.025],
                                  "size": [0.03, 0.03, 0.05], "confidence": 0.97}},
    }), encoding="utf-8")
    monkeypatch.setattr(sys, "argv", [
        "mtc_command_runner.py", "--robot-profile", "pallet_gripper",
        "--pallet-scene-file", str(scene_path), "--pallet-execution-mode", "standard",
        "--cartesian-transfer", "--upright-transfer", "--transfer-height", "0.10",
        "--totg-path-tolerance", "0.01",
        "--dry-run", json.dumps({"action": "pick_place", "pick_object": "red_block",
                                  "place_xyz": [0.3, -0.3, 0.025]}),
    ])
    assert mtc_command_runner.main() == 0
    output = capsys.readouterr().out
    assert "planning_only:=false" in output
    assert "totg_path_tolerance:=0.010000" in output
    assert "cartesian_transfer:=true" in output
    assert "upright_transfer:=true" in output
    assert "lift_min_distance:=0.100000" in output
    assert "lower_max_distance:=0.100000" in output
    assert "place_y:=-0.300000" in output
    assert "gripper_close_min:=0.000" in output
    assert "gripper_close_max:=0.000" in output
    assert "arm_velocity_scaling:=0.500" in output
    assert "arm_acceleration_scaling:=0.200" in output
    assert "cartesian_velocity_scaling:=0.350" in output
    assert "cartesian_acceleration_scaling:=0.200" in output


def test_pallet_physical_verification_requires_fresh_success(monkeypatch):
    expected = [0.3, -0.3, 0.025]
    summary = {
        "frame": "base_footprint", "unit": "m", "target": expected,
        "fresh_feedback": True, "placement_pass": True,
    }
    calls = []

    def completed(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0, stdout=json.dumps(summary) + "\n", stderr="")

    monkeypatch.setattr(mtc_command_runner.subprocess, "run", completed)
    assert mtc_command_runner.verify_pallet_physical_placement(expected) == 0
    assert calls[0][0][-4:] == ["--target", "0.300000", "-0.300000", "0.025000"]
    assert calls[0][0][calls[0][0].index("--pose-topic") + 1] == "/isaac_calibration_block_pose"
    assert calls[0][1]["timeout"] == 15
    summary["placement_pass"] = False
    assert mtc_command_runner.verify_pallet_physical_placement(expected) == 3
    summary["placement_pass"] = True
    summary["fresh_feedback"] = False
    assert mtc_command_runner.verify_pallet_physical_placement(expected) == 3


def test_pallet_block_pose_topic_supports_all_camera_blocks():
    assert pallet_block_pose_topic("red_block") == "/isaac_calibration_block_pose"
    assert pallet_block_pose_topic("green_block") == "/isaac_green_block_pose"
    assert pallet_block_pose_topic("blue_block") == "/isaac_blue_block_pose"


def test_pallet_standard_propagates_physical_failure(tmp_path, monkeypatch):
    import sys
    scene_path = tmp_path / "pallet.json"
    scene_path.write_text(json.dumps({
        "frame": "base_footprint", "unit": "m", "updated_at": time.time(),
        "camera": {"eye": [-0.55, 0.65, 0.85], "target": [-0.3, 0.3, 0.025],
                   "up": [0.0, 0.0, 1.0]},
        "objects": {"red_block": {"position": [-0.3, 0.3, 0.025],
                                  "size": [0.03, 0.03, 0.05], "confidence": 0.97}},
    }), encoding="utf-8")
    monkeypatch.setattr(sys, "argv", [
        "mtc_command_runner.py", "--robot-profile", "pallet_gripper",
        "--pallet-scene-file", str(scene_path), "--pallet-execution-mode", "standard",
        "--task-state-file", str(tmp_path / "task-state.json"),
        "--once", json.dumps({"action": "pick_place", "pick_object": "red_block",
                              "place_xyz": [0.3, -0.3, 0.025]}),
    ])
    monkeypatch.setattr(mtc_command_runner.subprocess, "run",
                        lambda *args, **kwargs: SimpleNamespace(returncode=0))
    targets = []
    monkeypatch.setattr(mtc_command_runner, "verify_pallet_physical_placement",
                        lambda target, object_id="red_block":
                        targets.append((object_id, target.tolist())) or 3)
    assert mtc_command_runner.main() == 2
    assert targets == [("red_block", [0.3, -0.3, 0.025])]
    task_state = json.loads((tmp_path / "task-state.json").read_text(encoding="utf-8"))
    assert task_state["state"] == "recovery_required"
    assert task_state["result"]["returncode"] == 3


def test_pallet_standard_uses_failure_detecting_launch_wrapper(tmp_path, monkeypatch):
    import sys
    scene_path = tmp_path / "pallet.json"
    scene_path.write_text(json.dumps({
        "frame": "base_footprint", "unit": "m", "updated_at": time.time(),
        "camera": {"eye": [-0.55, 0.65, 0.85], "target": [-0.3, 0.3, 0.025],
                   "up": [0.0, 0.0, 1.0]},
        "objects": {"green_block": {"position": [-0.23, 0.26, 0.025],
                                     "size": [0.03, 0.03, 0.05], "confidence": 0.97}},
    }), encoding="utf-8")
    wrapper = tmp_path / "checked-wrapper.sh"
    calls = []
    monkeypatch.setattr(sys, "argv", [
        "mtc_command_runner.py", "--robot-profile", "pallet_gripper",
        "--pallet-scene-file", str(scene_path), "--pallet-execution-mode", "standard",
        "--task-state-file", str(tmp_path / "task-state.json"),
        "--run-script", str(wrapper), "--once",
        json.dumps({"action": "pick_place", "pick_object": "green_block",
                    "place_xyz": [0.3, -0.3, 0.025]}),
    ])

    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=1)

    monkeypatch.setattr(mtc_command_runner.subprocess, "run", run)
    monkeypatch.setattr(
        mtc_command_runner, "verify_pallet_physical_placement",
        lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("must not verify")))
    assert mtc_command_runner.main() == 1
    assert calls[0][:2] == ["bash", str(wrapper)]


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
    assert "object_size_z:=0.051500" in mtc_args
    assert "object_z:=0.025750" in mtc_args
    assert "place_z:=0.025750" in mtc_args


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
