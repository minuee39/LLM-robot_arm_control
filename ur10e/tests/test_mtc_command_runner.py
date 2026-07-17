import numpy as np

from mtc_command_runner import build_mtc_invocation, command_to_mtc_args, execute_user_command
from scene_manager import SceneManager


def test_build_mtc_invocation_from_local_command():
    command, mtc_args = build_mtc_invocation("물체를 파란 블럭 옆에 둬", provider="local")

    assert command["pick_object"] == "object"
    assert command["target_object"] == "blue_block"
    assert command["relation"] == "near"
    assert "object_x:=0.700000" in mtc_args
    assert "object_y:=0.400000" in mtc_args
    assert "place_x:=0.420000" in mtc_args
    assert "place_y:=0.420000" in mtc_args


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

    assert "object_z:=0.050000" in mtc_args
    assert "place_z:=0.050000" in mtc_args


def test_scene_manager_defaults_include_mtc_object():
    scene_manager = SceneManager.from_defaults()

    assert "object" in scene_manager.names()
    np.testing.assert_allclose(scene_manager.get_object("object").position, np.array([0.70, 0.40, 0.05]))


def test_execute_user_command_dry_run_keeps_memory_unchanged():
    scene_manager = SceneManager.from_defaults()

    result = execute_user_command(
        "물체를 파란 블럭 옆에 둬",
        scene_manager=scene_manager,
        provider="local",
        run_script="/unused",
        dry_run=True,
    )

    assert result == 0
    assert scene_manager.memory.last_moved_object is None


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

    result = execute_user_command(
        "물체를 파란 블럭 옆에 둬",
        scene_manager=scene_manager,
        provider="local",
        run_script="/run_mtc",
    )

    assert result == 0
    assert calls
    assert scene_manager.memory.last_moved_object == "object"
    np.testing.assert_allclose(scene_manager.get_object("object").position, np.array([0.42, 0.42, 0.05]))
