import numpy as np

from scene_manager import SceneManager
from vision.base import Detection


def test_scene_manager_defaults_include_blocks():
    manager = SceneManager.from_defaults()

    assert manager.names() == {"object", "red_block", "blue_block", "green_block"}
    scene = manager.as_command_scene()
    assert scene["red_block"]["position"].shape == (3,)
    assert scene["red_block"]["size"].shape == (3,)


def test_scene_manager_update_object_changes_command_scene():
    manager = SceneManager.from_defaults()
    new_position = np.array([0.1, 0.2, 0.3])
    original_size = manager.get_object("red_block").size

    manager.update_object("red_block", new_position, confidence=0.9)

    scene = manager.as_command_scene()
    np.testing.assert_allclose(scene["red_block"]["position"], new_position)
    np.testing.assert_allclose(scene["red_block"]["size"], original_size)
    assert scene["red_block"]["confidence"] == 0.9


def test_scene_manager_update_from_detections():
    manager = SceneManager.from_defaults()
    detection = Detection("red_block", np.array([0.4, 0.5, 0.6]), 0.8)

    manager.update_from_detections({"red_block": detection})

    scene = manager.as_command_scene()
    np.testing.assert_allclose(scene["red_block"]["position"], detection.position)
    assert scene["red_block"]["confidence"] == 0.8


def test_scene_manager_resolves_target_position():
    manager = SceneManager.from_defaults()
    command = {
        "action": "pick_place",
        "pick_object": "red_block",
        "target_object": "blue_block",
        "relation": "near",
        "confidence": 1.0,
    }

    target_position = manager.resolve_target_position(command)

    np.testing.assert_allclose(
        target_position,
        manager.get_object("blue_block").position + np.array([0.12, 0.12, 0.0]),
    )


def test_scene_manager_apply_pick_place_result_updates_memory():
    manager = SceneManager.from_defaults()
    command = {
        "action": "pick_place",
        "pick_object": "red_block",
        "target_object": "blue_block",
        "relation": "near",
        "confidence": 1.0,
    }
    final_position = np.array([0.42, 0.24, 0.08])

    manager.apply_pick_place_result(command, final_position)

    np.testing.assert_allclose(manager.get_object("red_block").position, final_position)
    assert manager.memory.last_picked_object == "red_block"
    assert manager.memory.last_moved_object == "red_block"
    assert manager.memory.last_target_object == "blue_block"
    assert manager.memory.last_relation == "near"


def test_scene_manager_resolve_missing_target_error():
    manager = SceneManager.from_defaults()
    command = {
        "action": "pick_place",
        "pick_object": "red_block",
        "target_object": "yellow_block",
        "relation": "near",
        "confidence": 1.0,
    }

    try:
        manager.resolve_target_position(command)
    except ValueError as exc:
        assert "scene에 없는 target_object" in str(exc)
    else:
        raise AssertionError("Expected missing target object to raise ValueError")
