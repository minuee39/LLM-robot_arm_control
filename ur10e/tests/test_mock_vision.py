import pytest

from command_parser import command_to_target_position, validate_command
from vision.mock_vision import MockVision
from vision.scene_objects import detections_to_scene_objects


def scene_objects_from_mock_vision() -> dict:
    vision = MockVision()
    detections = vision.detect_objects()
    return detections_to_scene_objects(detections)


def test_mock_vision_detects_blocks():
    vision = MockVision()
    detections = vision.detect_objects()

    assert "red_block" in detections
    assert "blue_block" in detections
    assert "green_block" in detections
    assert detections["red_block"].confidence >= 0.9


def test_mock_vision_can_drive_command_target_position():
    scene_objects = scene_objects_from_mock_vision()

    command = {
        "action": "pick_place",
        "pick_object": "blue_block",
        "target_object": "green_block",
        "relation": "near",
        "confidence": 1.0,
    }

    validate_command(command, scene_objects)
    target_position = command_to_target_position(command, scene_objects)

    assert target_position.shape == (3,)


def test_validate_command_fails_when_target_missing_from_mock_vision_scene():
    scene_objects = scene_objects_from_mock_vision()
    scene_objects.pop("green_block")

    command = {
        "action": "pick_place",
        "pick_object": "blue_block",
        "target_object": "green_block",
        "relation": "near",
        "confidence": 1.0,
    }

    with pytest.raises(ValueError):
        validate_command(command, scene_objects)


def test_mock_vision_updates_runtime_object_position():
    vision = MockVision()
    new_position = [0.42, -0.10, 0.02575]

    vision.update_object_position("blue_block", new_position)
    detections = vision.detect_objects()

    assert detections["blue_block"].position.tolist() == new_position


def test_detections_to_scene_objects_copies_detection_positions():
    vision = MockVision()
    detections = vision.detect_objects()
    scene_objects = detections_to_scene_objects(detections)

    detections["red_block"].position[0] = 999.0

    assert scene_objects["red_block"]["position"][0] != 999.0
