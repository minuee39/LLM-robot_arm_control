import numpy as np
import pytest
from types import SimpleNamespace

from command_parser import (
    RELATION_OFFSETS,
    build_camera_relation_offsets,
    parse_user_command,
    parse_user_command_with_memory,
    validate_command,
    command_to_target_position,
)

from scene_config import build_scene_objects


def test_relation_offsets_follow_current_camera_view():
    np.testing.assert_allclose(RELATION_OFFSETS["left_of"], [0.15, 0.0, 0.0])
    np.testing.assert_allclose(RELATION_OFFSETS["right_of"], [-0.15, 0.0, 0.0])
    np.testing.assert_allclose(RELATION_OFFSETS["front_of"], [0.0, 0.15, 0.0])
    np.testing.assert_allclose(RELATION_OFFSETS["behind"], [0.0, -0.15, 0.0])
    np.testing.assert_allclose(RELATION_OFFSETS["near"], [0.12, 0.12, 0.0])


def test_relation_offsets_rotate_with_camera_view():
    offsets = build_camera_relation_offsets(
        camera_eye=[1.0, 0.0, 1.0],
        camera_target=[0.0, 0.0, 0.0],
        camera_up=[0.0, 0.0, 1.0],
    )

    np.testing.assert_allclose(offsets["left_of"], [0.0, -0.15, 0.0])
    np.testing.assert_allclose(offsets["right_of"], [0.0, 0.15, 0.0])
    np.testing.assert_allclose(offsets["front_of"], [0.15, 0.0, 0.0])
    np.testing.assert_allclose(offsets["behind"], [-0.15, 0.0, 0.0])


def test_parse_red_to_blue_near():
    command = parse_user_command("빨간 블럭을 파란 블럭 옆에 둬")

    assert command["action"] == "pick_place"
    assert command["pick_object"] == "red_block"
    assert command["target_object"] == "blue_block"
    assert command["relation"] == "near"


def test_parse_red_to_blue_near_with_user_wording():
    command = parse_user_command("빨간색 블럭을 파란색 블럭 옆에 놔")

    assert command["action"] == "pick_place"
    assert command["pick_object"] == "red_block"
    assert command["target_object"] == "blue_block"
    assert command["relation"] == "near"


def test_parse_command_with_particle_typo_and_split_color():
    command = parse_user_command("빨간블럭을를 파 란 블럭 위에 둬")

    assert command["pick_object"] == "red_block"
    assert command["target_object"] == "blue_block"
    assert command["relation"] == "on"


def test_parse_blue_to_red_near():
    command = parse_user_command("파란 블럭을 빨간 블럭 옆에 둬")

    assert command["pick_object"] == "blue_block"
    assert command["target_object"] == "red_block"
    assert command["relation"] == "near"


def test_parse_green_to_blue_left():
    command = parse_user_command("초록 블럭을 파란 블럭 왼쪽에 둬")

    assert command["pick_object"] == "green_block"
    assert command["target_object"] == "blue_block"
    assert command["relation"] == "left_of"


def test_invalid_pick_object_error_identifies_only_pick_slot():
    with pytest.raises(ValueError, match=r"^pick_object를 이해하지 못했습니다: 노란 블럭$"):
        parse_user_command("노란 블럭을 파란 블럭 위에 둬")


def test_invalid_target_object_error_identifies_only_target_slot():
    with pytest.raises(ValueError, match=r"^target_object를 이해하지 못했습니다: 노란 블럭$"):
        parse_user_command("빨간 블럭을 노란 블럭 위에 둬")


def test_validate_valid_command():
    scene_objects = build_scene_objects()

    command = {
        "action": "pick_place",
        "pick_object": "red_block",
        "target_object": "blue_block",
        "relation": "near",
        "confidence": 1.0,
    }

    validate_command(command, scene_objects)


def test_validate_same_object_error():
    scene_objects = build_scene_objects()

    command = {
        "action": "pick_place",
        "pick_object": "red_block",
        "target_object": "red_block",
        "relation": "near",
        "confidence": 1.0,
    }

    with pytest.raises(ValueError):
        validate_command(command, scene_objects)


def test_unknown_object_error():
    scene_objects = build_scene_objects()

    command = {
        "action": "pick_place",
        "pick_object": "yellow_block",
        "target_object": "blue_block",
        "relation": "near",
        "confidence": 1.0,
    }

    with pytest.raises(ValueError):
        validate_command(command, scene_objects)


def test_command_to_target_position():
    scene_objects = build_scene_objects()

    command = {
        "action": "pick_place",
        "pick_object": "red_block",
        "target_object": "blue_block",
        "relation": "near",
        "confidence": 1.0,
    }

    target_position = command_to_target_position(command, scene_objects)

    assert target_position.shape == (3,)


def test_parse_memory_reference_for_last_moved_object():
    memory = SimpleNamespace(
        last_moved_object="red_block",
        last_picked_object=None,
        last_target_object=None,
        last_relation=None,
    )

    command = parse_user_command_with_memory("방금 옮긴 블럭을 초록 블럭 위에 올려", memory)

    assert command["pick_object"] == "red_block"
    assert command["target_object"] == "green_block"
    assert command["relation"] == "on"


def test_parse_memory_reference_for_last_target_object():
    memory = SimpleNamespace(
        last_moved_object=None,
        last_picked_object=None,
        last_target_object="blue_block",
        last_relation=None,
    )

    command = parse_user_command_with_memory("빨간 블럭을 마지막 대상 옆에 둬", memory)

    assert command["pick_object"] == "red_block"
    assert command["target_object"] == "blue_block"
    assert command["relation"] == "near"


def test_parse_memory_reference_without_memory_error():
    memory = SimpleNamespace(
        last_moved_object=None,
        last_picked_object=None,
        last_target_object=None,
        last_relation=None,
    )

    with pytest.raises(ValueError, match="참조할 이전 작업 기억이 없습니다"):
        parse_user_command_with_memory("방금 옮긴 블럭을 초록 블럭 위에 올려", memory)


def test_parse_memory_reference_same_object_error():
    memory = SimpleNamespace(
        last_moved_object="green_block",
        last_picked_object=None,
        last_target_object=None,
        last_relation=None,
    )

    with pytest.raises(ValueError, match="pick_object와 target_object가 같습니다"):
        parse_user_command_with_memory("방금 옮긴 블럭을 초록 블럭 옆에 둬", memory)


def test_parse_past_picked_object_to_rightmost_object():
    memory = SimpleNamespace(
        last_moved_object=None,
        last_picked_object="blue_block",
        last_target_object=None,
        last_relation=None,
    )
    scene_objects = build_scene_objects()

    command = parse_user_command_with_memory(
        "과거에 집은 물체를 제일 오른쪽 물체 옆에 둬",
        memory,
        scene_objects,
    )

    assert command["pick_object"] == "blue_block"
    # The current scene camera sees negative world X on the image's right.
    assert command["target_object"] == "red_block"
    assert command["relation"] == "near"


def test_parse_rightmost_object_as_pick_object():
    memory = SimpleNamespace(
        last_moved_object=None,
        last_picked_object=None,
        last_target_object=None,
        last_relation=None,
    )

    command = parse_user_command_with_memory(
        "제일 오른쪽 물체를 초록 블록 위에 올려",
        memory,
        build_scene_objects(),
    )

    assert command["pick_object"] == "red_block"
    assert command["target_object"] == "green_block"
    assert command["relation"] == "on"


def test_parse_left_object_onto_right_object():
    command = parse_user_command(
        "왼쪽 물체를 오른쪽 물체 위로 옮겨",
        build_scene_objects(),
    )

    # From the configured camera, positive world X is the image's left side.
    assert command["pick_object"] == "blue_block"
    assert command["target_object"] == "red_block"
    assert command["relation"] == "on"


def test_spatial_reference_requires_scene_objects():
    memory = SimpleNamespace(
        last_moved_object=None,
        last_picked_object=None,
        last_target_object=None,
        last_relation=None,
    )

    with pytest.raises(ValueError, match="scene 정보가 없습니다"):
        parse_user_command_with_memory(
            "제일 오른쪽 물체를 초록 블록 위에 올려",
            memory,
        )
