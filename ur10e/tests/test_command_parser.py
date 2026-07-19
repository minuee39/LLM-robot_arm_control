import pytest
from types import SimpleNamespace

from command_parser import (
    parse_user_command,
    parse_user_command_with_memory,
    validate_command,
    command_to_target_position,
)

from scene_config import build_scene_objects


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
