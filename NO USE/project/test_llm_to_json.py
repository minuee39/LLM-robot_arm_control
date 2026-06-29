# test_llm_to_json.py

import os
import pytest
from pydantic import ValidationError

from llm_to_json import (
    PickPlaceAction,
    build_prompt,
    LLMJsonParser,
)


def sample_scene_objects():
    return [
        {
            "name": "blue block",
            "type": "block",
            "position": {"x": -0.10, "y": -0.45, "z": 0.03},
        },
        {
            "name": "green bowl",
            "type": "bowl",
            "position": {"x": 0.12, "y": -0.50, "z": 0.00},
        },
        {
            "name": "red bowl",
            "type": "bowl",
            "position": {"x": -0.20, "y": -0.55, "z": 0.00},
        },
    ]


def test_pick_place_action_valid():
    action = PickPlaceAction(
        action="pick_place",
        pick_object="blue block",
        target_object="green bowl",
        relation="on",
        offset_m=None,
        confidence=0.95,
    )

    assert action.action == "pick_place"
    assert action.pick_object == "blue block"
    assert action.target_object == "green bowl"
    assert action.relation == "on"
    assert action.confidence == 0.95


def test_pick_place_action_rejects_invalid_action():
    with pytest.raises(ValidationError):
        PickPlaceAction(
            action="move",
            pick_object="blue block",
            target_object="green bowl",
            relation="on",
            confidence=0.9,
        )


def test_pick_place_action_rejects_invalid_relation():
    with pytest.raises(ValidationError):
        PickPlaceAction(
            action="pick_place",
            pick_object="blue block",
            target_object="green bowl",
            relation="inside",
            confidence=0.9,
        )


def test_pick_place_action_rejects_invalid_confidence():
    with pytest.raises(ValidationError):
        PickPlaceAction(
            action="pick_place",
            pick_object="blue block",
            target_object="green bowl",
            relation="on",
            confidence=1.5,
        )


def test_build_prompt_contains_user_command_and_objects():
    scene_objects = sample_scene_objects()
    user_command = "Put the blue block on the green bowl"

    prompt = build_prompt(user_command, scene_objects)

    assert user_command in prompt
    assert "blue block" in prompt
    assert "green bowl" in prompt
    assert "red bowl" in prompt
    assert "Return only valid JSON" in prompt


def test_build_prompt_contains_leftmost_rule():
    prompt = build_prompt(
        "Put the blue block to the left of the leftmost bowl",
        sample_scene_objects(),
    )

    assert "Smaller x means more left" in prompt
    assert "Resolve phrases such as" in prompt


@pytest.mark.integration
def test_gemini_parse_simple_command():
    api_key = os.environ.get("GEMINI_API_KEY")

    if not api_key:
        pytest.skip("GEMINI_API_KEY is not set.")

    parser = LLMJsonParser(api_key=api_key)

    action = parser.parse(
        user_command="Put the blue block on the green bowl",
        scene_objects=sample_scene_objects(),
    )

    assert action.action == "pick_place"
    assert action.pick_object == "blue block"
    assert action.target_object == "green bowl"
    assert action.relation == "on"


@pytest.mark.integration
def test_gemini_parse_leftmost_bowl_command():
    api_key = os.environ.get("GEMINI_API_KEY")

    if not api_key:
        pytest.skip("GEMINI_API_KEY is not set.")

    parser = LLMJsonParser(api_key=api_key)

    action = parser.parse(
        user_command="Put the blue block to the left of the leftmost bowl",
        scene_objects=sample_scene_objects(),
    )

    assert action.action == "pick_place"
    assert action.pick_object == "blue block"
    assert action.target_object == "red bowl"
    assert action.relation == "left_of"