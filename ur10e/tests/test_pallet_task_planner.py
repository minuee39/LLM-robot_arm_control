import json

import pytest

import pallet_task_planner
from mtc_command_runner import PalletSceneManager


def test_gemini_task_plan_is_validated(monkeypatch):
    response = {
        "version": 1,
        "frame": "base_footprint",
        "unit": "m",
        "tasks": [
            {"action": "pick_place", "pick_object": "red_block",
             "place_xyz": [0.24, -0.28, 0.025]},
            {"action": "pick_place", "pick_object": "blue_block",
             "place_xyz": [0.28, -0.28, 0.025]},
        ],
    }
    monkeypatch.setattr(
        pallet_task_planner, "_parse_with_gemini",
        lambda unused_prompt: json.dumps(response),
    )

    spec = pallet_task_planner.compile_task_spec_with_gemini(
        "빨간 블록은 왼쪽 아래, 파란 블록은 오른쪽 아래")

    assert [task["pick_object"] for task in spec["tasks"]] == [
        "red_block", "blue_block"]


def test_gemini_task_plan_rejects_out_of_contract_output(monkeypatch):
    monkeypatch.setattr(
        pallet_task_planner, "_parse_with_gemini",
        lambda unused_prompt: '{"error":"ambiguous_or_unsafe"}',
    )
    with pytest.raises(ValueError, match="rejected"):
        pallet_task_planner.compile_task_spec_with_gemini("아무거나 해")


def test_task_prompt_forbids_execution_parameters():
    prompt = pallet_task_planner.build_task_spec_prompt("빠르게 옮겨")
    assert "Never emit controller settings, speeds, shell commands" in prompt
    assert "gemini" not in prompt.lower()


def test_relative_request_preserves_llm_relation_until_execution(monkeypatch):
    scene = {
        "green_block": {
            "position": [-0.32, -0.18, 0.025], "size": [0.03, 0.03, 0.05],
            "confidence": 0.9,
        },
        "blue_block": {
            "position": [-0.32, 0.05, 0.025], "size": [0.03, 0.03, 0.05],
            "confidence": 0.9,
        },
    }
    manager = PalletSceneManager(scene, {
        "on": [0.0, 0.0, 0.105],
        "left_of": [0.06, 0.0, 0.0],
        "right_of": [-0.06, 0.0, 0.0],
        "front_of": [0.0, 0.06, 0.0],
        "behind": [0.0, -0.06, 0.0],
        "near": [0.06, 0.0, 0.0],
    })
    monkeypatch.setattr(
        pallet_task_planner,
        "parse_user_command_with_llm",
        lambda *args, **kwargs: {
            "action": "pick_place", "pick_object": "green_block",
            "target_object": "blue_block", "relation": "near", "confidence": 1.0,
        },
    )
    spec = pallet_task_planner.compile_task_spec_with_gemini(
        "초록 블럭을 파란 블럭 옆에 둬", scene_manager=manager)

    assert spec["tasks"] == [{
        "action": "pick_place",
        "pick_object": "green_block",
        "target_object": "blue_block",
        "relation": "near",
        "confidence": 1.0,
    }]


def test_relative_request_requires_live_scene():
    with pytest.raises(ValueError, match="current camera scene"):
        pallet_task_planner.compile_task_spec_with_gemini(
            "초록 블럭을 파란 블럭 옆에 둬")
