import pytest

import llm_to_json
from scene_config import build_scene_objects


def test_select_gemini_provider():
    assert llm_to_json.select_llm_provider(True, False) == "gemini"


def test_select_chatgpt_provider():
    assert llm_to_json.select_llm_provider(False, True) == "chatgpt"


def test_select_no_provider_uses_local_parser():
    assert llm_to_json.select_llm_provider(False, False) is None


def test_select_both_providers_raises_error():
    with pytest.raises(ValueError, match="동시에 True"):
        llm_to_json.select_llm_provider(True, True)


def test_parse_with_gemini_uses_3_1_flash_lite_by_default(monkeypatch):
    called = {}

    def fake_call(prompt, api_key, model):
        called.update(prompt=prompt, api_key=api_key, model=model)
        return "{}"

    monkeypatch.setenv("GEMINI_API_KEY", "test-key")
    monkeypatch.delenv("GEMINI_MODEL", raising=False)
    monkeypatch.setattr(llm_to_json, "_call_gemini_rest", fake_call)

    assert llm_to_json._parse_with_gemini("test prompt") == "{}"
    assert called["model"] == "gemini-3.1-flash-lite"


def test_parse_with_gemini_allows_model_override(monkeypatch):
    called = {}

    def fake_call(prompt, api_key, model):
        called["model"] = model
        return "{}"

    monkeypatch.setenv("GEMINI_API_KEY", "test-key")
    monkeypatch.setenv("GEMINI_MODEL", "custom-gemini-model")
    monkeypatch.setattr(llm_to_json, "_call_gemini_rest", fake_call)

    assert llm_to_json._parse_with_gemini("test prompt") == "{}"
    assert called["model"] == "custom-gemini-model"


def test_gemini_prompt_separates_object_selectors_from_placement_relation():
    prompt = llm_to_json.build_gemini_prompt(
        "왼쪽 물체를 오른쪽 물체 위로 옮겨",
        build_scene_objects(),
    )

    assert "leftmost object: blue_block" in prompt
    assert "middle object: green_block" in prompt
    assert "rightmost object: red_block" in prompt
    assert '"pick_object": "blue_block"' in prompt
    assert '"target_object": "red_block"' in prompt
    assert '"relation": "on"' in prompt
    assert "left/right does not specify relation" in prompt
    assert "Never infer\n   screen side directly from the sign of a world coordinate" in prompt


def test_gemini_provider_uses_gemini_specific_prompt(monkeypatch):
    response_text = (
        '{"action":"pick_place","pick_object":"blue_block",'
        '"target_object":"red_block","relation":"on","confidence":1.0}'
    )
    prompts = []

    def fake_parse(prompt):
        prompts.append(prompt)
        return response_text

    monkeypatch.setattr(llm_to_json, "_parse_with_gemini", fake_parse)

    command = llm_to_json.parse_user_command_with_llm(
        "왼쪽 물체를 오른쪽 물체 위로 옮겨",
        build_scene_objects(),
        provider="gemini",
    )

    assert "Camera-image horizontal references for this exact scene" in prompts[0]
    assert command["pick_object"] == "blue_block"
    assert command["target_object"] == "red_block"
    assert command["relation"] == "on"


@pytest.mark.parametrize("provider", ["gemini", "chatgpt"])
def test_parse_user_command_uses_selected_provider(monkeypatch, provider):
    response_text = (
        '{"action":"pick_place","pick_object":"red_block",'
        '"target_object":"blue_block","relation":"near","confidence":1.0}'
    )
    called = []

    def fake_parse(prompt):
        called.append(prompt)
        return response_text

    monkeypatch.setattr(llm_to_json, f"_parse_with_{provider}", fake_parse)

    command = llm_to_json.parse_user_command_with_llm(
        "빨간 블럭을 파란 블럭 옆에 둬",
        build_scene_objects(),
        provider=provider,
    )

    assert called
    assert command["pick_object"] == "red_block"
    assert command["target_object"] == "blue_block"
