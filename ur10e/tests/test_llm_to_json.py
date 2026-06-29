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
