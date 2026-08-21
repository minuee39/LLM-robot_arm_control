# llm_to_json.py

import json
import os
import urllib.request
import urllib.error
from typing import Literal, Optional

import numpy as np
from pydantic import BaseModel, Field, ValidationError, field_validator


class PickPlaceAction(BaseModel):
    action: Literal["pick_place"]
    pick_object: str
    target_object: str
    relation: Literal[
        "on",
        "left_of",
        "right_of",
        "front_of",
        "behind",
        "near",
    ]

    offset_m: Optional[list[float]] = None
    confidence: float = Field(default=1.0, ge=0.0, le=1.0)

    @field_validator("offset_m")
    @classmethod
    def validate_offset(cls, value):
        if value is None:
            return value

        if len(value) != 3:
            raise ValueError("offset_m must contain exactly 3 values: [x, y, z].")

        if any(abs(v) > 0.5 for v in value):
            raise ValueError("offset_m value is too large for safe execution.")

        return value


LLMProvider = Literal["gemini", "chatgpt"]


def select_llm_provider(
    use_gemini_api: bool,
    use_chatgpt_api: bool,
) -> Optional[LLMProvider]:
    if use_gemini_api and use_chatgpt_api:
        raise ValueError(
            "USE_GEMINI_API와 USE_CHATGPT_API를 동시에 True로 설정할 수 없습니다."
        )

    if use_gemini_api:
        return "gemini"

    if use_chatgpt_api:
        return "chatgpt"

    return None


def scene_objects_to_llm_list(scene_objects: dict) -> list[dict]:
    llm_objects = []

    for object_name, info in scene_objects.items():
        position = np.array(info["position"], dtype=float)

        llm_objects.append(
            {
                "name": object_name,
                "type": "block",
                "position": {
                    "x": float(position[0]),
                    "y": float(position[1]),
                    "z": float(position[2]),
                },
            }
        )

    return llm_objects


def build_prompt(user_command: str, scene_objects: dict) -> str:
    llm_scene_objects = scene_objects_to_llm_list(scene_objects)

    return f"""
You are a robot task parser.

Convert the user's Korean or English natural language command into a JSON action.

Available objects:
{json.dumps(llm_scene_objects, ensure_ascii=False, indent=2)}

Available action:
- pick_place

Available relations:
- on
- left_of
- right_of
- front_of
- behind
- near

Rules:
- Return only valid JSON.
- Do not include markdown.
- Do not include explanations.
- Use only object names from Available objects.
- The only supported action is "pick_place".
- If the command is ambiguous, set confidence below 0.7.
- If the pick object or target object cannot be clearly resolved, set confidence below 0.7.
- If you are confident the command is executable, set confidence between 0.7 and 1.0.
- Interpret left and right from the camera image. Interpret front as closer to the camera.
- Use exact object names such as "red_block", "blue_block", "green_block".
- Do not output names like "red block" or "blue block".

User command:
{user_command}

Example output:
{{
  "action": "pick_place",
  "pick_object": "red_block",
  "target_object": "blue_block",
  "relation": "near",
  "confidence": 1.0
}}
""".strip()


def build_gemini_prompt(user_command: str, scene_objects: dict) -> str:
    """Build the Gemini-specific prompt with deterministic camera references."""
    from command_parser import resolve_spatial_object

    llm_scene_objects = scene_objects_to_llm_list(scene_objects)
    leftmost_object = resolve_spatial_object("leftmost", scene_objects)
    middle_object = resolve_spatial_object("middle", scene_objects)
    rightmost_object = resolve_spatial_object("rightmost", scene_objects)

    return f"""
You are a deterministic robot pick-and-place command parser.

Convert one Korean or English user command into exactly one JSON object.
Do not plan robot motion. Only identify the object to pick, the destination
object, and their final placement relation.

Available objects:
{json.dumps(llm_scene_objects, ensure_ascii=False, indent=2)}

Camera-image horizontal references for this exact scene:
- leftmost object: {leftmost_object}
- middle object: {middle_object}
- rightmost object: {rightmost_object}

Output fields:
- action: always "pick_place"
- pick_object: the object that the robot must grasp and move
- target_object: the stationary reference object at the destination
- relation: one of "on", "left_of", "right_of", "front_of", "behind", "near"
- confidence: a number from 0.0 to 1.0

Interpretation rules, in priority order:
1. Resolve the pick_object and target_object before resolving relation.
2. In Korean, the object marked by "을/를" is normally pick_object. The object
   followed by the destination phrase such as "위에", "왼쪽에", or "옆에" is
   target_object. Preserve these roles; never swap source and destination.
3. "왼쪽 물체", "오른쪽 물체", "left object", and "right object" are
   object selectors. Resolve them only from the camera-image reference list
   above. In these noun phrases, left/right does not specify relation.
4. A left/right word specifies relation only when it describes the final
   destination relative to an already identified target object. For example,
   "빨간 블럭을 파란 블럭 왼쪽에 둘" means pick red_block,
   target blue_block, relation left_of.
5. Choose relation from the final destination phrase, not from words inside an
   object selector:
   - "위", "위에", "위로", "올려", "on", "on top of" -> "on"
   - "왼쪽에", "left of" -> "left_of"
   - "오른쪽에", "right of" -> "right_of"
   - "앞에", "front of" -> "front_of"
   - "뒤에", "behind" -> "behind"
   - "옆에", "근처에", "near" -> "near"
6. Camera-image left/right is not the same as world-coordinate X. Never infer
   screen side directly from the sign of a world coordinate.
7. Use only exact names from Available objects. Never invent an object name.
8. If either object cannot be resolved uniquely, or the command has conflicting
   roles, set confidence below 0.7. Otherwise use confidence from 0.7 to 1.0.
9. Return JSON only. Do not include markdown or explanations.

Examples for the current scene:
User: "왼쪽 물체를 오른쪽 물체 위로 옮겨"
Output:
{{
  "action": "pick_place",
  "pick_object": "{leftmost_object}",
  "target_object": "{rightmost_object}",
  "relation": "on",
  "confidence": 1.0
}}

User: "오른쪽 물체를 가운데 물체 옆에 놓아"
Output:
{{
  "action": "pick_place",
  "pick_object": "{rightmost_object}",
  "target_object": "{middle_object}",
  "relation": "near",
  "confidence": 1.0
}}

User command:
{user_command}
""".strip()


def _extract_text_from_gemini_response(response_json: dict) -> str:
    try:
        return response_json["candidates"][0]["content"]["parts"][0]["text"]
    except (KeyError, IndexError, TypeError) as error:
        raise ValueError(f"Gemini 응답 형식이 예상과 다릅니다: {response_json}") from error


def _call_gemini_rest(prompt: str, api_key: str, model: str) -> str:
    url = (
        "https://generativelanguage.googleapis.com/v1beta/"
        f"models/{model}:generateContent?key={api_key}"
    )

    payload = {
        "contents": [
            {
                "role": "user",
                "parts": [
                    {
                        "text": prompt
                    }
                ],
            }
        ],
        "generationConfig": {
            "temperature": 0,
            "responseMimeType": "application/json",
        },
    }

    request = urllib.request.Request(
        url=url,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    try:
        with urllib.request.urlopen(request, timeout=20) as response:
            response_json = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        body = error.read().decode("utf-8", errors="replace")
        raise ValueError(f"Gemini API HTTP 오류: {error.code}, {body}") from error
    except urllib.error.URLError as error:
        raise ValueError(f"Gemini API 연결 오류: {error}") from error

    return _extract_text_from_gemini_response(response_json)


def _extract_text_from_chatgpt_response(response_json: dict) -> str:
    try:
        return response_json["choices"][0]["message"]["content"]
    except (KeyError, IndexError, TypeError) as error:
        raise ValueError(f"ChatGPT 응답 형식이 예상과 다릅니다: {response_json}") from error


def _call_chatgpt_rest(prompt: str, api_key: str, model: str) -> str:
    payload = {
        "model": model,
        "messages": [
            {
                "role": "user",
                "content": prompt,
            }
        ],
        "temperature": 0,
        "response_format": {"type": "json_object"},
    }

    request = urllib.request.Request(
        url="https://api.openai.com/v1/chat/completions",
        data=json.dumps(payload).encode("utf-8"),
        headers={
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
        },
        method="POST",
    )

    try:
        with urllib.request.urlopen(request, timeout=20) as response:
            response_json = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        body = error.read().decode("utf-8", errors="replace")
        raise ValueError(f"ChatGPT API HTTP 오류: {error.code}, {body}") from error
    except urllib.error.URLError as error:
        raise ValueError(f"ChatGPT API 연결 오류: {error}") from error

    return _extract_text_from_chatgpt_response(response_json)


def parse_user_command_with_llm(
    user_text: str,
    scene_objects: dict,
    provider: LLMProvider = "gemini",
) -> dict:
    if provider == "gemini":
        prompt = build_gemini_prompt(user_text, scene_objects)
        response_text = _parse_with_gemini(prompt)
    elif provider == "chatgpt":
        prompt = build_prompt(user_text, scene_objects)
        response_text = _parse_with_chatgpt(prompt)
    else:
        raise ValueError(f"지원하지 않는 LLM 제공자입니다: {provider}")

    try:
        raw_json = json.loads(response_text)
    except json.JSONDecodeError as error:
        raise ValueError(f"LLM 응답을 JSON으로 파싱하지 못했습니다: {response_text}") from error

    try:
        parsed = PickPlaceAction.model_validate(raw_json)
    except ValidationError as error:
        raise ValueError(f"LLM JSON schema 검증 실패: {raw_json}") from error

    return parsed.model_dump(exclude_none=True)


def _parse_with_gemini(prompt: str) -> str:
    api_key = os.environ.get("GEMINI_API_KEY")
    if not api_key:
        raise ValueError("GEMINI_API_KEY 환경변수가 설정되지 않았습니다.")

    model = os.environ.get("GEMINI_MODEL", "gemini-3.1-flash-lite")
    return _call_gemini_rest(prompt, api_key, model)


def _parse_with_chatgpt(prompt: str) -> str:
    api_key = os.environ.get("OPENAI_API_KEY") or os.environ.get("CHATGPT_API_KEY")
    if not api_key:
        raise ValueError(
            "OPENAI_API_KEY 또는 CHATGPT_API_KEY 환경변수가 설정되지 않았습니다."
        )

    model = os.environ.get("OPENAI_MODEL", "gpt-4.1-mini")
    return _call_chatgpt_rest(prompt, api_key, model)
