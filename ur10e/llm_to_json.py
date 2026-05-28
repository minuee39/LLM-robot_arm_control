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
- If the user says "옆", "근처", or "near", use relation "near".
- If the user says "위", "올려", or "on", use relation "on".
- If the user says "왼쪽" or "left", use relation "left_of".
- If the user says "오른쪽" or "right", use relation "right_of".
- If the user says "앞" or "front", use relation "front_of".
- If the user says "뒤" or "behind", use relation "behind".
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


def parse_user_command_with_llm(user_text: str, scene_objects: dict) -> dict:
    api_key = os.environ.get("GEMINI_API_KEY")
    if not api_key:
        raise ValueError("GEMINI_API_KEY 환경변수가 설정되지 않았습니다.")

    model = os.environ.get("GEMINI_MODEL", "gemini-2.5-flash")

    prompt = build_prompt(user_text, scene_objects)
    response_text = _call_gemini_rest(prompt, api_key, model)

    try:
        raw_json = json.loads(response_text)
    except json.JSONDecodeError as error:
        raise ValueError(f"LLM 응답을 JSON으로 파싱하지 못했습니다: {response_text}") from error

    try:
        parsed = PickPlaceAction.model_validate(raw_json)
    except ValidationError as error:
        raise ValueError(f"LLM JSON schema 검증 실패: {raw_json}") from error

    return parsed.model_dump(exclude_none=True)