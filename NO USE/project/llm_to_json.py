# llm_to_json.py

import json
import os
from typing import Literal, Optional

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
            raise ValueError("offset_m must contain exactly 3 values")

        if any(abs(v) > 0.5 for v in value):
            raise ValueError("offset_m values must be within ±0.5m")

        return value


def parse_user_command_with_llm(user_text: str) -> dict:
    try:
        from google import genai
        from google.genai import types
    except ModuleNotFoundError as e:
        raise ValueError(
            "google-genai 패키지가 Isaac Sim Python 환경에 설치되어 있지 않습니다. "
            "설치 명령: /home/minwoo/isaacsim/python.sh -m pip install google-genai"
        ) from e

    api_key = os.environ.get("GEMINI_API_KEY")
    if not api_key:
        raise ValueError("GEMINI_API_KEY 환경변수가 설정되지 않았습니다.")

    client = genai.Client(api_key=api_key)

    prompt = f"""
You are a robot command parser.

Convert the user's Korean or English robot command into JSON.

Available objects:
- red_block
- blue_block
- green_block

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
- Output JSON only.
- Do not include markdown.
- pick_object is the object to move.
- target_object is the reference object.
- If the user says "옆", use relation "near".
- Use exact object names from the available object list.

User command:
{user_text}
"""

    response = client.models.generate_content(
        model="gemini-1.5-flash",
        contents=prompt,
        config=types.GenerateContentConfig(
            temperature=0.0,
            response_mime_type="application/json",
        ),
    )

    try:
        raw_json = json.loads(response.text)
    except json.JSONDecodeError as e:
        raise ValueError(f"LLM 응답을 JSON으로 파싱하지 못했습니다: {response.text}") from e

    try:
        parsed = PickPlaceAction.model_validate(raw_json)
    except ValidationError as e:
        raise ValueError(f"LLM JSON schema 검증 실패: {e}") from e

    return parsed.model_dump(exclude_none=True)