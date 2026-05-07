# llm_to_json.py

import json
import os
from typing import Literal, Optional

from pydantic import BaseModel, Field, ValidationError
from google import genai
from google.genai import types


class PickPlaceAction(BaseModel):
    action: Literal["pick_place"] = Field(
        description="Only supported robot action."
    )
    pick_object: str = Field(
        description="Object to pick, e.g. 'blue block'."
    )
    target_object: str = Field(
        description="Reference target object, e.g. 'green bowl'."
    )
    relation: Literal[
        "on",
        "left_of",
        "right_of",
        "front_of",
        "behind",
        "near"
    ] = Field(
        description="Spatial relation between picked object and target object."
    )
    offset_m: Optional[list[float]] = Field(
        default=None,
        description="Optional xyz offset from target object in meters."
    )
    confidence: float = Field(
        default=1.0,
        ge=0.0,
        le=1.0,
        description="Confidence score from 0 to 1."
    )

MIN_CONFIDENCE = 0.7


def validate_action_for_execution(action: PickPlaceAction) -> None:
    """
    Validate whether the parsed action is safe enough to execute.

    Raises:
        ValueError: If the action should not be executed.
    """
    if action.confidence < MIN_CONFIDENCE:
        raise ValueError(
            f"Action confidence is too low: {action.confidence}. "
            f"Minimum required confidence is {MIN_CONFIDENCE}."
        )

    if not action.pick_object:
        raise ValueError("pick_object is missing.")

    if not action.target_object:
        raise ValueError("target_object is missing.")

    if not action.relation:
        raise ValueError("relation is missing.")
    
    
def build_prompt(user_command: str, scene_objects: list[dict]) -> str:
    return f"""
You are a robot task parser.

Convert the user's natural language command into a JSON action.

Available objects:
{json.dumps(scene_objects, ensure_ascii=False, indent=2)}

Rules:
- Return only valid JSON.
- Do not include markdown.
- Do not include explanations.
- Use only object names from Available objects.
- The only supported action is "pick_place".
- If the command is ambiguous, set confidence below 0.7.
- If the pick object or target object cannot be clearly resolved, set confidence below 0.7.
- If you are confident the command is executable, set confidence between 0.7 and 1.0.
- If the user says "put A in B", use relation "on".
- If the user says "put A on B", use relation "on".
- If the user says "to the left of B", use relation "left_of".
- If the user says "to the right of B", use relation "right_of".
- If the user says "in front of B", use relation "front_of".
- If the user says "behind B", use relation "behind".
- If the user says "near B", use relation "near".
- Resolve phrases such as "leftmost bowl" using object positions.
- Smaller x means more left.
- Larger x means more right.

User command:
{user_command}
""".strip()


class LLMJsonParser:
    def __init__(self, api_key: str, model: str = "gemini-2.5-flash"):
        if not api_key:
            raise ValueError("Gemini API key is required.")

        self.client = genai.Client(api_key=api_key)
        self.model = model

    def parse(self, user_command: str, scene_objects: list[dict]) -> PickPlaceAction:
        prompt = build_prompt(user_command, scene_objects)

        response = self.client.models.generate_content(
            model=self.model,
            contents=prompt,
            config=types.GenerateContentConfig(
                temperature=0,
                response_mime_type="application/json",
                response_schema=PickPlaceAction,
            ),
        )

        try:
            data = json.loads(response.text)
            return PickPlaceAction(**data)
        except (json.JSONDecodeError, ValidationError) as error:
            raise ValueError(f"Invalid LLM JSON output: {response.text}") from error


if __name__ == "__main__":
    scene_objects = [
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

    parser = LLMJsonParser(
        api_key=os.environ.get("GEMINI_API_KEY")
    )

    action = parser.parse(
    "Put the blue block to the left of the leftmost bowl",
    scene_objects,
    )

    validate_action_for_execution(action)

    print(action.model_dump_json(indent=2))