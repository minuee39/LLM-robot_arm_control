#!/usr/bin/env python3
"""Compile a natural-language palletizing request into a validated TaskSpec."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from command_parser import parse_user_command_with_memory, validate_command
from llm_to_json import _parse_with_gemini, parse_user_command_with_llm
from pallet_task_spec import validate_task_spec


SAFE_SLOTS = {
    "bottom_left": [0.24, -0.28, 0.025],
    "bottom_right": [0.28, -0.28, 0.025],
    "top_left": [0.24, -0.24, 0.025],
    "top_right": [0.28, -0.24, 0.025],
}

RELATIVE_DESTINATION_CUES = (
    "옆에", "옆으로", "근처에", "왼쪽에", "오른쪽에", "앞에", "뒤에",
    "위에", "위로", "올려", "next to", "beside", "near", "left of",
    "right of", "in front of", "behind", "on top of",
)


def is_relative_destination_request(user_text: str) -> bool:
    normalized = " ".join(user_text.lower().split())
    return any(cue in normalized for cue in RELATIVE_DESTINATION_CUES)


def compile_relative_task_spec(user_text: str, scene_manager) -> dict:
    """Resolve one object-relative request against the current camera scene."""
    scene_objects = scene_manager.as_command_scene()
    try:
        command = parse_user_command_with_llm(
            user_text, scene_objects, provider="gemini")
    except ValueError:
        command = parse_user_command_with_memory(
            user_text,
            scene_manager.memory,
            scene_objects,
            screen_right=scene_manager.relation_offsets["right_of"],
        )
    validate_command(command, scene_objects)
    return validate_task_spec({
        "version": 1,
        "frame": "base_footprint",
        "unit": "m",
        "tasks": [{
            "action": "pick_place",
            "pick_object": command["pick_object"],
            "target_object": command["target_object"],
            "relation": command["relation"],
            "confidence": command["confidence"],
        }],
    })


def build_task_spec_prompt(user_text: str) -> str:
    return f"""
You are a deterministic palletizing intent compiler.
Convert the Korean or English request into one versioned JSON TaskSpec.

Available objects: red_block, green_block, blue_block.
Allowed destination slots in base_footprint metres:
{json.dumps(SAFE_SLOTS, ensure_ascii=False, indent=2)}

Return exactly this structure and no markdown or explanation:
{{
  "version": 1,
  "frame": "base_footprint",
  "unit": "m",
  "tasks": [
    {{"action": "pick_place", "pick_object": "red_block", "place_xyz": [0.24, -0.28, 0.025]}}
  ]
}}

Rules:
- Produce one to three tasks in the requested order.
- Use each object at most once.
- Use only the exact object names and exact coordinates from Allowed destination slots.
- Never emit controller settings, speeds, shell commands, ROS parameters, or extra fields.
- Interpret bottom/아래 as y=-0.28, top/위 as y=-0.24,
  left/왼쪽 as x=0.24, and right/오른쪽 as x=0.28.
- If the request is ambiguous or unsafe, return {{"error":"ambiguous_or_unsafe"}}.

User request:
{user_text}
""".strip()


def compile_task_spec_with_gemini(user_text: str, *, scene_manager=None) -> dict:
    if not isinstance(user_text, str) or not user_text.strip():
        raise ValueError("task request must be nonempty")
    user_text = user_text.strip()
    if is_relative_destination_request(user_text):
        if scene_manager is None:
            raise ValueError("relative placement requires a current camera scene")
        return compile_relative_task_spec(user_text, scene_manager)
    response_text = _parse_with_gemini(build_task_spec_prompt(user_text))
    try:
        raw = json.loads(response_text)
    except json.JSONDecodeError as error:
        raise ValueError("Gemini task response was not valid JSON") from error
    if isinstance(raw, dict) and "error" in raw:
        raise ValueError(f"Gemini rejected the task request: {raw['error']}")
    return validate_task_spec(raw)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("request", nargs="+", help="Natural-language palletizing request")
    parser.add_argument("--output", type=Path, help="Optional validated TaskSpec output")
    parser.add_argument("--scene-file", type=Path, help="Current Pallet vision scene for relative placement")
    args = parser.parse_args()
    try:
        scene_manager = None
        if args.scene_file:
            from mtc_command_runner import load_pallet_scene
            scene_manager = load_pallet_scene(args.scene_file)
        spec = compile_task_spec_with_gemini(
            " ".join(args.request), scene_manager=scene_manager)
    except ValueError as error:
        parser.exit(2, f"Task planning failed: {error}\n")
    serialized = json.dumps(spec, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
