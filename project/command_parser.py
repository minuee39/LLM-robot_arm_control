import numpy as np


def parse_user_command(user_text: str) -> dict:
    """
    임시 명령 파서.
    나중에 이 부분을 LLM JSON 출력으로 교체하면 됨.
    """

    user_text = user_text.strip()

    if "왼쪽" in user_text:
        target_position = [-0.5, 0.6, 0.02575]
    elif "오른쪽" in user_text:
        target_position = [-0.1, 0.6, 0.02575]
    elif "앞" in user_text:
        target_position = [-0.3, 0.8, 0.02575]
    elif "뒤" in user_text:
        target_position = [-0.3, 0.4, 0.02575]
    else:
        target_position = [-0.3, 0.6, 0.02575]

    return {
        "action": "pick_place",
        "pick_object": "cube",
        "target_position": target_position,
        "confidence": 1.0,
    }


def command_to_target_position(command: dict) -> np.ndarray:
    if command["action"] != "pick_place":
        raise ValueError(f"Unsupported action: {command['action']}")

    if "target_position" not in command:
        raise ValueError("target_position is required")

    target_position = np.array(command["target_position"], dtype=float)

    if target_position.shape != (3,):
        raise ValueError("target_position must be [x, y, z]")

    return target_position