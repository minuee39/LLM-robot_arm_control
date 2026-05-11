import numpy as np


RELATION_OFFSETS = {
    "on": np.array([0.0, 0.0, 0.12]),
    "left_of": np.array([0.0, 0.15, 0.0]),
    "right_of": np.array([0.0, -0.15, 0.0]),
    "front_of": np.array([0.15, 0.0, 0.0]),
    "behind": np.array([-0.15, 0.0, 0.0]),
    "near": np.array([0.12, 0.12, 0.0]),
}

SUPPORTED_ACTIONS = {"pick_place"}
SUPPORTED_OBJECTS = {"cube", "target", "red_block", "blue_block", "bowl"}
SUPPORTED_RELATIONS = {"on", "left_of", "right_of", "front_of", "behind", "near"}

MIN_CONFIDENCE = 0.7


def validate_command(command: dict, scene_objects: dict) -> None:
    action = command.get("action")
    pick_object = command.get("pick_object")
    target_object = command.get("target_object")
    relation = command.get("relation")
    confidence = command.get("confidence", 0.0)

    if action not in SUPPORTED_ACTIONS:
        raise ValueError(f"지원하지 않는 action입니다: {action}")

    if pick_object not in SUPPORTED_OBJECTS:
        raise ValueError(f"지원하지 않는 pick_object입니다: {pick_object}")

    if target_object not in SUPPORTED_OBJECTS:
        raise ValueError(f"지원하지 않는 target_object입니다: {target_object}")

    if pick_object not in scene_objects:
        raise ValueError(f"scene에 없는 pick_object입니다: {pick_object}")

    if target_object not in scene_objects:
        raise ValueError(f"scene에 없는 target_object입니다: {target_object}")

    if pick_object == target_object:
        raise ValueError("pick_object와 target_object가 같습니다.")

    if relation not in SUPPORTED_RELATIONS:
        raise ValueError(f"지원하지 않는 relation입니다: {relation}")

    if confidence < MIN_CONFIDENCE:
        raise ValueError(f"명령 신뢰도가 낮습니다: {confidence}")
    
    
    
def parse_user_command(user_text: str) -> dict:
    user_text = user_text.strip().lower()

    # 1. 집을 물체 인식

    if "cube" in user_text or "큐브" in user_text or "블럭" in user_text:
        pick_object = "cube"
    else:
        raise ValueError(f"집을 물체를 이해하지 못했습니다: {user_text}")

    # 2. 기준 물체 인식
    if "target" in user_text or "목표" in user_text:
        target_object = "target"
    elif "blue" in user_text or "파란" in user_text:
        target_object = "blue_block"
    elif "red" in user_text or "빨간" in user_text:
        target_object = "red_block"
    elif "bowl" in user_text or "그릇" in user_text:
        target_object = "bowl"
    else:
        raise ValueError(f"기준 물체를 이해하지 못했습니다: {user_text}")

    # 같은 물체를 집고 같은 물체 기준으로 놓는 명령 방지
    if pick_object == target_object:
        raise ValueError(f"pick_object와 target_object가 같습니다: {pick_object}")

    # 3. 공간 관계 인식
    if "위" in user_text or "올려" in user_text or "on" in user_text:
        relation = "on"
    elif "왼쪽" in user_text or "left" in user_text:
        relation = "left_of"
    elif "오른쪽" in user_text or "right" in user_text:
        relation = "right_of"
    elif "앞" in user_text or "front" in user_text or "forward" in user_text:
        relation = "front_of"
    elif "뒤" in user_text or "back" in user_text or "behind" in user_text:
        relation = "behind"
    elif "근처" in user_text or "near" in user_text or "옆" in user_text:
        relation = "near"
    else:
        raise ValueError(f"공간 관계를 이해하지 못했습니다: {user_text}")

    return {
        "action": "pick_place",
        "pick_object": pick_object,
        "target_object": target_object,
        "relation": relation,
        "confidence": 1.0,
    }


def command_to_target_position(command: dict, scene_objects: dict) -> np.ndarray:
    if command.get("action") != "pick_place":
        raise ValueError(f"지원하지 않는 action입니다: {command.get('action')}")

    target_object = command.get("target_object")
    relation = command.get("relation")

    if target_object not in scene_objects:
        raise ValueError(f"scene에 없는 target_object입니다: {target_object}")

    if relation not in RELATION_OFFSETS:
        raise ValueError(f"지원하지 않는 relation입니다: {relation}")

    target_object_position = np.array(scene_objects[target_object]["position"], dtype=float)
    target_position = target_object_position + RELATION_OFFSETS[relation]

    return target_position

