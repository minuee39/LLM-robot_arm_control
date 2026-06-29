import numpy as np


# isaac sim 기준
RELATION_OFFSETS = {
    "on": np.array([0.0, 0.0, 0.06]),
    "left_of": np.array([0.15, -0.15, 0.0]),
    "right_of": np.array([-0.15, 0.15, 0.0]),
    "front_of": np.array([+0.15, +0.15, 0.0]),
    "behind": np.array([-0.15, -0.15, 0.0]),
    "near": np.array([0.12, 0.12, 0.0]),
}

SUPPORTED_ACTIONS = {"pick_place"}
SUPPORTED_OBJECTS = {"red_block", "blue_block", "green_block"}
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
    text = user_text.strip().lower()

    object_aliases = {
        
        "파란 블럭": "blue_block",
        "파란 블록": "blue_block",
        "파란색 블럭": "blue_block",
        "파란색 블록": "blue_block",
        "blue block": "blue_block",
        "blue_block": "blue_block",
        "blue": "blue_block",

        "빨간 블럭": "red_block",
        "빨간 블록": "red_block",
        "빨간색 블럭": "red_block",
        "빨간색 블록": "red_block",
        "red block": "red_block",
        "red_block": "red_block",
        "red": "red_block",

        "초록 블럭": "green_block",
        "초록 블록": "green_block",
        "초록색 블럭": "green_block",
        "초록색 블록": "green_block",
        "green block": "green_block",
        "green_block": "green_block",
        "green": "green_block",

        "큐브": "cube",
        "cube": "cube",

        "목표": "target",
        "target": "target",

        "그릇": "bowl",
        "보울": "bowl",
        "bowl": "bowl",
    }

    found_objects = []

    for alias, object_name in object_aliases.items():
        idx = text.find(alias)
        if idx != -1:
            found_objects.append((idx, alias, object_name))

    # 문장에 등장한 순서대로 정렬
    found_objects.sort(key=lambda x: x[0])

    # 같은 object가 여러 alias로 중복 잡히는 것 제거
    unique_objects = []
    for _, _, object_name in found_objects:
        if not unique_objects or unique_objects[-1] != object_name:
            unique_objects.append(object_name)

    if len(unique_objects) < 2:
        raise ValueError(f"pick_object와 target_object를 모두 이해하지 못했습니다: {user_text}")

    pick_object = unique_objects[0]
    target_object = unique_objects[1]

    if pick_object == target_object:
        raise ValueError(f"pick_object와 target_object가 같습니다: {pick_object}")

    if "위" in text or "올려" in text or "on" in text:
        relation = "on"
    elif "왼쪽" in text or "left" in text:
        relation = "left_of"
    elif "오른쪽" in text or "right" in text:
        relation = "right_of"
    elif "앞" in text or "front" in text or "forward" in text:
        relation = "front_of"
    elif "뒤" in text or "back" in text or "behind" in text:
        relation = "behind"
    elif "근처" in text or "near" in text or "옆" in text:
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

