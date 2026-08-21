import re

import numpy as np

from scene_config import SIM_CAMERA_EYE, SIM_CAMERA_TARGET, SIM_CAMERA_UP


RELATION_DISTANCE = 0.15
NEAR_RELATION_DISTANCE = 0.12


def _planar_unit(vector, name: str) -> np.ndarray:
    planar = np.asarray(vector, dtype=float).copy()
    if planar.shape != (3,):
        raise ValueError(f"{name} must contain exactly three values")
    planar[2] = 0.0
    norm = float(np.linalg.norm(planar))
    if norm <= 1e-9:
        raise ValueError(f"{name} has no usable direction on the table plane")
    return planar / norm


def build_camera_relation_offsets(
    camera_eye=SIM_CAMERA_EYE,
    camera_target=SIM_CAMERA_TARGET,
    camera_up=SIM_CAMERA_UP,
) -> dict[str, np.ndarray]:
    """Build table-plane relations as seen from the active scene camera."""
    eye = np.asarray(camera_eye, dtype=float)
    target = np.asarray(camera_target, dtype=float)
    up = np.asarray(camera_up, dtype=float)
    if eye.shape != (3,) or target.shape != (3,) or up.shape != (3,):
        raise ValueError("camera eye, target, and up must be three-dimensional")

    view_forward = target - eye
    screen_right = _planar_unit(np.cross(view_forward, up), "camera right")
    toward_camera = _planar_unit(eye - target, "camera front")
    screen_left = -screen_right

    return {
        "on": np.array([0.0, 0.0, 0.105]),
        "left_of": screen_left * RELATION_DISTANCE,
        "right_of": screen_right * RELATION_DISTANCE,
        "front_of": toward_camera * RELATION_DISTANCE,
        "behind": -toward_camera * RELATION_DISTANCE,
        "near": (
            screen_left * NEAR_RELATION_DISTANCE
            + toward_camera * NEAR_RELATION_DISTANCE
        ),
    }


# left/right follow the camera image; front means closer to the camera.
RELATION_OFFSETS = build_camera_relation_offsets()

SUPPORTED_ACTIONS = {"pick_place"}
SUPPORTED_OBJECTS = {"red_block", "blue_block", "green_block"}
SUPPORTED_RELATIONS = {"on", "left_of", "right_of", "front_of", "behind", "near"}

MIN_CONFIDENCE = 0.7

OBJECT_ALIASES = {
    "파란 블럭": "blue_block",
    "파란 블록": "blue_block",
    "파란블럭": "blue_block",
    "파란블록": "blue_block",
    "파란색 블럭": "blue_block",
    "파란색 블록": "blue_block",
    "blue block": "blue_block",
    "blue_block": "blue_block",
    "blue": "blue_block",

    "빨간 블럭": "red_block",
    "빨간 블록": "red_block",
    "빨간블럭": "red_block",
    "빨간블록": "red_block",
    "빨간색 블럭": "red_block",
    "빨간색 블록": "red_block",
    "red block": "red_block",
    "red_block": "red_block",
    "red": "red_block",

    "초록 블럭": "green_block",
    "초록 블록": "green_block",
    "초록블럭": "green_block",
    "초록블록": "green_block",
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

MEMORY_OBJECT_ALIASES = {
    "방금 옮긴 블럭": "last_moved_object",
    "방금 옮긴 블록": "last_moved_object",
    "마지막으로 옮긴 블럭": "last_moved_object",
    "마지막으로 옮긴 블록": "last_moved_object",
    "방금 옮긴 것": "last_moved_object",
    "마지막으로 옮긴 것": "last_moved_object",

    "방금 집은 블럭": "last_picked_object",
    "방금 집은 블록": "last_picked_object",
    "마지막으로 집은 블럭": "last_picked_object",
    "마지막으로 집은 블록": "last_picked_object",
    "방금 집은 것": "last_picked_object",
    "과거에 집은 물체": "last_picked_object",
    "과거에 집은 블럭": "last_picked_object",
    "과거에 집은 블록": "last_picked_object",
    "전에 집은 물체": "last_picked_object",
    "전에 집은 블럭": "last_picked_object",
    "전에 집은 블록": "last_picked_object",
    "이전에 집은 물체": "last_picked_object",
    "이전에 집은 블럭": "last_picked_object",
    "이전에 집은 블록": "last_picked_object",

    "마지막 대상": "last_target_object",
    "마지막 타겟": "last_target_object",
    "방금 대상": "last_target_object",
    "방금 목표": "last_target_object",
}

SPATIAL_OBJECT_ALIASES = {
    "제일 오른쪽 물체": "rightmost",
    "제일 오른쪽 블럭": "rightmost",
    "제일 오른쪽 블록": "rightmost",
    "가장 오른쪽 물체": "rightmost",
    "가장 오른쪽 블럭": "rightmost",
    "가장 오른쪽 블록": "rightmost",
    "제일 왼쪽 물체": "leftmost",
    "제일 왼쪽 블럭": "leftmost",
    "제일 왼쪽 블록": "leftmost",
    "가장 왼쪽 물체": "leftmost",
    "가장 왼쪽 블럭": "leftmost",
    "가장 왼쪽 블록": "leftmost",
    "오른쪽 물체": "rightmost",
    "오른쪽 블럭": "rightmost",
    "오른쪽 블록": "rightmost",
    "왼쪽 물체": "leftmost",
    "왼쪽 블럭": "leftmost",
    "왼쪽 블록": "leftmost",
    "가운데 물체": "middle",
    "가운데 블럭": "middle",
    "가운데 블록": "middle",
    "중간 물체": "middle",
    "중간 블럭": "middle",
    "중간 블록": "middle",
}

OBJECT_MENTION_PATTERN = re.compile(
    r"[가-힣]+(?:색)?\s*(?:물체|블럭|블록)|"
    r"[a-z]+(?:[ _-]+)(?:object|block)"
)


def _raise_invalid_object_slot(text: str, found_objects: list[tuple]) -> None:
    """Report a bad pick or target separately when both slots are present."""
    recognized_spans = sorted(
        (idx, idx + len(alias), object_name)
        for idx, alias, object_name in found_objects
    )

    # Multiple aliases can describe the same occurrence (for example "blue"
    # inside "blue block"). Keep one recognized mention per overlapping span.
    recognized_mentions = []
    for start, end, object_name in recognized_spans:
        overlapping = next(
            (
                mention
                for mention in recognized_mentions
                if start < mention[1] and end > mention[0]
            ),
            None,
        )
        if overlapping is None:
            recognized_mentions.append([start, end, object_name, True, text[start:end]])
        else:
            overlapping[0] = min(overlapping[0], start)
            overlapping[1] = max(overlapping[1], end)

    mentions = list(recognized_mentions)
    for match in OBJECT_MENTION_PATTERN.finditer(text):
        if any(match.start() < end and match.end() > start for start, end, *_ in mentions):
            continue
        mentions.append([match.start(), match.end(), None, False, match.group(0)])

    mentions.sort(key=lambda mention: mention[0])
    if len(mentions) < 2:
        return

    pick_mention, target_mention = mentions[:2]
    if not pick_mention[3] and target_mention[3]:
        raise ValueError(f"pick_object를 이해하지 못했습니다: {pick_mention[4]}")
    if pick_mention[3] and not target_mention[3]:
        raise ValueError(f"target_object를 이해하지 못했습니다: {target_mention[4]}")


def resolve_spatial_object(selector: str, scene_objects: dict) -> str:
    """Resolve horizontal selectors in the active camera image coordinates."""
    candidates = []
    screen_right = RELATION_OFFSETS["right_of"] / RELATION_DISTANCE

    for name, info in scene_objects.items():
        if name not in SUPPORTED_OBJECTS:
            continue
        position = np.asarray(info["position"], dtype=float)
        if position.shape != (3,):
            raise ValueError(f"{name}의 position은 3차원이어야 합니다.")
        candidates.append((float(np.dot(position, screen_right)), name))

    if not candidates:
        raise ValueError("공간 참조를 판단할 scene object가 없습니다.")

    candidates.sort(key=lambda item: (item[0], item[1]))
    if selector == "leftmost":
        return candidates[0][1]
    if selector == "rightmost":
        return candidates[-1][1]
    if selector == "middle":
        return candidates[(len(candidates) - 1) // 2][1]
    raise ValueError(f"지원하지 않는 공간 참조입니다: {selector}")


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
    
def parse_user_command(user_text: str, scene_objects: dict | None = None) -> dict:
    return _parse_user_command(user_text, memory=None, scene_objects=scene_objects)


def parse_user_command_with_memory(
    user_text: str,
    memory,
    scene_objects: dict | None = None,
) -> dict:
    return _parse_user_command(user_text, memory=memory, scene_objects=scene_objects)


def _parse_user_command(user_text: str, memory=None, scene_objects=None) -> dict:
    text = user_text.strip().lower()
    # 음성 인식이 색상 음절 사이에 넣는 공백은 의미가 없으므로 정규화한다.
    for spaced, normalized in (("파 란", "파란"), ("빨 간", "빨간"), ("초 록", "초록")):
        text = text.replace(spaced, normalized)
    found_objects = []
    reference_spans = []

    for alias, object_name in OBJECT_ALIASES.items():
        idx = text.find(alias)
        if idx != -1:
            found_objects.append((idx, alias, object_name))

    if memory is not None:
        for alias, memory_field in MEMORY_OBJECT_ALIASES.items():
            idx = text.find(alias)
            if idx != -1:
                object_name = getattr(memory, memory_field, None)
                if not object_name:
                    raise ValueError(f"참조할 이전 작업 기억이 없습니다: {alias}")
                found_objects.append((idx, alias, object_name))
                reference_spans.append((idx, idx + len(alias)))

    for alias, selector in SPATIAL_OBJECT_ALIASES.items():
        idx = text.find(alias)
        if idx == -1:
            continue
        if scene_objects is None:
            raise ValueError(f"공간 참조를 판단할 scene 정보가 없습니다: {alias}")
        object_name = resolve_spatial_object(selector, scene_objects)
        found_objects.append((idx, alias, object_name))
        reference_spans.append((idx, idx + len(alias)))

    # 문장에 등장한 순서대로 정렬
    found_objects.sort(key=lambda x: x[0])

    # 같은 object가 여러 alias로 중복 잡히는 것 제거
    unique_objects = []
    for _, _, object_name in found_objects:
        if not unique_objects or unique_objects[-1] != object_name:
            unique_objects.append(object_name)

    if len(unique_objects) < 2:
        _raise_invalid_object_slot(text, found_objects)
        mention_positions = {idx for idx, _, _ in found_objects}
        if len(mention_positions) >= 2 and unique_objects:
            raise ValueError(f"pick_object와 target_object가 같습니다: {unique_objects[0]}")
        raise ValueError(f"pick_object와 target_object를 모두 이해하지 못했습니다: {user_text}")

    pick_object = unique_objects[0]
    target_object = unique_objects[1]

    if pick_object == target_object:
        raise ValueError(f"pick_object와 target_object가 같습니다: {pick_object}")

    # "제일 오른쪽 물체"의 오른쪽을 배치 relation으로 다시 읽지 않는다.
    relation_text = "".join(
        " " if any(start <= idx < end for start, end in reference_spans) else char
        for idx, char in enumerate(text)
    )

    if "위" in relation_text or "올려" in relation_text or "on" in relation_text:
        relation = "on"
    elif "왼쪽" in relation_text or "left" in relation_text:
        relation = "left_of"
    elif "오른쪽" in relation_text or "right" in relation_text:
        relation = "right_of"
    elif "앞" in relation_text or "front" in relation_text or "forward" in relation_text:
        relation = "front_of"
    elif "뒤" in relation_text or "back" in relation_text or "behind" in relation_text:
        relation = "behind"
    elif "근처" in relation_text or "near" in relation_text or "옆" in relation_text:
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
