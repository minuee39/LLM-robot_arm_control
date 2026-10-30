"""Validate declarative job specifications before camera/MTC execution."""

import json
import argparse
import math
from pathlib import Path

OBJECTS = {"red_block", "green_block", "blue_block"}
RELATIONS = {"on", "left_of", "right_of", "front_of", "behind", "near"}


def validate_task_spec(spec):
    """Validate and normalize an untrusted versioned Pallet task document.

    This layer accepts intent only. Controller parameters and executable shell
    commands remain owned by the validated execution layer.
    """
    if not isinstance(spec, dict) or set(spec) != {"version", "frame", "unit", "tasks"}:
        raise ValueError("spec requires exactly version, frame, unit, tasks")
    if type(spec["version"]) is not int or spec["version"] != 1:
        raise ValueError("unsupported task spec version")
    if spec["frame"] != "base_footprint" or spec["unit"] != "m":
        raise ValueError("task coordinates require base_footprint and metres")
    if not isinstance(spec["tasks"], list) or not 1 <= len(spec["tasks"]) <= 3:
        raise ValueError("spec requires one to three tasks")
    placements = {}
    normalized_tasks = []
    for task in spec["tasks"]:
        if not isinstance(task, dict):
            raise ValueError("each task must be an object")
        direct_keys = {"action", "pick_object", "place_xyz"}
        relation_keys = {"action", "pick_object", "target_object", "relation"}
        relation_confidence_keys = relation_keys | {"confidence"}
        task_keys = frozenset(task)
        if task_keys not in {
                frozenset(direct_keys), frozenset(relation_keys),
                frozenset(relation_confidence_keys)}:
            raise ValueError(
                "task requires either action/pick_object/place_xyz or "
                "action/pick_object/target_object/relation")
        name = task["pick_object"]
        if task["action"] != "pick_place" or not isinstance(name, str) or name not in OBJECTS:
            raise ValueError("unsupported action or object")
        if name in placements:
            raise ValueError("each object may appear only once per job")
        if task_keys in {frozenset(relation_keys), frozenset(relation_confidence_keys)}:
            target_name = task["target_object"]
            relation = task["relation"]
            confidence = task.get("confidence", 1.0)
            if target_name not in OBJECTS or target_name == name:
                raise ValueError("relative task requires a different supported target object")
            if relation not in RELATIONS:
                raise ValueError("unsupported relative placement relation")
            if (isinstance(confidence, bool) or not isinstance(confidence, (int, float))
                    or not math.isfinite(confidence) or not 0.7 <= confidence <= 1.0):
                raise ValueError("relative task confidence must be between 0.7 and 1.0")
            placements[name] = None
            normalized_tasks.append({
                "action": "pick_place",
                "pick_object": name,
                "target_object": target_name,
                "relation": relation,
                "confidence": float(confidence),
            })
            continue
        xyz = task["place_xyz"]
        if (not isinstance(xyz, list) or len(xyz) != 3 or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in xyz)):
            raise ValueError("place_xyz requires three finite numbers")
        if not (-0.85 <= xyz[0] <= 0.85 and -0.35 <= xyz[1] <= 0.35 and
                abs(xyz[2] - 0.025) <= 1e-6):
            raise ValueError("destination is outside the tabletop block-centre envelope")
        if any(abs(xyz[0] - p[0]) < 0.04 - 1e-9 and
               abs(xyz[1] - p[1]) < 0.04 - 1e-9 for p in placements.values()):
            raise ValueError("destinations require 40 mm axis separation")
        placements[name] = list(map(float, xyz))
        normalized_tasks.append({
            "action": "pick_place", "pick_object": name,
            "place_xyz": placements[name],
        })
    return {
        "version": 1, "frame": "base_footprint", "unit": "m",
        "tasks": normalized_tasks,
    }


def load_task_spec(path):
    return validate_task_spec(json.loads(Path(path).read_text(encoding="utf-8")))


def load_placements(path):
    """Compile a validated task document to ordered base-frame placements."""
    spec = load_task_spec(path)
    placements = {}
    for task in spec["tasks"]:
        if "place_xyz" not in task:
            raise ValueError("relative TaskSpec cannot be compiled without a live scene")
        placements[task["pick_object"]] = task["place_xyz"]
    return placements


def load_tasks(path):
    """Return ordered, validated direct or object-relative tasks."""
    return load_task_spec(path)["tasks"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("spec", type=Path)
    args = parser.parse_args()
    try:
        print(json.dumps(load_placements(args.spec), ensure_ascii=False))
    except (ValueError, OSError) as error:
        parser.exit(2, f"Invalid task spec: {error}\n")
