import json

import pytest

from pallet_task_spec import load_placements, load_tasks


def test_compile_spec_and_reject_unsafe_contracts(tmp_path):
    path = tmp_path / "spec.json"
    spec = {"version": 1, "frame": "base_footprint", "unit": "m", "tasks": [
        {"action": "pick_place", "pick_object": "blue_block",
         "place_xyz": [0.28, -0.28, 0.025]}]}
    path.write_text(json.dumps(spec))
    assert load_placements(path) == {"blue_block": [0.28, -0.28, 0.025]}
    for replacement in (True, float("nan"), 2.0):
        spec["tasks"][0]["place_xyz"][0] = replacement
        path.write_text(json.dumps(spec))
        with pytest.raises(ValueError):
            load_placements(path)
    spec["tasks"][0]["place_xyz"][0] = 0.28
    spec["tasks"].append({"action": "pick_place", "pick_object": "red_block",
                          "place_xyz": [0.29, -0.28, 0.025]})
    path.write_text(json.dumps(spec))
    with pytest.raises(ValueError, match="separation"):
        load_placements(path)


def test_relative_task_preserves_target_and_relation(tmp_path):
    path = tmp_path / "relative.json"
    spec = {"version": 1, "frame": "base_footprint", "unit": "m", "tasks": [{
        "action": "pick_place",
        "pick_object": "green_block",
        "target_object": "blue_block",
        "relation": "near",
    }]}
    path.write_text(json.dumps(spec))

    assert load_tasks(path) == [{**spec["tasks"][0], "confidence": 1.0}]
    with pytest.raises(ValueError, match="live scene"):
        load_placements(path)
