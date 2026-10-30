import importlib.util
import ast
from collections import deque
import inspect
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "capture_pallet_rgbd.py"
spec = importlib.util.spec_from_file_location("capture_pallet_rgbd", SCRIPT)
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


def test_camera_revision_does_not_include_object_truth(tmp_path):
    path = tmp_path / "calibration.json"
    data = {"camera": {"optical_to_base_footprint": np.eye(4).tolist()},
            "objects": [{"name": "red_block", "position": [1, 2, 3]}]}
    path.write_text(json.dumps(data))
    transform, revision = capture.camera_calibration_only(path)
    data["objects"] = [{"name": "red_block", "position": [4, 5, 6]}]
    path.write_text(json.dumps(data))
    np.testing.assert_array_equal(transform, np.eye(4))
    assert capture.camera_calibration_only(path)[1] == revision
    data["camera"]["optical_to_base_footprint"][0][3] = .1
    path.write_text(json.dumps(data))
    assert capture.camera_calibration_only(path)[1] != revision


@pytest.mark.parametrize("flag,value", [
    ("--timeout", "0"), ("--timeout", "nan"), ("--timeout", "61"),
    ("--sync-slop", "-1"), ("--sync-slop", "nan"),
])
def test_invalid_cli_before_ros_initialization(tmp_path, flag, value):
    result = subprocess.run([sys.executable, str(SCRIPT), "--output-dir",
                             str(tmp_path/"new"), "--calibration", "missing",
                             flag, value], capture_output=True, text=True)
    assert result.returncode == 2
    assert not (tmp_path/"new").exists()


def test_existing_output_not_overwritten(tmp_path):
    result = subprocess.run([sys.executable, str(SCRIPT), "--output-dir", str(tmp_path),
                             "--calibration", "missing"], capture_output=True, text=True)
    assert result.returncode == 2
    assert "refusing to overwrite" in result.stderr


def test_camera_info_callback_is_inspectable_and_bounded():
    tree = ast.parse(SCRIPT.read_text())
    callback = next(n for n in ast.walk(tree)
                    if isinstance(n, ast.FunctionDef) and n.name == "remember_camera_info")
    namespace = {"info": deque(maxlen=1)}
    module = ast.Module(body=[callback], type_ignores=[])
    exec(compile(module, str(SCRIPT), "exec"), namespace)
    remember = namespace["remember_camera_info"]
    inspect.signature(remember).bind(object())
    remember("old")
    remember("new")
    assert list(namespace["info"]) == ["new"]
