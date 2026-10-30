import ast
from pathlib import Path
import subprocess
import sys

import pytest


SCRIPT = Path(__file__).resolve().parents[1]/"scripts/live_pallet_pose_diagnostic.py"


def test_rgb_depth_info_are_synchronized_and_latest_is_diagnostic_only():
    tree = ast.parse(SCRIPT.read_text())
    sync = next(n for n in ast.walk(tree) if isinstance(n, ast.Call)
                and isinstance(n.func, ast.Name) and n.func.id == "ApproximateTimeSynchronizer")
    assert [n.id for n in sync.args[0].elts] == ["rgb_sub", "depth_sub", "info_sub"]
    assert ast.literal_eval(sync.args[1]) == 3
    assert ast.literal_eval(sync.args[2]) == .01
    pair = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "pair")
    assert [n.arg for n in pair.args.args] == ["rgb", "depth", "ci"]
    assignments = [n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                   and any(isinstance(t, ast.Name) and t.id == "latest" for t in n.targets)]
    kwargs = {k.arg: k.value for k in assignments[0].value.keywords}
    assert ast.literal_eval(kwargs["diagnostic_only"]) is True
    assert ast.literal_eval(kwargs["collision_model_approved"]) is False


@pytest.mark.parametrize("extra", [
    ["--seconds", "nan"], ["--seconds", "0"], ["--seconds", "601"],
    ["--ransac-trials", "0"], ["--ransac-trials", "10001"],
    ['--plane-distance-limit-m', '0'], ['--plane-distance-limit-m', 'nan'],
    ['--plane-distance-limit-m', '0.000501'],
    ['--opening-kernel-size', '2'],
])
def test_invalid_cli_does_not_start_ros_or_create_output(tmp_path, extra):
    output = tmp_path/"new"
    result = subprocess.run([sys.executable, str(SCRIPT), "--output-dir", str(output),
                             "--calibration", "missing", "--confirmed-sim-render-contract", *extra],
                            capture_output=True, text=True)
    assert result.returncode == 2
    assert not output.exists()
