import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "analyze_pallet_rgbd.py"
spec = importlib.util.spec_from_file_location("analyze_pallet_rgbd", SCRIPT)
analyzer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analyzer)


def capture(tmp_path):
    np.savez(tmp_path/"rgbd.npz", rgb=np.zeros((5, 5, 3), dtype=np.uint8), depth=np.ones((5, 5)))
    meta = dict(width=5, height=5, d=[0]*5, r=np.eye(3).reshape(-1).tolist(),
                k=[100, 0, 2.5, 0, 100, 2.5, 0, 0, 1],
                rgb_frame="optical", depth_frame="optical", camera_info_frame="optical",
                rgb_stamp=10., depth_stamp=10., stamp_clock="ros_header", calibration_revision="test")
    (tmp_path/"metadata.json").write_text(json.dumps(meta))
    return meta


def test_no_publishing_or_collision_approval_even_for_valid_capture(tmp_path):
    capture(tmp_path)
    result = analyzer.analyze_capture(tmp_path, saturation=50, sim_render_contract_confirmed=True)
    assert result["diagnostic_only"] and not result["collision_model_approved"]
    assert result["uncertainty"] == "not_certified"
    assert result["current_motion_freshness"] == "not_evaluated_offline"
    assert result["detections"] == []


def test_unconfirmed_contract_is_rejected_before_io(tmp_path):
    with pytest.raises(ValueError, match="contract confirmation"):
        analyzer.analyze_capture(tmp_path, saturation=50, sim_render_contract_confirmed=False)


def test_pixel_convention_and_intrinsics_change_revision(tmp_path):
    meta = capture(tmp_path)
    first = analyzer.analyze_capture(tmp_path, saturation=50, sim_render_contract_confirmed=True)
    assert first['pixel_center_offset_px'] == .5
    meta['k'][0] = 200.
    (tmp_path/'metadata.json').write_text(json.dumps(meta))
    second = analyzer.analyze_capture(tmp_path, saturation=50, sim_render_contract_confirmed=True)
    assert first['calibration_revision'] != second['calibration_revision']


def test_tighter_plane_policy_changes_revision_without_approval(tmp_path):
    capture(tmp_path)
    default = analyzer.analyze_capture(tmp_path, saturation=50, sim_render_contract_confirmed=True)
    strict = analyzer.analyze_capture(tmp_path, saturation=50, sim_render_contract_confirmed=True,
                                     plane_distance_limit_m=.00005)
    assert default['plane_limits']['distance_limit_m'] == .0005
    assert strict['plane_limits']['distance_limit_m'] == .00005
    assert default['calibration_revision'] != strict['calibration_revision']
    assert strict['collision_model_approved'] is False


@pytest.mark.parametrize('limit', [0, -.1, .000501, float('nan'), float('inf'), True])
def test_plane_policy_cannot_be_loosened(tmp_path, limit):
    capture(tmp_path)
    with pytest.raises(ValueError, match='plane distance limit'):
        analyzer.analyze_capture(tmp_path, saturation=50, sim_render_contract_confirmed=True,
                                 plane_distance_limit_m=limit)


@pytest.mark.parametrize("field,value", [("depth_frame", "other"), ("d", [1, 0, 0, 0, 0])])
def test_incompatible_registration_metadata_is_rejected(tmp_path, field, value):
    meta = capture(tmp_path)
    meta[field] = value
    (tmp_path/"metadata.json").write_text(json.dumps(meta))
    with pytest.raises(ValueError, match="incompatible"):
        analyzer.analyze_capture(tmp_path, saturation=50, sim_render_contract_confirmed=True)
