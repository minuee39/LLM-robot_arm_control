import importlib.util
from pathlib import Path

import numpy as np
import pytest

from vision.box_pose_reader import DiagnosticPoseReader
from test_box_pose_reader import payload


SCRIPT = Path(__file__).resolve().parents[1]/'scripts/analyze_pallet_enclosure_sensitivity.py'
spec = importlib.util.spec_from_file_location('enclosure_sensitivity', SCRIPT)
analyzer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analyzer)


def scene():
    p = payload()
    p['objects']['red_block']['position'] = [.24, -.28, .025]
    p['objects']['green_block']['position'] = [.28, -.24, .025]
    p['objects']['blue_block']['position'] = [.28, -.28, .025]
    return DiagnosticPoseReader(expected_revision='revision', max_age_s=1.).parse(p, now_unix=100.)


def analyze(**changes):
    args = dict(nominal_size_m=[.03, .03, .05], position_errors_m=[0., .004, .006],
                rotation_errors_rad=[0., .05], size_error_m=0., numerical_margin_m=1e-6,
                position_min_m=[-.9, -.375, 0.], position_max_m=[.9, .375, .2])
    args.update(changes)
    return analyzer.analyze(scene(), **args)


def test_four_centimetre_layout_is_conditional_not_approved():
    result = analyze()
    assert result['diagnostic_only'] and result['collision_model_approved'] is False
    assert result['uncertainty'] == 'hypothetical_sensitivity_not_certified'
    assert all(p['nominal_sphere_signed_gap_m'] < 0 for p in result['nominal_unknown_direction_spheres'])
    rows = result['rows']
    assert rows[0]['minimum_pair_sat_axis_gap_m'] == pytest.approx(.01)
    assert rows[0]['all_pairs_separated']
    assert rows[2]['minimum_pair_sat_axis_gap_m'] == pytest.approx(.002)
    assert rows[2]['all_pairs_separated']
    assert not rows[3]['all_pairs_separated']  # full SO(3) inflation matters
    assert not rows[4]['all_pairs_separated']


def test_touching_is_not_clear_and_dimension_error_reduces_gap():
    touching = analyze(position_errors_m=[.005], rotation_errors_rad=[0.])
    assert not touching['rows'][0]['all_pairs_separated']
    dimension = analyze(position_errors_m=[0.], rotation_errors_rad=[0.], size_error_m=.001)
    assert dimension['rows'][0]['minimum_pair_sat_axis_gap_m'] == pytest.approx(.01-np.sqrt(3)*.001)


@pytest.mark.parametrize('changes', [
    dict(position_errors_m=[]), dict(rotation_errors_rad=[]),
    dict(position_errors_m=[-.1]), dict(rotation_errors_rad=[float('nan')]),
    dict(position_errors_m=[True]), dict(size_error_m=-.1),
    dict(numerical_margin_m=-.1), dict(position_errors_m=[0.]*401),
    dict(position_min_m=[.2399, -.375, 0.]),
])
def test_invalid_grid_or_outside_scope_is_rejected(changes):
    with pytest.raises(ValueError):
        analyze(**changes)
