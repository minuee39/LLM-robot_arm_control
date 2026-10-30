import sys
from pathlib import Path
import numpy as np
import pytest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from calibrate_hand_guiding import fit, axes_at_zero


def test_signed_mapping_round_trip():
    for direction in (-1,1):
        samples = [[q, direction*3*q+0.8] for q in (-0.3,0,0.2,0.5)]
        result = fit(samples,0.001)
        assert result['direction'] == direction
        assert result['ratio'] == pytest.approx(3)
        assert result['zero_rad'] == pytest.approx(0.8)
        for q,m in samples:
            assert result['direction']*(m-result['zero_rad'])/result['ratio'] == pytest.approx(q)


def test_reject_ambiguous_and_nonlinear_samples():
    for samples in ([[0,0],[0,1],[0,2]], [[0,0],[1,1],[2,4]], [[0,0],[1,float('nan')],[2,2]]):
        with pytest.raises(ValueError):
            fit(samples,0.01)


def test_urdf_axes():
    report = axes_at_zero(ROOT/'urdf/pallet_with_gripper.urdf')
    assert report['frame'] == 'base_footprint'
    assert len(report['joints']) == 8
    for joint in report['joints']:
        assert np.linalg.norm(joint['positive_axis_root']) == pytest.approx(1)
    joints = {j['joint']:j for j in report['joints']}
    assert np.allclose(joints['J2']['positive_axis_root'],joints['J3']['positive_axis_root'])
