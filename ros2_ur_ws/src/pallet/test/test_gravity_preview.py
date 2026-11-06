import sys
from pathlib import Path
import yaml
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from pallet_gravity_preview import evaluate_preview
from motor_joint_state_core import MotorJointMapping
from hand_guiding_core import GravityModel


def test_missing_conversion_never_invents_current():
    path=ROOT/'config/hand_guiding.yaml';urdf=ROOT/'urdf/pallet_with_gripper.urdf'
    cfg=yaml.safe_load(path.read_text())
    mapping=MotorJointMapping(path,urdf)
    result=evaluate_preview(GravityModel(urdf),mapping,dict.fromkeys(mapping.ids,0.),cfg['joints'],cfg['gravity_root_m_s2'])
    assert len(result['arm'])==6
    assert all(row['predicted_iq_a'] is None for row in result['arm'].values())
    assert result['arm']['J2']['motor_output_nm']==pytest.approx(-result['arm']['J2']['gravity_joint_nm'])
    cfg['joints']['J2']['output_nm_per_a']=2.0
    cfg['joints']['J2']['current_limit_a']=0.1
    result=evaluate_preview(GravityModel(urdf),mapping,dict.fromkeys(mapping.ids,0.),cfg['joints'],cfg['gravity_root_m_s2'])
    row=result['arm']['J2']
    assert row['predicted_iq_a']==pytest.approx(row['motor_output_nm']/2.)
    assert 'current_limit_a EXCEEDED' in row['issues']
