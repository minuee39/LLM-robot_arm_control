import importlib.util
import json
import subprocess
import sys
from pathlib import Path
import pytest

ROOT=Path(__file__).parents[1]
spec=importlib.util.spec_from_file_location('gripper_calibration',ROOT/'scripts/gripper_calibration.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def model():
    return module.Calibration(json.loads((ROOT/'config/gripper_calibration.json').read_text()))


@pytest.mark.parametrize('angle,width,depth',[(-12,19,10),(0,38,6),(12,53,2),(-6,28.5,8),(6,45.5,4)])
def test_measured_samples_and_interpolation(angle,width,depth):
    c=model()
    assert c.motor_to_opening(angle)==pytest.approx(width)
    assert c.opening_to_motor(width)==pytest.approx(angle)
    assert c.motor_to_nut_depth(angle)==pytest.approx(depth)
    assert c.nut_depth_to_motor(depth)==pytest.approx(angle)


@pytest.mark.parametrize('angle',[-12.01,12.01,float('nan'),float('inf'),True])
def test_rejects_invalid_angle(angle):
    with pytest.raises(ValueError):model().motor_to_opening(angle)
    with pytest.raises(ValueError):model().motor_to_nut_depth(angle)


@pytest.mark.parametrize('width',[0,18.99,53.01,float('nan'),float('-inf'),True])
def test_rejects_invalid_opening(width):
    with pytest.raises(ValueError):model().opening_to_motor(width)


def test_rejects_ambiguous_inverse():
    data=json.loads((ROOT/'config/gripper_calibration.json').read_text())
    data['samples'][1]['opening_mm']=19
    with pytest.raises(ValueError,match='strictly increasing'):module.Calibration(data)


@pytest.mark.parametrize('depth',[1.99,10.01,float('nan'),float('inf'),True])
def test_rejects_invalid_nut_depth(depth):
    with pytest.raises(ValueError):model().nut_depth_to_motor(depth)


@pytest.mark.parametrize('bad_depth',[10,11,-1,float('nan'),True])
def test_rejects_invalid_depth_calibration(bad_depth):
    data=json.loads((ROOT/'config/gripper_calibration.json').read_text())
    data['samples'][1]['nut_depth_mm']=bad_depth
    with pytest.raises(ValueError):module.Calibration(data)


@pytest.mark.parametrize('flag,value,expected',[
    ('--motor-rad','0',{'raw_motor_rad':0,'estimated_free_space_opening_mm':38,'estimated_nut_depth_mm':6}),
    ('--opening-mm','19',{'requested_opening_mm':19,'motor_target_rad':-12,'estimated_nut_depth_mm':10}),
    ('--nut-depth-mm','4',{'requested_nut_depth_mm':4,'motor_target_rad':6,'estimated_free_space_opening_mm':45.5}),
])
def test_offline_cli(flag,value,expected):
    result=subprocess.run([sys.executable,str(ROOT/'scripts/gripper_calibration.py'),
                           '--config',str(ROOT/'config/gripper_calibration.json'),flag,value],
                          check=True,capture_output=True,text=True)
    assert json.loads(result.stdout)=={'motor_id':7,**expected,'actuation':False}


def test_no_fully_closed_reference_claim():
    data=json.loads((ROOT/'config/gripper_calibration.json').read_text())
    assert 'closed_reference_motor_rad' not in data
    assert 'closed_reference_opening_mm' not in data
    assert not data['actuation_commissioned']
