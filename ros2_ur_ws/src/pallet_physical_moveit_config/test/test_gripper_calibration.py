import importlib.util
import json
from pathlib import Path
import pytest

ROOT=Path(__file__).parents[1]
spec=importlib.util.spec_from_file_location('gripper_calibration',ROOT/'scripts/gripper_calibration.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def model():
    return module.Calibration(json.loads((ROOT/'config/gripper_calibration.json').read_text()))


@pytest.mark.parametrize('angle,width',[(-12,0),(0,20),(12,40),(-6,10),(6,30)])
def test_measured_samples_and_interpolation(angle,width):
    c=model()
    assert c.motor_to_opening(angle)==pytest.approx(width)
    assert c.opening_to_motor(width)==pytest.approx(angle)


@pytest.mark.parametrize('angle',[-12.01,12.01,float('nan'),float('inf'),True])
def test_rejects_invalid_angle(angle):
    with pytest.raises(ValueError):model().motor_to_opening(angle)


@pytest.mark.parametrize('width',[-.01,40.01,float('nan'),float('-inf'),True])
def test_rejects_invalid_opening(width):
    with pytest.raises(ValueError):model().opening_to_motor(width)


def test_rejects_ambiguous_inverse():
    data=json.loads((ROOT/'config/gripper_calibration.json').read_text())
    data['samples'][1]['opening_mm']=0
    with pytest.raises(ValueError,match='strictly increasing'):module.Calibration(data)
