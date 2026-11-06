import importlib.util
from pathlib import Path
import pytest
spec = importlib.util.spec_from_file_location('gripper_direct', Path(__file__).resolve().parents[1]/'scripts/gripper_direct.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

@pytest.mark.parametrize('start,target', [(0.,0.), (0.,.2), (0.,6.), (0.,-6.), (0.,12.), (-12.,12.)])
def test_smooth_command_limits(start,target):
    goal,t=m.make_goal(start,target)
    assert 1.5*abs(target-start)/t <= 2.+1e-10
    assert 6*abs(target-start)/t**2 <= 5.+1e-10
    assert list(goal.trajectory.points[0].velocities) == [0.]
    assert list(goal.trajectory.points[-1].positions) == [target]

@pytest.mark.parametrize('start,target', [(0.,13.), (0.,float('nan'))])
def test_invalid_target(start,target):
    with pytest.raises(ValueError):m.make_goal(start,target)
