import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from pallet_multi_axis_position import run
from serial_trajectory_core import DryBackend,load_config

class Clock:
    t=0.
    def now(self):return self.t
    def sleep(self,t):self.t+=t

@pytest.mark.parametrize('failure',[None,'sample','travel','stop'])
def test_six_axis_bounded_run_and_stop(failure):
    root=Path(__file__).parents[1]
    config,joints=load_config(root/'config/serial_trajectory.yaml',root)
    config.update(max_joint_travel_rad=.02,goal_tolerance_rad=.001)
    c=Clock();rows=[]
    class Backend(DryBackend):
        stopped=False
        def sample(self):
            if c.t>.2:
                if failure=='sample':raise RuntimeError('Sample failure')
                if failure=='travel':return [.03]*6
            return super().sample()
        def stop(self):
            self.stopped=True
            if failure=='stop':raise RuntimeError('STOP not confirmed')
    b=Backend()
    if failure:
        with pytest.raises(RuntimeError):run(b,joints,config,.01,rows.append,c.now,c.sleep)
    else:
        run(b,joints,config,.01,rows.append,c.now,c.sleep)
        assert b.positions==pytest.approx([.01]*6)
        assert any(r['event']=='goal_reached_and_hold_observed' for r in rows)
    assert b.stopped
    if failure=='stop':assert not any(r['event']=='stop_confirmed_all' for r in rows)
