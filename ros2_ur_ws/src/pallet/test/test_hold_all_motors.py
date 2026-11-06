from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from pallet_hold_all_motors import hold,configuration

class Clock:
    t=0.
    def now(self):return self.t
    def sleep(self,t):self.t+=t

@pytest.mark.parametrize('failure',[None,'feedback','drift','interrupt','stop'])
def test_seven_motor_hold_preserves_captured_targets_and_stops(failure):
    class Backend:
        commands=[];prepared=None;stopped=False;reads=0
        def sample(self):
            self.reads+=1
            if self.reads>1:
                if failure=='feedback':raise RuntimeError('No feedback')
                if failure=='interrupt':raise KeyboardInterrupt
                if failure=='drift':return [.5]*7
            return [i*.01 for i in range(1,8)]
        def prepare(self,target):self.prepared=list(target)
        def command(self,target):self.commands.append(list(target))
        def stop(self):
            self.stopped=True
            if failure=='stop':raise RuntimeError('STOP not confirmed')
    backend=Backend();clock=Clock();rows=[]
    if failure:
        with pytest.raises((RuntimeError,KeyboardInterrupt)):
            hold(backend,.1,.3,rows.append,clock.now,clock.sleep)
    else:hold(backend,.1,.3,rows.append,clock.now,clock.sleep)
    assert backend.prepared==[i*.01 for i in range(1,8)]
    assert all(q==backend.prepared for q in backend.commands)
    assert backend.stopped
    assert any(r['event']=='stop_confirmed_all_7' for r in rows)==(failure!='stop')


def test_gripper_is_raw_motor_angle_not_mimic_joint():
    config,joints=configuration(Path(__file__).parents[1])
    assert [j.motor_id for j in joints]==list(range(1,8))
    assert joints[-1].measured(1.2)==1.2 and joints[-1].raw(1.2)==1.2
    assert config['current_trip_a']['ID7_gripper_raw'] is None
    assert 'ID7_gripper_raw' not in config['expected_current_limit_a']
