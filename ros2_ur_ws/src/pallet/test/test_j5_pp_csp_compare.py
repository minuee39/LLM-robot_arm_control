from pathlib import Path
import sys
from types import SimpleNamespace
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
import pallet_j5_pp_csp_compare as module


@pytest.mark.parametrize('joint,index',[('J2',1),('J5',4)])
@pytest.mark.parametrize('fault',[False,True])
def test_identical_references_modes_and_stop(monkeypatch,fault,joint,index):
    class Clock:
        t=0.
        def now(self):return self.t
        def sleep(self,t):self.t+=t
    clock=Clock();events=[]
    class Axis:
        joint=SimpleNamespace(lower=-3,upper=3)
        position_mode=1
        def write(self,*args):events.append(('write',args))
    class Backend:
        axes=[Axis() for _ in range(6)]
        q=[0.]*6
        commands={1:[],5:[]}
        stops=0
        def sample(self):
            if fault and self.commands[5]:raise RuntimeError('feedback lost')
            return list(self.q)
        def prepare(self,start):events.append(('prepare',self.axes[index].position_mode))
        def command(self,q):self.commands[self.axes[index].position_mode].append(list(q));self.q=list(q)
        def stop(self):self.stops+=1;events.append(('stop',))
    monkeypatch.setattr(module,'verify_register',lambda *args:None)
    b=Backend()
    if fault:
        with pytest.raises(RuntimeError,match='feedback lost'):
            module.compare_modes(b,{'period_s':.1},lambda row:None,clock.now,clock.sleep,joint=joint)
    else:
        result=module.compare_modes(b,{'period_s':.1},lambda row:None,clock.now,clock.sleep,joint=joint)
        assert len(result)==2
        assert b.commands[1]==b.commands[5]
        assert max(q[index] for q in b.commands[1])==pytest.approx(.1)
        assert all(all(q[i]==0 for i in range(6) if i!=index) for q in b.commands[1])
    assert b.stops>=2
    assert events[-2][0]=='stop' and events[-1][0]=='write'
    assert b.axes[index].position_mode==1


def test_csp_whitelist_is_opt_in_and_speed_bounded():
    from single_axis_position_core import PositionTestBus
    bus=PositionTestBus.__new__(PositionTestBus)
    bus.limits=SimpleNamespace(velocity_rad_s=.1,current_limit_a=None,acceleration_rad_s2=.1)
    bus.joint=SimpleNamespace(ratio=1)
    with pytest.raises(ValueError,match='PP mode1'):bus.write(0x7005,5,'<B')
    with pytest.raises(ValueError,match='not allowed'):bus.write(0x7017,.1,'<f')
    bus.position_mode=5
    with pytest.raises(ValueError,match='CSP mode5'):bus.write(0x7005,1,'<B')
    with pytest.raises(ValueError,match='cap exceeded'):bus.write(0x7017,.2,'<f')
