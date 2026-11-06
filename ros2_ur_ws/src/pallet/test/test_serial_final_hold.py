import sys, importlib.util, threading
from types import SimpleNamespace as NS
from pathlib import Path
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'scripts'))
import pytest
pytest.importorskip("rclpy")
import pallet_serial_trajectory_bridge as m
from serial_trajectory_core import load_config, NAMES, HealthGate
config,joints=load_config(root/'config/serial_trajectory_dry_run.yaml',root)
def test_final_hold_success_cancel_and_drift(monkeypatch):
    for scenario in ('success','hold','small_drift','cancel','drift'):
        config['hold_after_success']=scenario in ('hold','small_drift')
        config['hold_tolerance_rad']=0.1
        clock=NS(t=0.)
        monkeypatch.setattr(m,"time",NS(monotonic=lambda:clock.t,sleep=lambda dt:setattr(clock,'t',clock.t+max(dt,.000001))))
        class Backend:
            def __init__(self):self.q=[0.]*6;self.commands=[];self.stops=0
            def prepare(self,*args):pass
            def command(self,q):self.q=list(q);self.commands.append(clock.t)
            def stop(self):self.stops+=1
        backend=Backend();logs=[];state=[]
        class Handle:
            request=NS(trajectory=NS(joint_names=NAMES),goal_time_tolerance=NS(sec=0,nanosec=0),path_tolerance=[],goal_tolerance=[])
            @property
            def is_cancel_requested(self):return scenario=='cancel' and clock.t>=2
            def publish_feedback(self,f):pass
            def succeed(self):state.append('success')
            def canceled(self):state.append('canceled')
            def abort(self):state.append('aborted')
        def sample():
            q=list(backend.q)
            if scenario=='drift' and clock.t>=2:q[0]+=config['goal_tolerance_rad']*1.1
            if scenario=='small_drift' and clock.t>=2:q[0]+=0.001019
            return q
        lock=threading.Lock();lock.acquire()
        obj=NS(names=NAMES,config=config,joints=joints,io=threading.Lock(),reservation=lock,closing=threading.Event(),backend=backend,
               health=HealthGate(1,.2),
               sample=sample,get_logger=lambda:NS(info=logs.append,error=logs.append),
               get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:m.FJT.Feedback().header.stamp)),result=m.Bridge.result)
        old=m.validate_goal
        m.validate_goal=lambda *a:(NAMES,[(0.,[0.]*6),(.1,[0.]*6)])
        monkeypatch.setattr(m,'PositionTrajectory',lambda *args:NS(at=lambda t:[0.]*6))
        try:r=m.Bridge.execute(obj,Handle())
        finally:m.validate_goal=old
        assert backend.stops==(0 if scenario in ('hold','small_drift') else 1)
        if scenario in ('success','hold','small_drift'):
            assert sum(line.startswith("ARM_CYCLE ") for line in logs) == len(backend.commands)
            assert state==['success'] and clock.t>=5.3 and len(backend.commands)>=50,(clock.t,state,logs[-3:])
            assert obj.health.fault is None
            if scenario in ('hold','small_drift'):
                assert obj.hold_target==[0.]*6
                assert obj.hold_caps==dict.fromkeys(NAMES,0.1)
                assert not obj.health.stop_confirmed
        else:assert state==[{'cancel':'canceled','drift':'aborted'}[scenario]] and clock.t<3
        if scenario=='drift':
            assert r.error_code==m.FJT.Result.PATH_TOLERANCE_VIOLATED
            assert obj.health.fault and obj.health.stop_confirmed
        if scenario=='cancel':assert obj.health.fault is None
        print(scenario,state,round(clock.t,2),r.error_string)
