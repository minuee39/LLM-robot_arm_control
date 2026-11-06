"""Endpoint moves retain arm monitoring, cancellation and arrival deadlines."""
import sys
import threading
from pathlib import Path
from types import SimpleNamespace as NS
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import pallet_serial_trajectory_bridge as m
from serial_trajectory_core import load_config,HealthGate
from single_axis_position_core import Joint
from adaptive_motor_mapping import load_motor_profile
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint


@pytest.mark.parametrize('scenario',['success','stall','arm_drift','cancel'])
def test_gripper_sends_only_endpoint_and_preserves_guards(monkeypatch,scenario):
    config,joints=load_config(ROOT/'config/serial_trajectory_dry_run.yaml',ROOT)
    joints.append(Joint('gripper_motor',7,1,0.,1.,-12.,12.))
    config['axis_limits']={'gripper_motor':load_motor_profile(ROOT/'config/adaptive_gripper_motor.yaml')}
    config['hold_after_success']=False
    clock=NS(t=0.)
    monkeypatch.setattr(m,'time',NS(monotonic=lambda:clock.t,sleep=lambda dt:setattr(clock,'t',clock.t+max(dt,.000001))))
    class Backend:
        def __init__(self):self.q=[0.]*7;self.commands=[];self.stops=0
        def prepare(self,*args):pass
        def command(self,q):self.commands.append(list(q))
        def stop(self):self.stops+=1
        def sample(self):
            if self.commands and scenario!='stall':self.q[-1]=min(.5,self.q[-1]+.05)
            if self.commands and scenario=='arm_drift':self.q[0]=.2
            return list(self.q)
    backend=Backend();state=[]
    trajectory=JointTrajectory(joint_names=['gripper_motor'])
    trajectory.points=[JointTrajectoryPoint(positions=[0.],velocities=[0.]),
                       JointTrajectoryPoint(positions=[.5],velocities=[0.])]
    trajectory.points[-1].time_from_start.sec=1
    class Handle:
        request=NS(trajectory=trajectory,goal_time_tolerance=NS(sec=0,nanosec=0),path_tolerance=[],goal_tolerance=[])
        @property
        def is_cancel_requested(self):return scenario=='cancel' and clock.t>=.2
        def publish_feedback(self,f):pass
        def succeed(self):state.append('success')
        def canceled(self):state.append('canceled')
        def abort(self):state.append('aborted')
    reservation=threading.Lock();reservation.acquire()
    obj=NS(config=config,joints=joints,names=[j.name for j in joints],adaptive=True,cached=[0.]*7,
           io=threading.Lock(),reservation=reservation,closing=threading.Event(),backend=backend,
           health=HealthGate(1,.2),sample=backend.sample,
           get_logger=lambda:NS(info=lambda _:None,error=lambda _:None),
           get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:m.FJT.Feedback().header.stamp)),result=m.Bridge.result)
    result=m.Bridge.execute(obj,Handle())
    assert backend.commands and all(q==[0.]*6+[.5] for q in backend.commands)
    assert backend.stops==1 and not reservation.locked()
    assert state==[{'success':'success','stall':'aborted','arm_drift':'aborted','cancel':'canceled'}[scenario]]
    if scenario=='success':assert result.error_code==0 and clock.t>=config['final_hold_s']
    if scenario=='stall':assert result.error_code==m.FJT.Result.GOAL_TOLERANCE_VIOLATED
    if scenario=='arm_drift':assert 'J1' in result.error_string
    if scenario=='cancel':assert obj.health.fault is None
