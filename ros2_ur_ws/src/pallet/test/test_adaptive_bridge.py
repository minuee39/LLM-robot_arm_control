"""Seven-motor bridge contract, independently of a serial device."""
import sys
from pathlib import Path
from copy import deepcopy
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from adaptive_motor_mapping import AdaptiveMotorMapping,load_motor_profile
from single_axis_position_core import Joint
from serial_trajectory_core import load_config,validate_goal,tracking_violation
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from position_trajectory import PositionTrajectory


def setup():
    c,j=load_config(ROOT/'config/serial_trajectory_dry_run.yaml',ROOT)
    j.append(Joint('gripper_motor',7,1,0.,1.,-12.,12.))
    c['axis_limits']={'gripper_motor':load_motor_profile(ROOT/'config/adaptive_gripper_motor.yaml')}
    t=JointTrajectory(joint_names=[q.name for q in j])
    t.points=[JointTrajectoryPoint(positions=[0.]*7),JointTrajectoryPoint(positions=[0.]*6+[.1])]
    t.points[-1].time_from_start.sec=2
    return c,j,t


def test_seven_axis_interpolation_and_feedback_mapping():
    c,j,t=setup();validate_goal(t,c,j)
    q=PositionTrajectory(t,j,c).at(1.)
    assert q==pytest.approx([0.]*6+[.05])
    mapped=AdaptiveMotorMapping(ROOT/'config/adaptive_gripper/geometry.json').measured_state(t.joint_names[:6],q[:6],q[6])
    assert len(mapped)==22 and mapped['gripper_motor']==.05
    caps=dict.fromkeys(t.joint_names,.01)
    assert 'gripper_motor' in tracking_violation([0.]*7,q,caps)


@pytest.mark.parametrize('bad',[13.,float('nan')])
def test_invalid_motor_target(bad):
    c,j,t=setup();t.points[-1].positions[-1]=bad
    with pytest.raises(ValueError):validate_goal(t,c,j)


def test_gripper_acceleration_uses_own_cap_and_arm_keeps_existing_cap():
    c,j,t=setup()
    t.points[0].velocities=[0.]*7;t.points[1].velocities=[0.]*7
    t.points[0].accelerations=[0.]*6+[.4];t.points[1].accelerations=[0.]*7
    validate_goal(t,c,j)
    t.points[0].accelerations[0]=.4
    with pytest.raises(ValueError):validate_goal(t,c,j)


def test_faster_gripper_profile_keeps_arm_speed_monitor(monkeypatch):
    import serial_trajectory_core as core
    from types import SimpleNamespace
    c,j,_=setup()
    c['speed_trip_rad_s']=1.2
    c['current_trip_a']=dict.fromkeys([q.name for q in j],.8)
    monkeypatch.setattr(core,'SerialTransport',lambda *args:SimpleNamespace(serial=SimpleNamespace(),shutdown=lambda:None))
    backend=core.SerialBackend(c,j)
    assert [m.cap for m in backend.velocity_monitors[:6]]==pytest.approx([1.2]*6)
    assert backend.velocity_monitors[6].cap==pytest.approx(2.2)
    assert backend.axes[6].limits.velocity_rad_s==2.
    assert backend.axes[6].limits.acceleration_rad_s2==5.
    backend.velocity_monitors[6].update(0.,2.,0.)
    with pytest.raises(RuntimeError,match='velocity trip'):
        backend.velocity_monitors[0].update(0.,2.,0.)


def test_segment_speed_uses_gripper_cap_and_preserves_arm_cap():
    c,j,t=setup()
    t.points[-1].positions[-1]=3.
    validate_goal(t,c,j)
    t.points[-1].positions[0]=3.
    with pytest.raises(ValueError,match='Segment speed'):
        validate_goal(t,c,j)
