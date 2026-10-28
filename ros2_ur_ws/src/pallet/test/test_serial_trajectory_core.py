from pathlib import Path
import sys
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from serial_trajectory_core import NAMES,load_config,validate_goal,DryBackend
ROOT=Path(__file__).parents[1]


def config():return load_config(ROOT/'config/serial_trajectory.yaml',ROOT)


def trajectory(names=None):
    def point(t,value):return NS(time_from_start=NS(sec=t,nanosec=0),positions=[value]+[0.]*5,velocities=[],accelerations=[],effort=[])
    return NS(joint_names=names or NAMES,header=NS(stamp=NS(sec=0,nanosec=0)),points=[point(0,0.),point(1,.02)])


def test_six_axis_goal_and_dry_backend():
    c,j=config();names,points=validate_goal(trajectory(),c,j)
    assert names==NAMES and points[-1][1][0]==.02
    backend=DryBackend();backend.command(points[-1][1])
    assert backend.sample()==points[-1][1]


@pytest.mark.parametrize('case',['duplicate','nan','time','speed','acceleration','position','effort','header'])
def test_invalid_goals_are_rejected(case):
    c,j=config();t=trajectory()
    if case=='duplicate':t.joint_names=['J1']*6
    if case=='nan':t.points[-1].positions[0]=float('nan')
    if case=='time':t.points[-1].time_from_start.sec=0
    if case=='speed':t.points[-1].positions[0]=.2
    if case=='acceleration':t.points[-1].accelerations=[1.]*6
    if case=='position':t.points[-1].positions[0]=10.
    if case=='effort':t.points[-1].effort=[1.]*6
    if case=='header':t.header.stamp.sec=1
    with pytest.raises(ValueError):validate_goal(t,c,j)


def test_live_refused_before_backend_creation():
    with pytest.raises(ValueError,match='all six axes'):
        load_config(ROOT/'config/serial_trajectory.yaml',ROOT,live=True)


def test_long_virtual_goal_keeps_live_envelope_separate():
    hardware,j=config()
    virtual,_=load_config(ROOT/'config/serial_trajectory_dry_run.yaml',ROOT)
    t=trajectory();t.points[-1].time_from_start.sec=60
    t.points[-1].positions[0]=.02
    with pytest.raises(ValueError,match='60.000s exceeds configured maximum 30.000s'):
        validate_goal(t,hardware,j)
    t.points[-1].positions[0]=.5
    assert validate_goal(t,virtual,j)[1][-1][0]==60
    with pytest.raises(ValueError,match='all six axes'):
        load_config(ROOT/'config/serial_trajectory_dry_run.yaml',ROOT,live=True)


def test_observation_reads_all_axes_without_stop_or_writes(monkeypatch):
    import serial_trajectory_core as core
    requests=[];closed=[]
    class Reader:
        def __init__(self,*args):self.mid=1
        def read(self,index,fmt,timeout):
            requests.append((self.mid,index))
            return 0.
        def close(self):closed.append(True)
    monkeypatch.setattr(core,'ReadOnlyParameters',Reader)
    c,j=config();backend=core.ObserveBackend(c,j)
    assert backend.sample()==[0.]*6
    assert requests==[(mid,index) for mid in range(1,7) for index in (0x7019,0x701a,0x701b)]
    backend.stop();backend.close()
    assert len(requests)==18 and closed==[True]
    assert not hasattr(backend,'command') and not hasattr(backend,'prepare')


def test_velocity_spike_is_logged_but_sustained_or_position_speed_stops():
    from serial_trajectory_core import VelocityMonitor
    m=VelocityMonitor(.2,2)
    m.update(0.,0.,0.)
    assert m.update(.0002,.2376,.1)['raw_speed_suspect']
    assert not m.update(.0004,0.,.2)['raw_speed_suspect']
    assert m.update(.0006,.2376,.3)['raw_overspeed_count']==1
    with pytest.raises(RuntimeError,match='Raw velocity trip'):m.update(.0008,.2376,.4)
    m=VelocityMonitor(.2,2);m.update(0.,0.,0.)
    with pytest.raises(RuntimeError,match='Position-derived'):m.update(.03,0.,.1)
    with pytest.raises(RuntimeError,match='Raw velocity trip'):VelocityMonitor(.2,2).update(0.,.4,0.)
    with pytest.raises(RuntimeError,match='Raw velocity trip'):VelocityMonitor(.2).update(0.,.2376,0.)


def test_tested_live_profile_disables_only_explicit_software_trip():
    c,j=load_config(ROOT/'config/serial_trajectory_live_tested.yaml',ROOT,live=True)
    assert c['goal_tolerance_rad']==.001 and c['speed_trip_rad_s']==1.2
    assert c['expected_current_limit_a']['J2']==57.0
    assert all(c['current_trip_a'][n] is None for n in NAMES)


def test_idle_hand_motion_does_not_latch_execution_velocity_fault():
    from serial_trajectory_core import VelocityMonitor
    m=VelocityMonitor(.2,2)
    m.update(0.,0.,0.,enforce=False)
    for i in range(1,5):
        details=m.update(i*.1,.5,i*.1,enforce=False)
        assert details['raw_speed_suspect'] and not details['velocity_guard_active']
        assert details['raw_overspeed_count']==0
    # Active motion continues to trip immediately on large raw speed.
    with pytest.raises(RuntimeError,match='Raw velocity trip: raw=0.500000'):
        m.update(.4,.5,.5,enforce=True)


def test_serial_backend_applies_speed_guard_only_while_active():
    from serial_trajectory_core import SerialBackend,VelocityMonitor
    backend=SerialBackend.__new__(SerialBackend)
    backend.config={'max_joint_travel_rad':.02};backend.active_start=None
    joint=NS(name='J6',motor_id=6,lower=-3.,upper=3.,ratio=1.,measured=lambda q:q)
    axis=NS(joint=joint,limits=NS(current_trip_a=None),read=lambda index,*args: {0x7019:0.,0x701a:0.,0x701b:.5}[index])
    backend.axes=[axis];backend.velocity_monitors=[VelocityMonitor(.2,2)]
    assert backend.sample()==[0.]
    backend.active_start=[0.]*6
    with pytest.raises(RuntimeError,match='J6: Raw velocity trip'):backend.sample()


def test_feedback_read_retries_only_timeout_with_finite_cycle_budget():
    from serial_trajectory_core import read_feedback
    calls=[];now=[0.]
    def read(index,fmt,timeout):
        calls.append(timeout);now[0]+=timeout
        if len(calls)==1:raise RuntimeError('Feedback timeout: 0x701a')
        return .1
    a=NS(joint=NS(name='J2'),read=read)
    assert read_feedback(a,0x701a,.1,lambda:now[0])==(.1,1)
    assert len(calls)==2
    calls.clear();now[0]=0.
    with pytest.raises(RuntimeError,match='cycle deadline'):
        read_feedback(a,0x701a,.005,lambda:now[0])
    assert len(calls)==1
    def fault(*args):raise RuntimeError('Motor fault frame received')
    a.read=fault
    with pytest.raises(RuntimeError,match='J2: Motor fault'):
        read_feedback(a,0x701a,1.,lambda:0.)
    def timeout(*args):raise RuntimeError('Feedback timeout: 0x701a')
    a.read=timeout
    with pytest.raises(RuntimeError,match='after 2 read attempts'):
        read_feedback(a,0x701a,1.,lambda:0.)


def test_live_duration_301_seconds_and_full_excursion_diagnostics():
    c,j=load_config(ROOT/'config/serial_trajectory_live_tested.yaml',ROOT,live=True)
    t=trajectory();t.points[-1].time_from_start=NS(sec=30,nanosec=100000000)
    t.points[-1].positions[0]=.01
    assert validate_goal(t,c,j)[1][-1][0]==pytest.approx(30.1)
    t.points[-1].time_from_start=NS(sec=61,nanosec=0)
    with pytest.raises(ValueError,match='Total trajectory duration 61.000s'):
        validate_goal(t,c,j)
    t.points[-1].positions[0]=.3
    c['max_joint_travel_rad']=.02
    with pytest.raises(ValueError,match='J1: planned excursion 0.300000rad'):
        validate_goal(t,c,j)


def test_no_relative_travel_envelope_keeps_urdf_bounds_and_raw_target_window():
    from serial_trajectory_core import SharedAxis
    from single_axis_position_core import Limits
    c,j=load_config(ROOT/'config/serial_trajectory_live_tested.yaml',ROOT,live=True)
    assert c['max_joint_travel_rad'] is None
    t=trajectory();t.points[-1].time_from_start.sec=10;t.points[-1].positions[0]=.5
    assert validate_goal(t,c,j)[1][-1][1][0]==.5
    t.points[-1].positions[0]=j[0].upper+1.
    with pytest.raises(ValueError,match='URDF bounds'):validate_goal(t,c,j)
    axis=SharedAxis.__new__(SharedAxis);axis.joint=j[0]
    axis.limits=Limits(None,None,.1,.1,j[0].upper-j[0].lower,.001)
    axis.bind_window(j[0].raw(.5))
    assert sorted(axis.window)==pytest.approx(sorted([j[0].raw(j[0].lower),j[0].raw(j[0].upper)]))


def test_tracking_error_identifies_joint_and_keeps_same_tolerance():
    from serial_trajectory_core import tracking_violation
    caps={n:.01 for n in NAMES}
    assert tracking_violation([.005]*6,[0.]*6,caps) is None
    actual=[0.]*6;actual[3]=.012
    message=tracking_violation(actual,[0.]*6,caps)
    assert 'J4' in message and 'error=0.012000rad' in message and 'tolerance=0.010000rad' in message


@pytest.mark.parametrize('policy,writes_profile',[('preserve_existing',False),('write_command_limits',True)])
def test_prepare_preserves_motor_profiles_when_requested(policy,writes_profile):
    from serial_trajectory_core import SerialBackend,VelocityMonitor
    c,j=config();c['motor_profile_policy']=policy
    joint=j[0];calls=[];registers={0x7018:16.,0x7019:0.,0x7016:0.}
    def write(index,value,fmt):calls.append(index);registers[index]=value
    a=NS(joint=joint,limits=NS(goal_tolerance_rad=.001,velocity_rad_s=.1,acceleration_rad_s2=.1),
         stop_and_confirm=lambda:None,bind_window=lambda value:None,
         read=lambda index,*args:registers[index],write=write,enable_and_confirm=lambda:None)
    b=SerialBackend.__new__(SerialBackend);b.config=c;b.axes=[a];b.velocity_monitors=[VelocityMonitor(.2,2)]
    b.prepare([0.])
    assert (0x7024 in calls)==writes_profile and (0x7025 in calls)==writes_profile
    assert 0x7018 not in calls
