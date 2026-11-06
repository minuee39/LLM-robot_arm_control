from pathlib import Path
import sys
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from position_trajectory import PositionTrajectory
from serial_trajectory_core import NAMES


def setup(kind,delta=.2,duration=2):
    def point(t,q):
        return NS(time_from_start=NS(sec=t,nanosec=0),positions=[q]*6,
                  velocities=[0.]*6 if kind in ('cubic','quintic') else [],
                  accelerations=[0.]*6 if kind=='quintic' else [])
    trajectory=NS(joint_names=list(NAMES),points=[point(0,0.),point(duration,delta)])
    joints=[NS(name=n,lower=-1.,upper=1.) for n in NAMES]
    config={'velocity_rad_s':1.,'acceleration_rad_s2':1.,'max_joint_travel_rad':None}
    return trajectory,joints,config


@pytest.mark.parametrize('kind,fraction',[('linear',.25),('cubic',.15625),('quintic',.103515625)])
def test_interpolation_preserves_time_endpoints_and_derivatives(kind,fraction):
    t,j,c=setup(kind);p=PositionTrajectory(t,j,c)
    assert p.at(-1)==[0.]*6 and p.at(3)==[.2]*6
    assert p.at(.5)==pytest.approx([.2*fraction]*6)
    assert p.at(1)==pytest.approx([.1]*6)


@pytest.mark.parametrize('case',['position','velocity','acceleration','travel','terminal','fields'])
def test_continuous_bounds_detect_failures_between_waypoints(case):
    t,j,c=setup('cubic')
    if case=='position':
        t.points[0].velocities=[1.]*6;t.points[1].positions=[0.]*6
        for joint in j:joint.upper=.1
    if case=='velocity':c['velocity_rad_s']=.1  # endpoint velocities are zero; interior peak is .15.
    if case=='acceleration':c['acceleration_rad_s2']=.1
    if case=='travel':c['max_joint_travel_rad']=.1
    if case=='terminal':t.points[-1].velocities=[.01]*6
    if case=='fields':t.points[-1].velocities=[]
    with pytest.raises(ValueError):PositionTrajectory(t,j,c)


def test_reordered_names_map_to_physical_joint_order():
    t,j,c=setup('quintic');t.joint_names=list(reversed(NAMES))
    t.points[-1].positions=[.06,.05,.04,.03,.02,.01]
    assert PositionTrajectory(t,j,c).at(2)==pytest.approx([.01,.02,.03,.04,.05,.06])


def test_moveit_totg_zero_terminal_velocity_with_nonzero_acceleration():
    t,j,c=setup('quintic');t.points[-1].accelerations=[-.1]*6
    assert PositionTrajectory(t,j,c).at(3)==[.2]*6


def test_spawn_worker_roundtrips_ros_trajectory_and_rejects_invalid_goal():
    import multiprocessing
    from concurrent.futures import ProcessPoolExecutor
    pytest.importorskip('trajectory_msgs')
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from position_trajectory import build_checked_trajectory
    t=JointTrajectory(joint_names=list(NAMES),points=[
        JointTrajectoryPoint(positions=[0.]*6,time_from_start=Duration(sec=0)),
        JointTrajectoryPoint(positions=[.1]*6,time_from_start=Duration(sec=2))])
    j=[NS(name=n,lower=-1.,upper=1.) for n in NAMES]
    c={'velocity_rad_s':.5,'acceleration_rad_s2':.2,
       'max_joint_travel_rad':None,'max_goal_duration_s':60.}
    with ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn')) as pool:
        names,points,curve=pool.submit(build_checked_trajectory,t,j,c).result(timeout=10)
        assert names==list(NAMES) and len(points)==2
        assert curve.at(1)==pytest.approx([.05]*6)
        t.points[1].positions=[2.]*6
        with pytest.raises(ValueError):
            pool.submit(build_checked_trajectory,t,j,c).result(timeout=10)
