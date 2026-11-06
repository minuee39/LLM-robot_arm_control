from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
pytest.importorskip('rclpy')
from rclpy.time import Time
import pallet_serial_trajectory_bridge as m
from serial_trajectory_core import HealthGate,FeedbackUnavailable


def node(backend):
    published=[];status=[];logs=[]
    obj=NS(names=list(m.NAMES),mapping=None,adaptive=False,backend=backend,cached=None,health=HealthGate(3,.2),dry=False,observe=False,
        io=threading.Lock(),reservation=threading.Lock(),closing=threading.Event(),
        config={'period_s':.1},publisher=NS(publish=published.append),status=NS(publish=status.append),
        get_clock=lambda:NS(now=lambda:Time(seconds=100.)),
        get_logger=lambda:NS(warn=logs.append,error=logs.append))
    obj.sample=lambda:m.Bridge.sample(obj)
    obj.publish_status=lambda:m.Bridge.publish_status(obj)
    return obj,published,status


def test_joint_states_keep_measured_timestamp_and_do_not_publish_partial_samples(monkeypatch):
    monkeypatch.setattr(m.time,'monotonic',lambda:100.)
    class Backend:
        last_sample=99.94
        def sample(self):return [.1]*6
    obj,published,_=node(Backend())
    obj.sample()
    stamp=published[0].header.stamp
    assert stamp.sec+stamp.nanosec*1e-9==pytest.approx(99.94)
    assert list(published[0].position[:6])==[.1]*6
    def fail():raise FeedbackUnavailable('J4: missing current')
    obj.backend.sample=fail
    m.Bridge.publish_state(obj)
    assert len(published)==1
    assert obj.health.good==0 and obj.health.fault is None
    assert obj.cached is None


@pytest.mark.parametrize('stopped',[True,False])
def test_critical_fault_latches_and_missing_stop_prevents_idle_reads(monkeypatch,stopped):
    monkeypatch.setattr(m.time,'monotonic',lambda:100.)
    class Backend:
        reads=0;stops=0
        def sample(self):self.reads+=1;raise RuntimeError('J6: motor fault')
        def stop(self):
            self.stops+=1
            if not stopped:raise RuntimeError('J1 STOP missing')
    backend=Backend();obj,_,_=node(backend)
    m.Bridge.publish_state(obj)
    assert obj.health.fault=='J6: motor fault'
    assert obj.health.stop_confirmed==stopped
    if not stopped:
        m.Bridge.publish_state(obj)
        assert backend.reads==1 and backend.stops==1


@pytest.mark.parametrize('failure',[None,'sample','stop'])
def test_reset_confirms_stop_and_feedback_without_enable(monkeypatch,failure):
    monkeypatch.setattr(m.time,'monotonic',lambda:100.)
    monkeypatch.setattr(m.time,'sleep',lambda _:None)
    class Backend:
        last_sample=100.;calls=[]
        def stop(self):
            self.calls.append('stop')
            if failure=='stop':raise RuntimeError('STOP missing')
        def sample(self):
            self.calls.append('sample')
            if failure=='sample':raise FeedbackUnavailable('feedback missing')
            return [0.]*6
    backend=Backend();obj,_,_=node(backend)
    obj.health.latch('old fault',False)
    response=m.Bridge.reset_fault(obj,None,NS())
    assert response.success==(failure is None)
    assert backend.calls[0]=='stop'
    assert not obj.reservation.locked()
    if failure is None:
        assert backend.calls==['stop','sample','sample','sample']
        assert obj.health.ready(100.)
    else:assert obj.health.fault is not None and not obj.health.ready(100.)


def test_goal_rejects_multiple_joint_state_sources_without_motor_access():
    obj,_,_=node(NS())
    obj.get_publishers_info_by_topic=lambda topic:[NS(),NS()]
    assert m.Bridge.goal(obj,None)==m.GoalResponse.REJECT
    assert not obj.reservation.locked()


@pytest.mark.parametrize('failure',[None,'small_drift','feedback','drift','stale'])
def test_continuous_hold_refreshes_and_stops_on_failure(monkeypatch,failure):
    monkeypatch.setattr(m.time,'monotonic',lambda:100.)
    class Backend:
        last_sample=100.;commands=[];stops=0
        def command(self,q):self.commands.append(list(q))
        def sample(self):
            if failure=='feedback':raise FeedbackUnavailable('missing')
            return [0.2 if failure=='drift' else (0.001658 if failure=='small_drift' else 0.)]*6
        def stop(self):self.stops+=1
    backend=Backend();obj,_,status=node(backend)
    obj.hold_target=[0.]*6;obj.hold_caps={n:.1 for n in m.NAMES}
    obj.health.stop_confirmed=False
    obj.health.last_sample=99. if failure=='stale' else 100.
    m.Bridge.publish_state(obj)
    if failure in (None,'small_drift'):
        assert backend.commands==[[0.]*6] and backend.stops==0
        assert obj.hold_target==[0.]*6 and 'HOLDING' in status[-1].data
    else:
        assert backend.stops==1 and obj.hold_target is None
        assert obj.health.fault and obj.health.stop_confirmed


def test_reserved_next_goal_keeps_previous_target_alive(monkeypatch):
    monkeypatch.setattr(m.time,'monotonic',lambda:100.)
    class Backend:
        last_sample=100.;commands=[]
        def command(self,q):self.commands.append(list(q))
        def sample(self):return [0.02]*6
    backend=Backend();obj,_,_=node(backend)
    obj.hold_target=[0.02]*6;obj.hold_caps=dict.fromkeys(m.NAMES,.1)
    obj.health.stop_confirmed=False;obj.health.last_sample=100.
    obj.reservation.acquire()  # Goal accepted, execute not yet dispatched.
    m.Bridge.publish_state(obj)
    assert backend.commands==[[0.02]*6]
    assert obj.reservation.locked() and obj.hold_target==[0.02]*6
    assert obj.health.fault is None


@pytest.mark.parametrize('state',['fresh','stale','missing','fault','stopped'])
def test_next_goal_uses_fresh_hold_sample_without_duplicate_io(monkeypatch,state):
    monkeypatch.setattr(m.time,'monotonic',lambda:100.)
    class Backend:
        last_sample=100.;reads=0
        def sample(self):self.reads+=1;return [0.03]*6
    backend=Backend();obj,_,_=node(backend)
    obj.hold_target=None if state=='stopped' else [0.02]*6
    obj.cached=None if state=='missing' else [0.021]*6
    obj.health.last_sample=99. if state=='stale' else 99.95
    if state=='fault':obj.health.fault='TX failure'
    with obj.io:
        if state in ('stale','missing','fault'):
            with pytest.raises(RuntimeError):m.Bridge.start_snapshot(obj)
        else:
            q=m.Bridge.start_snapshot(obj)
            assert q==([0.03]*6 if state=='stopped' else [0.021]*6)
            assert q is not obj.cached
    assert backend.reads==(1 if state=='stopped' else 0)
    if state=='fresh':assert obj.health.last_sample==99.95
