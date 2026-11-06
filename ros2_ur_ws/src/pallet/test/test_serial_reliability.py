from pathlib import Path
import sys
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from serial_trajectory_core import HealthGate,stop_all_axes,check_arm_fault,FeedbackUnavailable
from pallet_serial_health_check import qualify


def test_health_requires_consecutive_fresh_samples_and_explicit_fault_reset():
    h=HealthGate(3,.2)
    h.sample_ok(0.);h.sample_ok(.1)
    assert not h.ready(.1)
    h.sample_failed();h.sample_ok(.2);h.sample_ok(.3)
    assert not h.ready(.3)
    h.sample_ok(.4);assert h.ready(.4)
    assert not h.ready(.7)
    h.latch('J4 timeout',True)
    for t in (.8,.9,1.):h.sample_ok(t)
    assert not h.ready(1.)
    h.reset(1.);assert h.ready(1.)
    h.latch('STOP missing',False)
    for t in (1.1,1.2,1.3):h.sample_ok(t)
    with pytest.raises(RuntimeError,match='confirmed STOP'):h.reset(1.3)


def test_feedback_gap_requires_a_new_complete_readiness_run():
    h=HealthGate(3,.2)
    for t in (0.,.1,.2):h.sample_ok(t)
    assert h.ready(.2)
    h.sample_ok(1.)
    assert h.good==1 and not h.ready(1.)
    h.sample_ok(1.1);h.sample_ok(1.2)
    assert h.ready(1.2)


@pytest.mark.parametrize('missing',[set(),{1,3}])
def test_stop_sends_all_axes_first_and_retries_only_missing(missing):
    now=[0.];queue=[];sent=[];waits=[]
    def send(mid,kind):
        assert kind==4
        sent.append(mid)
        if mid not in missing:
            queue.append(NS(arbitration_id=(2<<24)|(mid<<8)|253,data=bytes(8)))
    axes=[NS(mid=i,joint=NS(name=f'J{i}'),send=lambda kind,i=i:send(i,kind)) for i in range(1,7)]
    def recv(timeout):
        if timeout>0:waits.append(list(sent))
        if queue:return queue.pop()
        now[0]+=timeout;return None
    transport=NS(recv=recv)
    if missing:
        with pytest.raises(RuntimeError,match='J1.*J3'):
            stop_all_axes(axes,transport,lambda:now[0])
        assert now[0]==pytest.approx(.3)
        assert sent[6:]==[1,3,1,3]
    else:stop_all_axes(axes,transport,lambda:now[0])
    assert sent[:6]==list(range(1,7))
    assert waits[0]==list(range(1,7))


def test_stop_attempts_every_motor_even_with_receive_and_transmit_failure():
    sent=[];now=[0.]
    def send(mid,kind):
        sent.append(mid)
        if mid==1:raise OSError('TX broken')
    def recv(timeout):raise OSError('RX broken')
    axes=[NS(mid=i,joint=NS(name=f'J{i}'),send=lambda kind,i=i:send(i,kind)) for i in range(1,7)]
    with pytest.raises(RuntimeError,match='J1.*TX broken.*J6'):
        stop_all_axes(axes,NS(recv=recv),lambda:now[0])
    assert sent==list(range(1,7))*3


def test_fault_from_another_axis_is_not_discarded_while_reading():
    for kind in (2,21,24):
        msg=NS(arbitration_id=(kind<<24)|(1<<16)|(6<<8)|253,data=bytes(8))
        with pytest.raises(RuntimeError,match='J6: motor fault'):check_arm_fault(msg)


@pytest.mark.parametrize('miss',[False,True])
def test_readonly_qualification_reports_missing_samples_and_budget(miss):
    now=[0.];events=[]
    class Backend:
        calls=0;last_feedback_retries=0
        def sample(self):
            self.calls+=1;now[0]+=.058
            if miss and self.calls==10:raise FeedbackUnavailable('J4: missing current')
            return [0.]*6
    config={'period_s':.1,'readiness_samples':20}
    result=qualify(Backend(),config,30,events.append,lambda:now[0],lambda dt:now.__setitem__(0,now[0]+dt))
    assert result['passed']==(not miss)
    assert result['missing']==int(miss)
    assert result['read_budget_s']==pytest.approx(.082)
    assert result['actuation'] is False
