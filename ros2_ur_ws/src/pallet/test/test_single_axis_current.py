from pathlib import Path
import sys
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from single_axis_current_core import Limits,pulse_current,run_experiment,CurrentTestBus


class Clock:
    t=0.
    def now(self):return self.t
    def sleep(self,dt):self.t+=dt


class FakeBus:
    watchdog_index=0x7028
    def __init__(self,clock,fail=None):
        self.clock=clock;self.enabled=False;self.events=[];self.writes={};self.fail=fail
    def stop(self):self.events.append(('stop',));self.enabled=False
    def send(self,kind):self.events.append(('enable',kind));self.enabled=True
    def write(self,index,value,fmt):self.writes[index]=value
    def current(self,value):self.events.append(('current',value))
    def read(self,index,fmt,timeout):
        self.clock.sleep(.001)
        if index==self.watchdog_index:return 0 if self.fail=='watchdog' else self.writes[index]
        if index==0x7005:return self.writes[index]
        if index==0x7006:return 0.
        if index==0x701a:return .5 if self.fail=='overcurrent' else 0.
        if self.enabled:
            if self.fail=='timeout':raise RuntimeError('Feedback timeout')
            if self.fail=='travel':return .1
            if self.fail=='speed':return .01
        return 0.


def test_bounded_pulse_and_cleanup():
    clock=Clock();bus=FakeBus(clock);rows=[];limits=Limits()
    run_experiment(bus,limits,-1,rows.append,clock.now,clock.sleep)
    assert rows and max(abs(r['command_iq_a']) for r in rows)<=limits.current_a
    assert all(-limits.current_a<=r['command_iq_a']<=0 for r in rows)
    for a,b in zip(rows,rows[1:]):
        assert abs(b['command_iq_a']-a['command_iq_a'])<=limits.slew_a_s*(b['elapsed_s']-a['elapsed_s'])+1e-9
    assert bus.events[-2:]==[('current',0.),('stop',)]
    assert clock.t<2.2


@pytest.mark.parametrize('failure,match',[('watchdog','Parameter'),('timeout','timeout'),('travel','Travel'),('speed','Speed'),('overcurrent','Measured current')])
def test_faults_stop_and_watchdog_failure_never_enables(failure,match):
    clock=Clock();bus=FakeBus(clock,failure)
    with pytest.raises(RuntimeError,match=match):run_experiment(bus,Limits(),1,lambda r:None,clock.now,clock.sleep)
    assert bus.events[-2:]==[('current',0.),('stop',)]
    if failure=='watchdog':assert not any(e[0]=='enable' for e in bus.events)


def test_caps_and_invalid_sign():
    for limits in [Limits(current_a=float('nan')),Limits(current_a=.6),Limits(duration_s=6),Limits(slew_a_s=-1)]:
        with pytest.raises(ValueError):limits.validate()
    with pytest.raises(ValueError):pulse_current(.1,Limits(),0)


def test_log_failure_still_stops():
    clock=Clock();bus=FakeBus(clock)
    def broken_log(row):raise OSError('disk full')
    with pytest.raises(OSError):run_experiment(bus,Limits(),1,broken_log,clock.now,clock.sleep)
    assert bus.events[-2:]==[('current',0.),('stop',)]


def test_zero_write_and_cleanup_log_failure_cannot_skip_stop():
    clock=Clock();bus=FakeBus(clock,'timeout')
    original=bus.current
    def bad_zero(value):
        if value==0 and bus.enabled:raise OSError('disconnected')
        original(value)
    bus.current=bad_zero
    def bad_log(row):raise OSError('disk full')
    with pytest.raises(RuntimeError,match='Cleanup incomplete'):
        run_experiment(bus,Limits(),1,bad_log,clock.now,clock.sleep)
    assert bus.events[-1]==('stop',)


def test_serial_frame_whitelist_and_current_cap():
    from types import SimpleNamespace
    bus=object.__new__(CurrentTestBus)
    bus.mid=2;bus.host=253;bus.watchdog_index=0x7028;bus.cap=.1
    frames=[]
    def write(frame):frames.append(frame);return len(frame)
    bus.transport=SimpleNamespace(serial=SimpleNamespace(write=write))
    bus.current(.1);bus.stop()
    assert len(frames)==2
    assert (int.from_bytes(frames[0][2:6],'big')>>3)==(18<<24)|(253<<8)|2
    assert frames[0][7:9]==b'\x06\x70'
    assert (int.from_bytes(frames[1][2:6],'big')>>3)==(4<<24)|(253<<8)|2
    with pytest.raises(ValueError):bus.current(.2)
    with pytest.raises(ValueError):bus.write(0x7006,.2,'<f')
    with pytest.raises(ValueError):bus.write(0x7005,1,'<B')
    with pytest.raises(ValueError):bus.send(6)
    with pytest.raises(ValueError):bus.write(0x7028,0,'<I')


def test_setup_timeout_read_retried_without_duplicate_write():
    from single_axis_current_core import prepare_current_mode
    clock=Clock();bus=FakeBus(clock)
    original=bus.read
    attempts=[]
    def drop_first_mode_read(index,fmt,timeout):
        if index==0x7005:
            attempts.append(index)
            if len(attempts)==1:raise RuntimeError('Feedback timeout: 0x7005')
        return original(index,fmt,timeout)
    bus.read=drop_first_mode_read
    prepare_current_mode(bus,5000,clock.sleep)
    assert len(attempts)==2
    assert not bus.enabled
    assert bus.writes[0x7005]==3


def test_setup_timeout_failure_prevents_enable():
    clock=Clock();bus=FakeBus(clock)
    original=bus.read
    def absent_mode(index,fmt,timeout):
        if index==0x7005:raise RuntimeError('Feedback timeout: 0x7005')
        return original(index,fmt,timeout)
    bus.read=absent_mode
    with pytest.raises(RuntimeError,match='timeout'):
        run_experiment(bus,Limits(),1,lambda r:None,clock.now,clock.sleep)
    assert not any(e[0]=='enable' for e in bus.events)
    assert bus.events[-1]==('stop',)


def test_parameter_error_frame_is_not_accepted_as_zero():
    from types import SimpleNamespace
    bus=object.__new__(CurrentTestBus);bus.mid=6;bus.host=253
    bus.send=lambda *args:None
    # Drain sees no data, then a rejected parameter response arrives.
    messages=[None,SimpleNamespace(arbitration_id=0x110106fd,data=bytes.fromhex('0670000000000000'))]
    bus.transport=SimpleNamespace(recv=lambda timeout:messages.pop(0) if messages else None)
    with pytest.raises(RuntimeError,match='Parameter read rejected'):
        bus.read(0x7006,'<f',.1)


def test_gripper_requires_explicit_transport_optin(monkeypatch):
    import single_axis_current_core as core
    from types import SimpleNamespace
    ports=[]
    def transport(port,baud):
        ports.append(port)
        return SimpleNamespace(serial=SimpleNamespace(write_timeout=None))
    monkeypatch.setattr(core,'SerialTransport',transport)
    with pytest.raises(ValueError):core.CurrentTestBus('unused',7,0x7028,.1)
    assert not ports
    assert core.CurrentTestBus('fake',7,0x7028,.1,allow_gripper=True).mid==7


def test_stop_requires_reset_acknowledgement():
    from types import SimpleNamespace
    bus=object.__new__(CurrentTestBus);bus.mid=4;bus.host=253
    calls=[]
    def status(state=0,fault=0,data=bytes(8)):
        return SimpleNamespace(arbitration_id=(2<<24)|(state<<22)|(fault<<16)|(4<<8)|253,data=data)
    # A stale Reset must not satisfy the new STOP; neither may malformed/faulted replies.
    queue=[status()]
    def stop():
        calls.append('STOP')
        queue.extend([status(2),status(data=b''),status(fault=1),status()])
    bus.stop=stop
    bus.transport=SimpleNamespace(recv=lambda timeout:queue.pop(0) if queue else None)
    bus.stop_and_confirm()
    assert calls==['STOP']
    assert queue==[]


def test_serial_transmit_pacing(monkeypatch):
    import single_axis_current_core as core
    from types import SimpleNamespace
    clock=[1.]
    monkeypatch.setattr(core.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(core.time,'sleep',lambda dt:clock.__setitem__(0,clock[0]+dt))
    bus=object.__new__(CurrentTestBus);bus.mid=4;bus.host=253;bus.cap=.1;bus.watchdog_index=0x7028
    times=[]
    def write(frame):times.append(clock[0]);return len(frame)
    bus.transport=SimpleNamespace(serial=SimpleNamespace(write=write))
    bus.current(0.);bus.stop()
    assert times[1]-times[0]>=.003-1e-12
