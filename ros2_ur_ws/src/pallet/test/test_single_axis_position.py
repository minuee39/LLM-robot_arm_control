import sys
from pathlib import Path
import pytest

sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from single_axis_position_core import Joint,Limits,make_plan,run,PositionTestBus

JOINT=Joint('J1',1,-1,0.,1.,-3.14,3.14)
LIMITS=Limits(.1,.2,.1,.2,.02,.0005,timeout_s=2.,hold_s=.1)


class Clock:
    t=0.
    def now(self):return self.t
    def sleep(self,value):self.t+=value


class Bus:
    def __init__(self,clock,fail=None):
        self.clock=clock;self.fail=fail;self.calls=[];self.registers={0x7019:0.};self.enabled=False;self.position=0.
    def stop_and_confirm(self):self.calls.append(('stop',));self.enabled=False
    def bind_window(self,current):self.calls.append(('window',current))
    def write(self,index,value,fmt):self.calls.append(('write',index,value));self.registers[index]=value
    def enable_and_confirm(self):self.calls.append(('enable',));self.enabled=True
    def read(self,index,fmt,timeout):
        self.calls.append(('read',index));self.clock.sleep(.002)
        if self.enabled:
            if self.fail=='timeout':raise RuntimeError('Feedback timeout')
            if self.fail=='interrupt':raise KeyboardInterrupt
        if index==0x7019:
            if self.enabled:
                difference=self.registers[0x7016]-self.position
                self.position+=max(-.0002,min(.0002,difference))
            elif self.fail=='setup_drift' and 0x7005 in self.registers:self.position=.01
            return self.position
        if index==0x701a:return .3 if self.fail=='current' else .01
        if index==0x701b:return 10. if self.fail=='speed' else .01
        return self.registers[index]


def test_joint_coordinates_and_nonfinite_plan():
    p=make_plan(JOINT,.1,.01,LIMITS)
    assert p['start_joint_rad']==pytest.approx(-.1)
    assert p['target_motor_rad']==pytest.approx(.09)
    for value in [float('nan'),float('inf'),.02,.03]:
        with pytest.raises(ValueError):make_plan(JOINT,0.,value,LIMITS)


def test_pp_setup_order_success_and_write_only_registers():
    c=Clock();bus=Bus(c);logs=[]
    run(bus,JOINT,.004,LIMITS,logs.append,c.now,c.sleep)
    index=bus.calls.index(('enable',))
    assert ('write',0x7016,0.) in bus.calls[:index]
    assert ('read',0x7016) in bus.calls[:index]
    assert not any(('read',p) in bus.calls for p in [0x7024,0x7025])
    assert bus.calls[-1]==('stop',)
    assert any(row['event']=='goal_reached_and_hold_observed' for row in logs)


@pytest.mark.parametrize('failure',['timeout','current','speed','setup_drift','interrupt'])
def test_fault_or_cancel_always_stops(failure):
    c=Clock();bus=Bus(c,failure)
    with pytest.raises((RuntimeError,KeyboardInterrupt)):
        run(bus,JOINT,.004,LIMITS,lambda _:None,c.now,c.sleep)
    assert bus.calls[-1]==('stop',)
    if failure=='setup_drift':assert ('enable',) not in bus.calls


def test_stop_confirmation_failure_is_not_success():
    c=Clock();bus=Bus(c)
    original=bus.stop_and_confirm
    def stop():
        if bus.calls:raise RuntimeError('No Reset')
        original()
    bus.stop_and_confirm=stop
    with pytest.raises(RuntimeError,match='STOP confirmation failed'):
        run(bus,JOINT,.004,LIMITS,lambda _:None,c.now,c.sleep)


def test_limits_require_actual_numbers():
    with pytest.raises(ValueError):Limits(.1,None,.1,.2,.02,.001).validate()


def test_pp_bus_forbids_other_modes_zeroing_and_unbounded_targets():
    bus=PositionTestBus.__new__(PositionTestBus)
    bus.limits=LIMITS;bus.joint=JOINT;bus.window=None
    with pytest.raises(ValueError,match='Unsupported'):bus.send(6)
    with pytest.raises(ValueError,match='PP mode1'):bus.write(0x7005,3,'<B')
    with pytest.raises(ValueError,match='bounded window'):bus.write(0x7016,0.,'<f')
    with pytest.raises(ValueError,match='cap exceeded'):bus.write(0x7018,.3,'<f')
    with pytest.raises(ValueError,match='not allowed'):bus.write(0x7006,.1,'<f')


def test_preserve_current_limit_never_writes_register():
    from dataclasses import replace
    limits=replace(LIMITS,current_limit_a=None)
    limits.validate()
    c=Clock();bus=Bus(c);bus.registers[0x7018]=16.;logs=[]
    run(bus,JOINT,.004,limits,logs.append,c.now,c.sleep)
    assert not any(call[0]=='write' and call[1]==0x7018 for call in bus.calls)
    assert any(row.get('observed_current_limit_a')==16. for row in logs)
    rawbus=PositionTestBus.__new__(PositionTestBus)
    rawbus.limits=limits;rawbus.joint=JOINT;rawbus.window=None
    with pytest.raises(ValueError,match='read-only'):rawbus.write(0x7018,16.,'<f')


def test_reference_readback_quantization_is_bounded_by_goal_tolerance():
    from single_axis_position_core import verify_position_reference
    limits=Limits(None,.2,.1,.1,.02,.001)
    c=Clock();bus=Bus(c)
    bus.registers[0x7016]=-.78+.000115
    result=verify_position_reference(bus,-.78,JOINT,limits)
    assert result['readback_tolerance_rad']==pytest.approx(.0005)
    bus.registers[0x7016]=-.78+.00023
    verify_position_reference(bus,-.78,JOINT,limits)
    bus.registers[0x7016]=-.78+.0006
    with pytest.raises(RuntimeError,match='LOC_REF mismatch'):
        verify_position_reference(bus,-.78,JOINT,limits)


def test_trip_sample_is_recorded_before_stop():
    c=Clock();bus=Bus(c,'current');rows=[]
    with pytest.raises(RuntimeError,match='Current trip'):
        run(bus,JOINT,.004,LIMITS,rows.append,c.now,c.sleep)
    event=next(row for row in rows if row.get('event')=='limit_event')
    assert event['reason']=='Current trip' and event['current_a']==.3
    assert rows[-1]['event']=='stop_confirmed'


def test_hardware_limit_preserved_with_software_current_trip_disabled():
    from dataclasses import replace
    limits=replace(LIMITS,current_limit_a=None,current_trip_a=None)
    limits.validate()
    c=Clock();bus=Bus(c,'current');bus.registers[0x7018]=8.;logs=[]
    run(bus,JOINT,.004,limits,logs.append,c.now,c.sleep)
    assert any(row['event']=='goal_reached_and_hold_observed' for row in logs)
    assert not any(call[0]=='write' and call[1]==0x7018 for call in bus.calls)
    assert any(row.get('observed_current_limit_a')==8. for row in logs)


@pytest.mark.parametrize('speed,should_stop',[(.15,False),(.2,True),(.21,True)])
def test_explicit_speed_trip_is_separate_from_command_speed(speed,should_stop):
    from dataclasses import replace
    limits=replace(LIMITS,speed_trip_rad_s=.2)
    c=Clock();bus=Bus(c);original=bus.read
    def read(index,fmt,timeout):
        value=original(index,fmt,timeout)
        return speed if index==0x701b else value
    bus.read=read
    if should_stop:
        with pytest.raises(RuntimeError,match='Speed trip'):
            run(bus,JOINT,.004,limits,lambda _:None,c.now,c.sleep)
    else:run(bus,JOINT,.004,limits,lambda _:None,c.now,c.sleep)
    assert bus.calls[-1]==('stop',)
