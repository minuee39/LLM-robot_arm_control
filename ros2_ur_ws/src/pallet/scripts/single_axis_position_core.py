"""Bounded PP commissioning, joint rad vs raw motor rad kept explicit."""
from dataclasses import dataclass
import math
import struct
import time
import xml.etree.ElementTree as ET
import yaml
from single_axis_current_core import CurrentTestBus, verify_register


@dataclass(frozen=True)
class Joint:
    name: str
    motor_id: int
    direction: int
    zero: float
    ratio: float
    lower: float
    upper: float

    def measured(self, motor):
        return self.direction*(motor-self.zero)/self.ratio

    def raw(self, joint):
        return self.zero+self.direction*joint*self.ratio


def load_joint(config, urdf, name):
    with open(config) as stream:
        d=yaml.safe_load(stream)['joints'][name]
    limit=ET.parse(urdf).getroot().find(f"joint[@name='{name}']/limit")
    if limit is None:raise ValueError('Joint not found in URDF')
    if name not in [f'J{i}' for i in range(1,7)] or type(d['motor_id']) is not int or d['motor_id']!=int(name[1:]):
        raise ValueError('Only confirmed arm J1..J6 -> ID1..6 allowed')
    if type(d['direction']) is not int or d['direction'] not in (-1,1):raise ValueError('Invalid direction')
    values=[d['zero_rad'],d['ratio'],float(limit.get('lower')),float(limit.get('upper'))]
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in values) or d['ratio']<=0:
        raise ValueError('Invalid mapping')
    return Joint(name,d['motor_id'],d['direction'],*values)


@dataclass(frozen=True)
class Limits:
    current_limit_a: float | None
    current_trip_a: float | None
    velocity_rad_s: float
    acceleration_rad_s2: float
    travel_rad: float
    goal_tolerance_rad: float
    timeout_s: float = 4.
    hold_s: float = .5
    period_s: float = .05
    feedback_timeout_s: float = .015
    watchdog_s: float = .25
    speed_trip_rad_s: float | None = None

    def validate(self):
        if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for k,v in vars(self).items() if k not in ('current_limit_a','current_trip_a','speed_trip_rad_s')):
            raise ValueError('All limits must be finite and positive')
        if self.speed_trip_rad_s is not None and (type(self.speed_trip_rad_s) not in (int,float) or not math.isfinite(self.speed_trip_rad_s) or not self.velocity_rad_s<=self.speed_trip_rad_s<=.5):
            raise ValueError('Speed trip must be between commanded speed and 0.5rad/s')
        if self.current_trip_a is not None and (type(self.current_trip_a) not in (int,float) or not math.isfinite(self.current_trip_a) or self.current_trip_a<=0):
            raise ValueError('Invalid measured-current trip')
        # Software commissioning ceilings, inherited from the existing current
        # test scope. These are not certified hardware or load ratings.
        if self.current_limit_a is None:
            if self.current_trip_a is not None and self.current_trip_a>1.:raise ValueError('Preserve policy measured-current trip must be <=1A or explicitly disabled')
        elif (type(self.current_limit_a) not in (int,float) or not math.isfinite(self.current_limit_a)
              or not 0<self.current_limit_a<=.5 or self.current_trip_a is None or not self.current_limit_a<=self.current_trip_a<=1.):
            raise ValueError('Require explicit command <=0.5A, command <= current trip <=1A')
        if self.travel_rad>.1 or self.velocity_rad_s>.5 or self.acceleration_rad_s2>1.:
            raise ValueError('Exceeds commissioning-tool envelope')
        if self.timeout_s>10 or self.hold_s>=self.timeout_s or not .02<=self.period_s<=.05:
            raise ValueError('Invalid bounded timing')
        if not self.feedback_timeout_s<self.period_s<self.watchdog_s<=.5:
            raise ValueError('Require feedback timeout < period < watchdog <=0.5s')
        if self.goal_tolerance_rad>=self.travel_rad:raise ValueError('Tolerance must be smaller than travel bound')


def make_plan(joint, current_raw, delta, limits):
    limits.validate()
    if not all(math.isfinite(v) for v in (current_raw,delta)):raise ValueError('Nonfinite angle')
    start=joint.measured(current_raw);target=start+delta
    if abs(delta)>=limits.travel_rad or (delta!=0 and abs(delta)<=2*limits.goal_tolerance_rad):
        raise ValueError('Delta must fit travel bound and exceed twice the tolerance (or be zero for hold test)')
    if not joint.lower<=start<=joint.upper or not joint.lower<=target<=joint.upper:
        raise ValueError('Start/goal outside URDF position bounds')
    return {'joint':joint.name,'motor_id':joint.motor_id,'start_joint_rad':start,
            'target_joint_rad':target,'initial_hold_motor_rad':current_raw,
            'target_motor_rad':joint.raw(target),'delta_joint_rad':delta}


class PositionTestBus(CurrentTestBus):
    """Reuse serial feedback/fault/STOP handling with a PP-only command whitelist."""
    def __init__(self,port,joint,limits):
        # Parent cap only guards its current-mode write API, which this PP class
        # replaces. Preserve mode never writes a current-limit register.
        super().__init__(port,joint.motor_id,0x7028,limits.current_limit_a if limits.current_limit_a is not None else .5)
        self.joint=joint;self.limits=limits;self.window=None

    def send(self,kind,data=bytes(8)):
        if kind not in (3,4,17,18) or len(data)!=8:raise ValueError('Unsupported PP command')
        index=int.from_bytes(data[:2],'little')
        reads={0x7005,0x7016,0x7018,0x7019,0x701a,0x701b,0x7028}
        writes={0x7005,0x7016,0x7018,0x7024,0x7025,0x7028}
        if kind==17 and index not in reads:raise ValueError('PP read not allowed')
        if kind==18:
            if index not in writes:raise ValueError('PP write not allowed')
            value=struct.unpack('<f',data[4:])[0]
            if index==0x7005:
                if data[4:]!=bytes([1,0,0,0]):raise ValueError('Only PP mode1 allowed')
            elif index==0x7028:
                if struct.unpack('<I',data[4:])[0]!=round(self.limits.watchdog_s*20000):raise ValueError('Watchdog mismatch')
            else:
                if index==0x7018 and self.limits.current_limit_a is None:raise ValueError('Existing motor current limit is read-only under preserve policy')
                if not math.isfinite(value):raise ValueError('Nonfinite command')
                cap={0x7018:self.limits.current_limit_a,0x7024:self.limits.velocity_rad_s*self.joint.ratio,0x7025:self.limits.acceleration_rad_s2*self.joint.ratio}.get(index)
                if cap is not None and not 0<value<=cap*(1+1e-6):raise ValueError('Configured command cap exceeded')
                if index==0x7016 and (self.window is None or not self.window[0]-1e-6<=value<=self.window[1]+1e-6):raise ValueError('Target outside bounded window')
        pause=.003-(time.monotonic()-self._last_tx)
        if pause>0:time.sleep(pause)
        arb=(kind<<24)|(self.host<<8)|self.mid
        frame=b'AT'+((arb<<3)|4).to_bytes(4,'big')+b'\x08'+data+b'\r\n'
        if self.transport.serial.write(frame)!=len(frame):raise RuntimeError('Short serial write')
        self._last_tx=time.monotonic()

    def write(self,index,value,fmt):
        raw=struct.pack(fmt,value)
        self.send(18,index.to_bytes(2,'little')+bytes(2)+raw+bytes(4-len(raw)))

    def enable_and_confirm(self):
        for _ in range(2048):
            if self.transport.recv(0) is None:break
        else:raise RuntimeError('Serial backlog before enable')
        self.send(3)
        deadline=time.monotonic()+.1
        while time.monotonic()<deadline:
            msg=self.transport.recv(max(0,deadline-time.monotonic()))
            if msg is None:continue
            self.check_fault(msg)
            arb=msg.arbitration_id
            if (arb>>24)&31==2 and (arb>>8)&255==self.mid and arb&255==self.host and len(msg.data)==8:
                if (arb>>22)&3==2 and (arb>>16)&63==0:return
        raise RuntimeError('ENABLE sent once but fresh Run acknowledgement missing')

    def bind_window(self,current_raw):
        radius=self.limits.travel_rad*self.joint.ratio
        lo,hi=sorted([self.joint.raw(self.joint.lower),self.joint.raw(self.joint.upper)])
        self.window=(max(lo,current_raw-radius),min(hi,current_raw+radius))


def verify_position_reference(bus, expected, joint, limits):
    # Stopped RS00 readback varies: later read-only probes showed 0.000230rad
    # reference span, including variation while current was zero. Bound this
    # specific check to half the goal tolerance; keep the separate position
    # drift checks and other register readbacks unchanged.
    tolerance=limits.goal_tolerance_rad*joint.ratio*.5
    actual=bus.read(0x7016,'<f',.1)
    if not math.isfinite(actual) or abs(actual-expected)>tolerance:
        raise RuntimeError(f'LOC_REF mismatch: expected {expected}, got {actual}, tolerance {tolerance}')
    return {'expected_motor_rad':expected,'readback_motor_rad':actual,
            'readback_error_rad':actual-expected,'readback_tolerance_rad':tolerance}


def run(bus,joint,delta,limits,log,clock=time.monotonic,sleep=time.sleep,expected_current_limit_a=None):
    limits.validate()
    try:
        bus.stop_and_confirm()
        initial=bus.read(0x7019,'<f',.1)
        plan=make_plan(joint,initial,delta,limits)
        bus.bind_window(initial)
        # Verify RW registers. 7024/7025 are write-only in the RS05 manual;
        # never treat a type2 status frame as parameter-value readback.
        settings=[(0x7028,round(limits.watchdog_s*20000),'<I'), (0x7005,1,'<B')]
        if limits.current_limit_a is None:
            existing=bus.read(0x7018,'<f',.1)
            if not math.isfinite(existing) or existing<=0:raise RuntimeError('Invalid existing current limit')
            if expected_current_limit_a is not None and not math.isclose(existing,expected_current_limit_a,rel_tol=1e-6,abs_tol=1e-6):
                raise RuntimeError(f'Current limit changed: expected {expected_current_limit_a}A, got {existing}A; refusing enable')
            log({'event':'current_limit_preserved','observed_current_limit_a':existing,
                 'measured_current_trip_a':limits.current_trip_a})
        else:
            settings.append((0x7018,limits.current_limit_a,'<f'))
        for index,value,fmt in settings:
            bus.write(index,value,fmt);verify_register(bus,index,value,fmt,sleep)
        bus.write(0x7024,limits.velocity_rad_s*joint.ratio,'<f')
        bus.write(0x7025,limits.acceleration_rad_s2*joint.ratio,'<f')
        log({'event':'profile_sent_write_only','velocity_joint_rad_s':limits.velocity_rad_s,'acceleration_joint_rad_s2':limits.acceleration_rad_s2})
        fresh=bus.read(0x7019,'<f',.1)
        if abs(joint.measured(fresh)-plan['start_joint_rad'])>limits.goal_tolerance_rad:
            raise RuntimeError('Axis moved during setup; refusing enable')
        bus.write(0x7016,fresh,'<f');sleep(.01)
        log({'event':'initial_reference_verified',**verify_position_reference(bus,fresh,joint,limits)})
        check=bus.read(0x7019,'<f',.1)
        if abs(check-fresh)/joint.ratio>limits.goal_tolerance_rad:raise RuntimeError('Axis moved before enable')
        bus.enable_and_confirm()
        # Start reference remains measured position; never restore an old target.
        bus.write(0x7016,plan['target_motor_rad'],'<f')
        log({'event':'enabled',**plan})
        started=last=clock();last_position=check;within_since=None
        while clock()-started<limits.timeout_s:
            cycle=clock()
            raw=bus.read(0x7019,'<f',limits.feedback_timeout_s)
            current=bus.read(0x701a,'<f',limits.feedback_timeout_s)
            velocity=bus.read(0x701b,'<f',limits.feedback_timeout_s)/joint.ratio
            now=clock();dt=now-last
            q=joint.measured(raw)
            error=abs(q-plan['target_joint_rad'])
            sample={'event':'sample','elapsed_s':now-started,'joint_rad':q,'motor_rad':raw,
                    'current_a':current,'velocity_rad_s':velocity,'goal_error_rad':error}
            def trip(reason):
                # Record the triggering sample too; never lose the value that
                # caused an abort by logging only successful cycles.
                log({**sample,'event':'limit_event','reason':reason})
                raise RuntimeError(reason)
            if dt<=0 or dt>2*limits.period_s:trip('Missed feedback deadline')
            if not all(math.isfinite(v) for v in (raw,current,velocity)):trip('Nonfinite feedback')
            if not joint.lower<=q<=joint.upper:trip('Measured joint outside URDF')
            if abs(q-plan['start_joint_rad'])>=limits.travel_rad:trip('Travel trip')
            if limits.current_trip_a is not None and abs(current)>=limits.current_trip_a:trip('Current trip')
            speed_trip=limits.speed_trip_rad_s if limits.speed_trip_rad_s is not None else limits.velocity_rad_s*1.1
            if max(abs(velocity),abs(raw-last_position)/(joint.ratio*dt))>=speed_trip:
                trip('Speed trip or angle discontinuity')
            log(sample)
            within_since=(within_since if within_since is not None else now) if error<=limits.goal_tolerance_rad else None
            if within_since is not None and now-within_since>=limits.hold_s:
                log({'event':'goal_reached_and_hold_observed'});return plan
            bus.write(0x7016,plan['target_motor_rad'],'<f')
            last=now;last_position=raw
            remaining=limits.period_s-(clock()-cycle)
            if remaining<=0:raise RuntimeError('Control cycle overrun')
            sleep(remaining)
        raise RuntimeError('Goal/hold timeout')
    finally:
        # STOP releases drive torque. The physical arm must remain supported.
        try:bus.stop_and_confirm()
        except Exception as error:
            raise RuntimeError(f'STOP confirmation failed: {error}') from error
        log({'event':'stop_confirmed'})
