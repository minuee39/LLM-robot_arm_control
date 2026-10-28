"""Bounded single-axis current experiment; SI units, no position control."""
from dataclasses import dataclass
import math
import struct
import sys
import time
from robstride_serial_readonly import SerialTransport


@dataclass(frozen=True)
class Limits:
    current_a: float = 0.1
    measured_current_a: float = 0.2
    slew_a_s: float = 0.1
    duration_s: float = 2.0
    travel_rad: float = 0.035
    speed_rad_s: float = 0.15
    period_s: float = 0.02
    feedback_timeout_s: float = 0.015
    watchdog_s: float = 0.25

    def validate(self):
        if not all(math.isfinite(v) and v>0 for v in vars(self).values()):
            raise ValueError('All experiment limits must be finite and positive')
        # Conservative software ceilings for this commissioning tool, not certified hardware ratings.
        if self.current_a>0.5 or self.slew_a_s>0.5 or self.duration_s>5 or self.travel_rad>0.1 or self.speed_rad_s>0.5:
            raise ValueError('Exceeds commissioning-tool ceiling')
        if not self.current_a<=self.measured_current_a<=1.:
            raise ValueError('Require current cap <= measured-current trip <= 1A')
        if not self.feedback_timeout_s<self.period_s<self.watchdog_s<=0.5:
            raise ValueError('Require feedback timeout < period < watchdog <= 0.5s')


def pulse_current(elapsed,limits,sign):
    """Ramp up, then ramp down before the bounded experiment deadline."""
    if sign not in (-1,1):raise ValueError('sign must be -1 or +1')
    return sign*max(0.,min(limits.current_a,elapsed*limits.slew_a_s,(limits.duration_s-elapsed)*limits.slew_a_s))


class CurrentTestBus:
    def __init__(self,port,mid,watchdog_index,cap,host=253,allow_gripper=False):
        if not math.isfinite(cap) or not 0<cap<=.5:raise ValueError('Invalid current cap')
        valid_id=1<=mid<=6 or (allow_gripper and mid==7)
        if not valid_id or watchdog_index not in (0x200c,0x7028):raise ValueError('Invalid arm ID or watchdog index')
        self.mid,self.host,self.watchdog_index,self.cap=mid,host,watchdog_index,cap
        self._last_tx=float("-inf")
        self.transport=SerialTransport(port,921600)
        self.transport.serial.write_timeout=.02

    def send(self,kind,data=bytes(8)):
        if kind not in (3,4,17,18,24) or len(data)!=8:raise ValueError('Unsupported command')
        if kind==24 and data not in (bytes([1,2,3,4,5,6,0,0]),bytes([1,2,3,4,5,6,1,0])):
            raise ValueError('Only active-report switch allowed for type 24')
        if kind in (17,18):
            index=int.from_bytes(data[:2],'little')
            if index not in (0x7005,0x7006,0x7019,0x701a,0x302c,self.watchdog_index):raise ValueError('Parameter not allowed')
            if kind==18 and index in (0x7019,0x701a,0x302c):raise ValueError('Position is read-only')
        pause=.003-(time.monotonic()-getattr(self,'_last_tx',float('-inf')))
        if pause>0:time.sleep(pause)
        arb=(kind<<24)|(self.host<<8)|self.mid
        frame=b'AT'+((arb<<3)|4).to_bytes(4,'big')+b'\x08'+data+b'\r\n'
        if self.transport.serial.write(frame)!=len(frame):raise RuntimeError('Short write')
        self._last_tx=time.monotonic()

    def check_fault(self,msg):
        mid=(msg.arbitration_id>>8)&255;kind=(msg.arbitration_id>>24)&31
        if mid==self.mid and (kind==21 or (kind in (2,24) and (msg.arbitration_id>>16)&63)):
            raise RuntimeError('Motor fault frame received')

    def read(self,index,fmt,timeout):
        for _ in range(2048):
            msg=self.transport.recv(0)
            if msg is None:break
            self.check_fault(msg)
        else:raise RuntimeError('Serial backlog')
        self.send(17,index.to_bytes(2,'little')+bytes(6))
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            msg=self.transport.recv(max(0,deadline-time.monotonic()))
            if msg is None:continue
            self.check_fault(msg)
            arb=msg.arbitration_id
            if (arb>>24)&31==17 and (arb>>8)&255==self.mid and arb&255==self.host and len(msg.data)==8 and int.from_bytes(msg.data[:2],'little')==index:
                status=(arb>>16)&255
                if status:raise RuntimeError(f'Parameter read rejected: 0x{index:04x}, status={status}')
                value=struct.unpack(fmt,msg.data[4:4+struct.calcsize(fmt)])[0]
                if not math.isfinite(value):raise RuntimeError('Nonfinite feedback')
                return value
        raise RuntimeError(f'Feedback timeout: 0x{index:04x}')

    def write(self,index,value,fmt):
        if index==0x7006 and (not math.isfinite(value) or abs(value)>self.cap):raise ValueError('Current cap exceeded')
        if index==0x7005 and (value!=3 or fmt!='<B'):raise ValueError('Only current mode is permitted')
        if index==self.watchdog_index and (fmt!='<I' or not 400<=value<=10000):raise ValueError('Watchdog must be enabled (20..500ms)')
        raw=struct.pack(fmt,value)
        self.send(18,index.to_bytes(2,'little')+bytes(2)+raw+bytes(4-len(raw)))

    def current(self,value):
        if not math.isfinite(value) or abs(value)>self.cap+1e-9:raise ValueError('Current cap exceeded')
        self.write(0x7006,value,'<f')

    def stop(self):self.send(4)
    def stop_and_confirm(self):
        # STOP is idempotent. Never assume successful serial write means the
        # motor has processed it; require Reset feedback from this motor.
        for attempt in range(3):
            for _ in range(2048):
                if self.transport.recv(0) is None:break
            else:raise RuntimeError('Serial backlog before STOP confirmation')
            self.stop()
            deadline=time.monotonic()+.1
            while time.monotonic()<deadline:
                msg=self.transport.recv(max(0,deadline-time.monotonic()))
                if msg is None:continue
                arb=msg.arbitration_id
                if (arb>>24)&31 in (2,24) and (arb>>8)&255==self.mid and arb&255==self.host:
                    if len(msg.data)==8 and (arb>>22)&3==0 and (arb>>16)&63==0:return
        raise RuntimeError('STOP was sent but Reset acknowledgement missing')

    def close(self):self.transport.shutdown()



def verify_register(bus,index,expected,fmt,sleep=time.sleep):
    # Space setup writes/read requests for the CH340 AT adapter. Retry reads
    # only; never repeat ENABLE or silently accept an unverified setting.
    sleep(.01)
    actual=None
    for attempt in range(3):
        try:
            actual=bus.read(index,fmt,.1)
            break
        except RuntimeError as exc:
            if not str(exc).startswith('Feedback timeout:') or attempt==2:
                raise
            sleep(.01)
    if not math.isfinite(actual) or abs(actual-expected)>(1e-6 if fmt=='<f' else 0):
        raise RuntimeError(f'Parameter 0x{index:04x}: expected {expected}, got {actual}')


def prepare_current_mode(bus,watchdog_ticks,sleep=time.sleep):
    bus.write(bus.watchdog_index,watchdog_ticks,'<I')
    verify_register(bus,bus.watchdog_index,watchdog_ticks,'<I',sleep)
    bus.write(0x7005,3,'<B')
    verify_register(bus,0x7005,3,'<B',sleep)
    bus.current(0.)
    verify_register(bus,0x7006,0.,'<f',sleep)


def run_experiment(bus,limits,sign,log,clock=time.monotonic,sleep=time.sleep):
    limits.validate()
    if sign not in (-1,1):raise ValueError('sign must be -1 or +1')
    try:
        bus.stop()
        sleep(.05)
        start_position=bus.read(0x7019,'<f',.2)
        ticks=round(limits.watchdog_s*20000)
        prepare_current_mode(bus,ticks,sleep)
        # Fresh position immediately before enable; reject movement during setup.
        last_position=bus.read(0x7019,'<f',.2)
        if abs(last_position-start_position)>=limits.travel_rad:raise RuntimeError('Axis moved during setup')
        bus.send(3)
        started=last=clock()
        previous_current=0.
        while True:
            tick=clock()
            if tick-started>=limits.duration_s:break
            position=bus.read(0x7019,'<f',limits.feedback_timeout_s)
            measured_current=bus.read(0x701a,'<f',limits.feedback_timeout_s)
            if abs(measured_current)>=limits.measured_current_a:raise RuntimeError('Measured current trip limit reached')
            now=clock();dt=now-last
            if dt<=0 or dt>2*limits.period_s:raise RuntimeError('Missed control deadline')
            speed=(position-last_position)/dt
            if abs(position-start_position)>=limits.travel_rad:raise RuntimeError('Travel limit reached')
            if abs(speed)>=limits.speed_rad_s:
                log({'limit_event':'speed','elapsed_s':now-started,'motor_rad':position,'travel_rad':position-start_position,'speed_rad_s':speed,'measured_iq_a':measured_current,'previous_command_iq_a':previous_current})
                raise RuntimeError('Speed limit reached or angle discontinuity')
            target=pulse_current(now-started,limits,sign)
            max_step=limits.slew_a_s*min(dt,limits.period_s)
            current=max(previous_current-max_step,min(previous_current+max_step,target))
            bus.current(current)
            log({'elapsed_s':now-started,'motor_rad':position,'travel_rad':position-start_position,'speed_rad_s':speed,'command_iq_a':current,'measured_iq_a':measured_current})
            previous_current=current;last_position=position;last=now
            remaining=limits.period_s-(clock()-tick)
            if remaining<=0:raise RuntimeError('Control cycle overrun')
            sleep(remaining)
    finally:
        # Stop selected motor even if the zero-current write fails. No restoration
        # or ENABLE of old position-mode targets; watchdog remains nonzero.
        errors=[]
        try:bus.current(0.)
        except Exception as exc:errors.append('zero current: '+str(exc))
        finally:
            try:getattr(bus,'stop_and_confirm',bus.stop)()
            except Exception as exc:errors.append('STOP: '+str(exc))
        for error in errors:
            try:log({'cleanup_error':error})
            except Exception:print('Cleanup error: '+error,file=sys.stderr)
        if errors:raise RuntimeError('Cleanup incomplete: '+'; '.join(errors))
