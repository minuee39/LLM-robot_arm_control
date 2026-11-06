"""6-axis trajectory validation and a single-owner serial PP backend."""
import math
import time
from pathlib import Path
import yaml
from single_axis_position_core import Joint,Limits,PositionTestBus,load_joint,verify_position_reference
from single_axis_current_core import verify_register
from robstride_serial_readonly import SerialTransport
from pallet_position_preflight import ReadOnlyParameters

NAMES=[f'J{i}' for i in range(1,7)]


class FeedbackUnavailable(RuntimeError):
    """Incomplete measured state. Never substitute an old value or zero."""


class HealthGate:
    def __init__(self,required,max_age):
        self.required=required;self.max_age=max_age;self.good=0
        self.last_sample=float('-inf');self.fault=None;self.stop_confirmed=True
    def sample_ok(self,stamp):
        self.good=1 if stamp-self.last_sample>self.max_age else self.good+1
        self.last_sample=stamp
    def sample_failed(self):self.good=0
    def latch(self,reason,stopped):
        self.fault=reason;self.stop_confirmed=stopped;self.good=0
    def ready(self,now):
        return self.fault is None and self.stop_confirmed and self.good>=self.required and now-self.last_sample<=self.max_age
    def reset(self,now):
        if not self.stop_confirmed or self.good<self.required or now-self.last_sample>self.max_age:
            raise RuntimeError('Reset requires confirmed STOP and consecutive fresh complete samples')
        self.fault=None


def check_arm_fault(msg):
    arb=msg.arbitration_id;mid=(arb>>8)&255;kind=(arb>>24)&31
    if 1<=mid<=7 and arb&255==253 and (kind==21 or (kind in (2,24) and (arb>>16)&63)):
        raise RuntimeError(f'J{mid}: motor fault frame 0x{arb:08x}, data={bytes(msg.data).hex()}')


def stop_all_axes(axes,transport,clock=time.monotonic):
    """Send STOP to every axis before waiting; retry only unresolved axes."""
    pending={a.mid:a for a in axes};details={};send_errors={}
    try:
        for _ in range(2048):
            if transport.recv(0) is None:break
    except Exception:
        pass  # An RX failure must not prevent transmission of STOP to every axis.
    for attempt in range(3):
        for mid,a in list(pending.items()):
            try:a.send(4)
            except Exception as error:send_errors[mid]=str(error)
        deadline=clock()+.1
        while pending and clock()<deadline:
            try:msg=transport.recv(max(0.,deadline-clock()))
            except Exception as error:
                for mid in pending:details[mid]=f'RX error={error}'
                break
            if msg is None:continue
            arb=msg.arbitration_id;mid=(arb>>8)&255
            if mid not in pending or arb&255!=253 or (arb>>24)&31 not in (2,24):continue
            details[mid]=f'last_status=0x{arb:08x}, data={bytes(msg.data).hex()}'
            if len(msg.data)==8 and (arb>>22)&3==0 and (arb>>16)&63==0:
                pending.pop(mid);send_errors.pop(mid,None)
        if not pending:return
    missing=[f'{a.joint.name} (ID {mid}): {details.get(mid,"no Reset reply")}'
             +(f'; TX error={send_errors[mid]}' if mid in send_errors else '') for mid,a in pending.items()]
    raise RuntimeError('STOP not confirmed after 3 group attempts: '+'; '.join(missing))


def tracking_violation(actual,desired,caps):
    violations=[(n,a,d,abs(a-d),caps[n]) for n,a,d in zip(caps,actual,desired) if abs(a-d)>caps[n]]
    if not violations:return None
    n,a,d,error,cap=max(violations,key=lambda row:row[3]/row[4])
    return f'Path tracking error at {n}: desired={d:.6f}rad actual={a:.6f}rad error={error:.6f}rad tolerance={cap:.6f}rad'


def load_config(path, model_root, live=False):
    config=yaml.safe_load(Path(path).read_text())
    for key in ('period_s','watchdog_s','max_goal_duration_s','max_start_error_rad',
                'max_path_error_rad','goal_tolerance_rad','settle_timeout_s','velocity_rad_s','acceleration_rad_s2'):
        v=config[key]
        if type(v) not in (float,int) or not math.isfinite(v) or v<=0:raise ValueError(f'Invalid {key}')
    travel=config['max_joint_travel_rad']
    if travel is not None and (type(travel) not in (float,int) or not math.isfinite(travel) or travel<=0):raise ValueError('Invalid max_joint_travel_rad')
    if not config['period_s']<config['watchdog_s']<=.5:raise ValueError('Require period < watchdog <=0.5s')
    if config.get('current_limit_policy')!='preserve_existing':raise ValueError('Only preserve-existing current limits supported')
    if config.get('motor_profile_policy','write_command_limits') not in ('write_command_limits','preserve_existing'):
        raise ValueError('Invalid motor_profile_policy')
    config.setdefault('feedback_timeout_s',.015)
    config.setdefault('readiness_samples',20)
    config.setdefault('final_hold_s',5.)
    config.setdefault('hold_tolerance_rad',config['goal_tolerance_rad'])
    v=config['hold_tolerance_rad']
    if type(v) not in (int,float) or not math.isfinite(v) or v<=0:raise ValueError('Invalid hold_tolerance_rad')
    config.setdefault('hold_after_success',False)
    if type(config['hold_after_success']) is not bool:raise ValueError('Invalid hold_after_success')
    if type(config['feedback_timeout_s']) not in (int,float) or not math.isfinite(config['feedback_timeout_s']) or not 0<config['feedback_timeout_s']<config['period_s']:
        raise ValueError('Invalid feedback timeout')
    if type(config['readiness_samples']) is not int or not 1<=config['readiness_samples']<=100:raise ValueError('Invalid readiness sample count')
    if type(config['final_hold_s']) not in (int,float) or not math.isfinite(config['final_hold_s']) or not 0<=config['final_hold_s']<=30:
        raise ValueError('Invalid final hold time')
    joints=[load_joint(model_root/'config/hand_guiding.yaml',model_root/'urdf/pallet_with_gripper.urdf',n) for n in NAMES]
    if live:
        if config.get('commissioned') is not True or set(config.get('verified_joints',[]))!=set(NAMES):
            raise ValueError('LIVE requires explicit commissioning of all six axes; no device opened')
        for name in NAMES:
            v=config.get('current_trip_a',{}).get(name)
            if config.get('software_current_trip_enabled') is False and name in config.get('current_trip_a',{}) and v is None:
                expected=config.get('expected_current_limit_a',{}).get(name)
                if type(expected) not in (float,int) or not math.isfinite(expected) or expected<=0:
                    raise ValueError(f'Disabled software current trip requires verified hardware limit for {name}')
                continue
            if type(v) not in (float,int) or not math.isfinite(v) or v<=0:
                raise ValueError(f'LIVE requires measured-current trip for {name}')
    return config,joints


def validate_goal(trajectory,config,joints):
    NAMES=[j.name for j in joints]
    count=len(NAMES)
    names=list(trajectory.joint_names)
    if len(names)!=count or set(names)!=set(NAMES):raise ValueError('Expected exactly J1..J6, no duplicates')
    if trajectory.header.stamp.sec or trajectory.header.stamp.nanosec:
        raise ValueError('Only immediate trajectories (zero header stamp) supported')
    if not trajectory.points:raise ValueError('Empty trajectory')
    # Check the full planned excursion first: a large IK goal must not be
    # disguised as a timing-limit error partway through a slow trajectory.
    positions=[list(p.positions) for p in trajectory.points]
    if config['max_joint_travel_rad'] is not None and all(len(q)==count and all(math.isfinite(v) for v in q) for q in positions):
        for i,name in enumerate(names):
            excursion=max(abs(q[i]-positions[0][i]) for q in positions)
            if excursion>config['max_joint_travel_rad']+1e-9:
                raise ValueError(f'{name}: planned excursion {excursion:.6f}rad exceeds commissioning limit {config["max_joint_travel_rad"]:.6f}rad; choose a smaller goal')
    total=max(p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in trajectory.points)
    if math.isfinite(total) and total>config['max_goal_duration_s']:
        raise ValueError(f'Total trajectory duration {total:.3f}s exceeds configured maximum {config["max_goal_duration_s"]:.3f}s')
    limits={j.name:j for j in joints};previous_time=-1.;previous=None;result=[]
    for point in trajectory.points:
        t=point.time_from_start.sec+point.time_from_start.nanosec*1e-9
        values=list(point.positions)
        if point.time_from_start.sec<0 or not 0<=point.time_from_start.nanosec<1000000000:
            raise ValueError('Invalid trajectory duration encoding')
        if not math.isfinite(t) or t<0:raise ValueError(f'Invalid trajectory time: {t}')
        if t>config['max_goal_duration_s']:
            raise ValueError(f'Trajectory time {t:.3f}s exceeds configured maximum {config["max_goal_duration_s"]:.3f}s')
        if t<=previous_time:raise ValueError(f'Non-increasing trajectory time: {t:.9f}s after {previous_time:.9f}s')
        if len(values)!=count or not all(math.isfinite(v) for v in values):raise ValueError('Expected six finite positions')
        for n,v in zip(names,values):
            if not limits[n].lower<=v<=limits[n].upper:raise ValueError('Position outside URDF bounds')
        for field,cap in [('velocities',config['velocity_rad_s']),('accelerations',config['acceleration_rad_s2'])]:
            caps=[config.get('axis_limits',{}).get(n,{}).get('velocity_rad_s' if field=='velocities' else 'acceleration_rad_s2',cap) for n in names]
            data=list(getattr(point,field))
            if data and (len(data)!=count or any(not math.isfinite(v) or abs(v)>limit*1.001 for v,limit in zip(data,caps))):
                raise ValueError(f'Invalid/excessive {field}')
        if point.effort:raise ValueError('Effort commands unsupported')
        speed_caps=[config.get('axis_limits',{}).get(n,{}).get('velocity_rad_s',config['velocity_rad_s']) for n in names]
        if previous is not None and any(abs(a-b)/(t-previous_time)>cap*1.001 for a,b,cap in zip(values,previous,speed_caps)):
            raise ValueError('Segment speed exceeds configured bound')
        result.append((t,values));previous=values;previous_time=t
    return names,result


class DryBackend:
    def __init__(self):self.positions=[0.]*6;self.last_sample=time.monotonic()
    def sample(self):self.last_sample=time.monotonic();return list(self.positions)
    def prepare(self,start,should_cancel=lambda:False):pass
    def command(self,values):self.positions=list(values)
    def stop(self):pass
    def close(self):pass


class ObserveBackend:
    """Same 18 register reads as LIVE, with read-only transport and no control API."""
    def __init__(self,config,joints):
        self.config=config;self.joints=joints;self.last_sample=0.
        self.cycle_s=0.;self.current=[];self.velocity=[]
        self.reader=ReadOnlyParameters(config['serial_port'],1,config['baud'])
        self.reader.check_fault=check_arm_fault
    def sample(self):
        began=time.monotonic();positions=[];currents=[];velocities=[];stamps=[]
        # Reserve the same six PP command slots as execution, without sending
        # commands. Passing this read test is necessary, not a loaded-motion proof.
        deadline=began+self.config['period_s']-.003*len(self.joints)
        self.last_feedback_retries=0;self.last_values=[]
        for joint in self.joints:
            self.reader.mid=joint.motor_id;self.reader.joint=joint
            values=[]
            for index in (0x7019,0x701a,0x701b):
                value,retries=read_feedback(self.reader,index,deadline,timeout=self.config.get('feedback_timeout_s',.015))
                self.last_feedback_retries+=retries;values.append(value)
                if index==0x7019:stamps.append(time.monotonic())
            raw,current,motor_velocity=values;velocity=joint.direction*motor_velocity/joint.ratio
            q=joint.measured(raw)
            if not all(math.isfinite(v) for v in [q,current,velocity]):raise RuntimeError('Non-finite feedback')
            if not joint.lower<=q<=joint.upper:raise RuntimeError(f'{joint.name}: measured position outside URDF')
            positions.append(q);currents.append(current);velocities.append(velocity)
            self.last_values.append({'joint':joint.name,'position_rad':q,'current_a':current,'velocity_rad_s':velocity,
                                     'position_sample_time_s':stamps[-1]})
        if time.monotonic()>deadline:raise FeedbackUnavailable('Feedback cycle deadline exceeded')
        self.last_sample=min(stamps);self.cycle_s=time.monotonic()-began
        self.current=currents;self.velocity=velocities
        return positions
    def stop(self):pass  # Observation must never send STOP, even on error/shutdown.
    def close(self):self.reader.close()


class SharedAxis(PositionTestBus):
    def __init__(self,owner,joint,limits):
        self.owner=owner;self.mid=joint.motor_id;self.host=253;self.watchdog_index=0x7028
        self.joint=joint;self.limits=limits;self.window=None;self.transport=owner.transport
        self._last_tx=float('-inf')
    def send(self,kind,data=bytes(8)):
        pause=.003-(time.monotonic()-self.owner.last_tx)
        if pause>0:time.sleep(pause)
        try:super().send(kind,data)
        except OSError as error:
            index=int.from_bytes(data[:2],'little')
            raise RuntimeError(f'{self.joint.name} (ID {self.mid}): serial TX failed; type={kind}, index=0x{index:04x}: {error}') from error
        self.owner.last_tx=time.monotonic()
    def check_fault(self,msg):check_arm_fault(msg)


class VelocityMonitor:
    """Preserve raw velocity; confirm isolated, position-inconsistent spikes.

    Default one sample retains immediate-trip behavior. Commissioning may
    explicitly select two samples; position overspeed and 2x raw cap still trip
    immediately. This does not prove an instantaneous hardware speed bound.
    """
    def __init__(self,cap,confirmation_samples=1):
        if not math.isfinite(cap) or cap<=0 or type(confirmation_samples) is not int or confirmation_samples not in (1,2):
            raise ValueError('Invalid velocity monitor configuration')
        self.cap=cap;self.required=confirmation_samples;self.count=0;self.previous=None
    def update(self,q,velocity,stamp,enforce=True):
        if not all(math.isfinite(v) for v in (q,velocity,stamp)):raise RuntimeError('Nonfinite velocity feedback')
        derived=None
        if self.previous is not None:
            old_q,old_t=self.previous
            if stamp<=old_t:raise RuntimeError('Non-increasing position sample time')
            derived=(q-old_q)/(stamp-old_t)
        self.previous=(q,stamp)
        self.count=self.count+1 if enforce and abs(velocity)>=self.cap else 0
        details={'position_velocity_rad_s':derived,'raw_overspeed_count':self.count,
                 'raw_speed_suspect':abs(velocity)>=self.cap,'velocity_confirmation_samples':self.required,
                 'velocity_guard_active':enforce}
        if not enforce:return details
        diagnostic=f'raw={velocity:.6f}rad/s; position_derived={derived}; cap={self.cap}; count={self.count}/{self.required}'
        if derived is not None and abs(derived)>=self.cap:raise RuntimeError('Position-derived velocity trip: '+diagnostic)
        if abs(velocity)>=2*self.cap or self.count>=self.required:raise RuntimeError('Raw velocity trip: '+diagnostic)
        return details


def read_feedback(axis,index,deadline,clock=time.monotonic,timeout=.015):
    """Retry one missing read only; never retry motor faults or rejected reads."""
    for attempt in range(2):
        remaining=deadline-clock()
        if remaining<=0:raise FeedbackUnavailable(f'{axis.joint.name}: feedback cycle deadline at 0x{index:04x}')
        try:return axis.read(index,'<f',min(timeout,remaining)),attempt
        except RuntimeError as error:
            if not str(error).startswith('Feedback timeout:'):
                raise RuntimeError(f'{axis.joint.name}: {error}') from error
            if attempt==1:
                raise FeedbackUnavailable(f'{axis.joint.name}: feedback timeout 0x{index:04x} after 2 read attempts') from error


class SerialBackend:
    """Called only under the node's I/O lock; all axes share one exclusive port."""
    def __init__(self,config,joints):
        self.config=config;self.joints=joints;self.last_tx=float('-inf');self.last_sample=0.
        self.last_values=[];self.active_start=None
        self.transport=SerialTransport(config['serial_port'],config['baud'])
        self.transport.serial.write_timeout=.02
        self.axes=[]
        try:
            for j in joints:
                limits=Limits(None,config['current_trip_a'][j.name],config.get('axis_limits',{}).get(j.name,{}).get('velocity_rad_s',config['velocity_rad_s']),config.get('axis_limits',{}).get(j.name,{}).get('acceleration_rad_s2',config['acceleration_rad_s2']),
                              config['max_joint_travel_rad'] if config['max_joint_travel_rad'] is not None else j.upper-j.lower,
                              config['goal_tolerance_rad'],watchdog_s=config['watchdog_s'],speed_trip_rad_s=config.get('speed_trip_rad_s'))
                self.axes.append(SharedAxis(self,j,limits))
            cap=config.get('speed_trip_rad_s',config['velocity_rad_s']*1.1)
            # Keep the arm trip unchanged; faster axis profiles retain 10% speed margin.
            self.velocity_monitors=[VelocityMonitor(max(cap,a.limits.velocity_rad_s*1.1),config.get('velocity_confirmation_samples',1)) for a in self.axes]
        except BaseException:self.transport.shutdown();raise
    def sample(self):
        began=time.monotonic();result=[];sample_times=[];self.last_values=[];self.last_feedback_retries=0
        # Reserve the six goal writes (~18ms) inside the 100ms LIVE cycle.
        period=self.config.get('period_s',.1)
        deadline=began+period-.003*len(self.axes)
        def read(a,index):
            value,retries=read_feedback(a,index,deadline,timeout=self.config.get('feedback_timeout_s',.015))
            self.last_feedback_retries+=retries
            return value
        for a,monitor in zip(self.axes,self.velocity_monitors):
            raw=read(a,0x7019);sample_times.append(time.monotonic())
            current=read(a,0x701a)
            velocity=a.joint.direction*read(a,0x701b)/a.joint.ratio
            q=a.joint.measured(raw)
            if self.active_start is not None and self.config['max_joint_travel_rad'] is not None and abs(q-self.active_start[a.joint.motor_id-1])>=self.config['max_joint_travel_rad']:
                raise RuntimeError(f'{a.joint.name}: measured travel trip')
            if not a.joint.lower<=q<=a.joint.upper:raise RuntimeError(f'{a.joint.name}: measured position outside URDF')
            details={'joint':a.joint.name,'position_rad':q,'current_a':current,'velocity_rad_s':velocity,'position_sample_time_s':sample_times[-1]}
            self.last_values.append(details)
            if a.limits.current_trip_a is not None and abs(current)>=a.limits.current_trip_a:raise RuntimeError(f'{a.joint.name}: current trip')
            try:details.update(monitor.update(q,velocity,sample_times[-1],enforce=self.active_start is not None))
            except RuntimeError as error:raise RuntimeError(f'{a.joint.name}: {error}') from error
            result.append(q)
        if time.monotonic()>deadline:raise FeedbackUnavailable('Feedback cycle deadline exceeded')
        self.last_sample=min(sample_times);self.cycle_s=time.monotonic()-began
        return result
    def prepare(self,start,should_cancel=lambda:False):
        self.observed_current_limits={}
        # Preparation must not invoke a subclass's end-of-experiment profile
        # restoration (SlowJ5ProfileBackend.stop). Only stop the drives here.
        SerialBackend.stop(self)
        # Configure every axis while stopped, then enable all axes. Never leave
        # the first axis enabled while configuring the remaining five.
        for a,q in zip(self.axes,start):
            if should_cancel():raise InterruptedError('Canceled during preparation')
            a.bind_window(a.joint.raw(q))
            for index,value,fmt in [(0x7028,round(self.config['watchdog_s']*20000),'<I'),(0x7005,getattr(a,'position_mode',1),'<B')]:
                a.write(index,value,fmt);verify_register(a,index,value,fmt)
            if getattr(a,'position_mode',1)==5:
                speed=a.limits.velocity_rad_s*a.joint.ratio
                a.write(0x7017,speed,'<f');verify_register(a,0x7017,speed,'<f')
            existing=a.read(0x7018,'<f',.1)
            if not math.isfinite(existing) or existing<=0:raise RuntimeError('Invalid preserved current limit')
            self.observed_current_limits[a.joint.name]=existing
            expected=self.config.get('expected_current_limit_a',{}).get(a.joint.name)
            if expected is not None and not math.isclose(existing,expected,rel_tol=1e-6,abs_tol=1e-6):raise RuntimeError(f'{a.joint.name}: current limit changed (expected={expected:.6f}A, actual={existing:.6f}A)')
            if self.config.get('motor_profile_policy','write_command_limits')=='write_command_limits':
                a.write(0x7024,a.limits.velocity_rad_s*a.joint.ratio,'<f')
                a.write(0x7025,a.limits.acceleration_rad_s2*a.joint.ratio,'<f')
        references=[]
        for a,q in zip(self.axes,start):
            if should_cancel():raise InterruptedError('Canceled before enable')
            raw=a.read(0x7019,'<f',.1)
            if abs(a.joint.measured(raw)-q)>self.config['max_start_error_rad']:raise RuntimeError('Axis moved during configuration')
            a.write(0x7016,raw,'<f');verify_position_reference(a,raw,a.joint,a.limits)
            references.append(raw)
        enabled=[]
        # Only commanded motion is subject to the execution speed guard.
        # Idle hand motion/feedback noise must not latch an execution fault.
        self.active_start=list(start)
        for monitor in self.velocity_monitors:monitor.previous=None;monitor.count=0
        for a in self.axes:
            if should_cancel():raise InterruptedError('Canceled before enable')
            # Refresh holds on already enabled axes during the sequential CAN
            # enable acknowledgements, avoiding an unrefreshed watchdog gap.
            for active,raw in zip(enabled,references):active.write(0x7016,raw,'<f')
            a.enable_and_confirm()
            enabled.append(a)
    def command(self,values):
        for a,q in zip(self.axes,values):a.write(0x7016,a.joint.raw(q),'<f')
    def stop(self):
        stop_all_axes(self.axes,self.transport)
        self.active_start=None
    def close(self):self.transport.shutdown()
