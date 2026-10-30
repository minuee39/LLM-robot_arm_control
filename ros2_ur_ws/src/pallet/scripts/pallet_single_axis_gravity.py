#!/usr/bin/env python3
"""J5 serial partial gravity compensation. Offline default; strict pose gates before ENABLE."""
import argparse
from dataclasses import asdict
import json
import math
import signal
from pathlib import Path
import time
import yaml
from hand_guiding_core import GravityModel
from motor_joint_state_core import MotorJointMapping,NAMES
from robstride_serial_readonly import ReadOnlySerialMotorBus
from single_axis_current_core import CurrentTestBus,prepare_current_mode
from single_axis_gravity_core import GravityLimits,output,rs05_reported_torque,RS05_NOMINAL_NM_PER_APK


class J5Bus(CurrentTestBus):
    def send(self,kind,data=bytes(8)):
        if kind != 17 and self.mid != 5:
            raise ValueError('Drive commands restricted to ID5')
        return super().send(kind,data)

    def check_fault(self,msg):
        a=msg.arbitration_id;mid=(a>>8)&255;kind=(a>>24)&31
        if mid in range(1,7) and (kind==21 or (kind in (2,24) and (a>>16)&63)):
            raise RuntimeError('Arm motor fault: ID'+str(mid))

    def sample(self):
        raw,stamps={},{}
        try:
            for mid in range(1,7):
                self.mid=mid
                raw[mid]=self.read(0x7019,'<f',.025)
                stamps[mid]=time.monotonic()
        finally:
            self.mid=5
        iq=self.read(0x701a,'<f',.025)
        return raw,iq,stamps

    def current(self,value):
        super().current(value)
        deadline=time.monotonic()+.015
        while time.monotonic()<deadline:
            msg=self.transport.recv(max(0,deadline-time.monotonic()))
            if msg is None:continue
            self.check_fault(msg)
            a=msg.arbitration_id
            if (a>>24)&31 in (2,24) and (a>>8)&255==5 and a&255==self.host and len(msg.data)==8:
                raw=int.from_bytes(msg.data[4:6],'big')
                self.report={'reported_torque_nm':rs05_reported_torque(raw),'raw_torque_u16':raw,'torque_sample_time':time.monotonic()}
                return
        raise RuntimeError('Current-write status acknowledgement timeout')


def calculate(model,mapping,raw,gravity):
    if set(raw)!=set(mapping.ids) or not all(math.isfinite(x) for x in raw.values()):
        raise ValueError('Complete finite motor snapshot required')
    q={}
    for name in NAMES[:6]:
        c=mapping.joints[name]
        angle=c['direction']*(raw[c['motor_id']]-c['zero_rad'])/c['ratio']
        if name=='J5':
            lo,hi=mapping.limits[name]
            if not lo<=angle<=hi:raise ValueError('J5 outside configured position limits')
            q[name]=angle
        else:
            # Geometry of fixed revolute joints is periodic. This is NOT a zero
            # change, winding/cable-limit inference, or permission to move them.
            q[name]=(angle+math.pi)%(2*math.pi)-math.pi
    q['g_r']=0.
    tau,_=model.evaluate(q,gravity)
    return q,tau['J5']


def live(bus,model,mapping,cfg,limits,log,clock=time.monotonic,sleep=time.sleep,*,continuous=False):
    c=cfg['joints']['J5']
    try:
        # Read and validate before changing drive state; again after setup.
        initial,iq,stamps=bus.sample()
        calculate(model,mapping,initial,cfg['gravity_root_m_s2'])
        if abs(iq)>=limits.measured_current_a:raise RuntimeError('Initial measured current exceeded')
        bus.stop_and_confirm()
        bus.send(24,bytes([1,2,3,4,5,6,0,0]))
        prepare_current_mode(bus,5000)
        previous,iq,stamps=bus.sample()
        calculate(model,mapping,previous,cfg['gravity_root_m_s2'])
        if any(abs(previous[m]-initial[m])>.01 for m in initial):raise RuntimeError('Arm moved during setup')
        start_position=previous[5]
        bus.send(3)
        started=clock();sent=0.
        while continuous or clock()-started<limits.duration_s:
            tick=clock();raw,iq,new_stamps=bus.sample()
            dt=new_stamps[5]-stamps[5]
            if not 0<dt<=2*limits.period_s:raise RuntimeError('Missed feedback deadline')
            raw_speed=(raw[5]-previous[5])/dt
            travel=raw[5]-start_position
            q,tau=calculate(model,mapping,raw,cfg['gravity_root_m_s2'])
            event=None
            if abs(iq)>=limits.measured_current_a:event='measured_current'
            elif abs(travel)>=limits.travel_rad:event='travel'
            elif abs(raw_speed)>=limits.speed_rad_s:event='speed'
            elif any(abs(raw[m]-initial[m])>=limits.upstream_motion_rad for m in (1,2,3,4,6)):event='other_axis_moved'
            row={'elapsed_s':clock()-started,'joint_rad':q,'gravity_j5_nm':tau,'motor_rad':raw[5],
                 'travel_rad':travel,'raw_speed_rad_s':raw_speed,'measured_iq_a':iq,'position_sample_time':new_stamps[5]}
            if event:
                row['limit_event']=event;log(row);raise RuntimeError(event+' limit reached')
            row.update(output(tau,c['direction']*raw_speed/c['ratio'],c['direction'],c['ratio'],clock()-started,sent,dt,limits))
            bus.current(row['command_iq_a']);sent=row['command_iq_a']
            row.update(bus.report);log(row)
            previous,stamps=raw,new_stamps
            remaining=limits.period_s-(clock()-tick)
            if remaining<=0:raise RuntimeError('Control cycle overrun')
            sleep(remaining)
    finally:
        errors=[]
        try:bus.current(0.)
        except Exception as e:errors.append('zero: '+str(e))
        try:bus.stop_and_confirm()
        except Exception as e:errors.append('STOP: '+str(e))
        if not errors:
            try:
                after={'fresh_reset_fault0':True,'iq_ref':bus.read(0x7006,'<f',.2),'iqf':bus.read(0x701a,'<f',.2)}
                log({'after':after})
                if after['iq_ref']!=0 or abs(after['iqf'])>.02:errors.append('Nonzero post-stop current')
            except Exception as e:errors.append('readback: '+str(e))
        bus.close()
        if errors:raise RuntimeError('Cleanup incomplete: '+'; '.join(errors))


def main():
    base=Path(__file__).resolve().parents[1]
    if not (base/'urdf').is_dir():
        from ament_index_python.packages import get_package_share_directory
        base=Path(get_package_share_directory('pallet'))
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default=str(base/'config/hand_guiding.yaml'))
    p.add_argument('--urdf',default=str(base/'urdf/pallet_with_gripper.urdf'))
    p.add_argument('--port',default='/dev/ttyUSB0')
    p.add_argument('--motor-positions',nargs=6,type=float,default=[0.]*6)
    p.add_argument('--read-only',action='store_true')
    p.add_argument('--live',action='store_true')
    p.add_argument('--arm-supported',action='store_true')
    p.add_argument('--assume-gripper-zero',action='store_true',help='Use g_r=0 model approximation for empty gripper; not a measured gripper angle')
    p.add_argument('--gain',type=float,default=.1)
    runtime=p.add_mutually_exclusive_group()
    runtime.add_argument('--duration-s',type=float,default=None)
    runtime.add_argument('--continuous',action='store_true',help='Run until Ctrl+C or a safety/communication limit; no time cutoff')
    p.add_argument('--log',default='/tmp/pallet_j5_gravity_'+str(time.time_ns())+'.jsonl')
    args=p.parse_args()
    if args.read_only and args.live:p.error('Choose read-only or live')
    if args.continuous and not args.live:p.error('--continuous requires --live')
    cfg=yaml.safe_load(Path(args.config).read_text());limits=GravityLimits(gain=args.gain,duration_s=1. if args.duration_s is None else args.duration_s);limits.validate()
    if cfg['joints']['J5']['motor_id']!=5 or cfg['joints']['J5']['model']!='RS05':p.error('Expected J5 RS05 ID5')
    mapping=MotorJointMapping(args.config,args.urdf,False);model=GravityModel(args.urdf)
    if not args.live:
        raw=dict(zip(range(1,7),args.motor_positions))
        if args.read_only:
            b=ReadOnlySerialMotorBus(args.port,mapping.host_id)
            try:raw=b.read(mapping.ids,.5)
            finally:b.close()
        print(json.dumps({'mode':'READ ONLY' if args.read_only else 'OFFLINE','raw_motor_rad':raw},indent=2),flush=True)
        q,tau=calculate(model,mapping,raw,cfg['gravity_root_m_s2'])
        c=cfg['joints']['J5'];cmd=output(tau,0.,c['direction'],c['ratio'],limits.ramp_s,0.,limits.period_s,limits)
        print(json.dumps({'joint_rad':q,'gravity_j5_nm':tau,'nominal_nm_per_apk':RS05_NOMINAL_NM_PER_APK,
                          'gripper_assumption':'g_r=0, no payload','limits':asdict(limits),'initial_step':cmd},indent=2))
        return
    if not args.arm_supported or not args.assume_gripper_zero:p.error('Live requires supported arm and explicit empty-gripper g_r=0 modelling assumption')
    if not cfg['joints']['J5'].get('watchdog_zero_current_state_test',{}).get('verified'):p.error('J5 watchdog preflight missing')
    with open(args.log,'x',buffering=1) as f:
        def log(row):f.write(json.dumps(row)+'\n')
        log({'mode':'LIMITED PARTIAL GRAVITY EXPERIMENT','continuous':args.continuous,'time_cutoff_s':None if args.continuous else limits.duration_s,'limits':asdict(limits),'nominal_nm_per_apk':RS05_NOMINAL_NM_PER_APK,'gripper_assumption':'empty g_r=0, unmeasured','fixed_axes_periodic_geometry_only':True})
        print(('Continuous J5 compensation: Ctrl+C to stop.' if args.continuous else 'Timed J5 compensation.')+' Log: '+args.log,flush=True)
        b=J5Bus(args.port,5,0x7028,limits.current_a)
        try:live(b,model,mapping,cfg,limits,log,continuous=args.continuous)
        except KeyboardInterrupt:
            log({'stopped_by':'interrupt','cleanup_confirmed':True})
            print('Stopped: Reset and zero-current readback confirmed. Log: '+args.log,flush=True)
            raise
        except BaseException as e:
            log({'aborted':type(e).__name__,'reason':str(e)});raise
    print('Completed limited partial-gravity experiment; log='+args.log)


if __name__=='__main__':
    def interrupted(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    try:main()
    except KeyboardInterrupt:raise SystemExit(130)
