#!/usr/bin/env python3
"""PP watchdog commissioning at measured position; default offline, never seeks a new pose."""
import argparse
import json
from pathlib import Path
import signal
import time
from ament_index_python.packages import get_package_share_directory
from single_axis_position_core import PositionTestBus,Limits,load_joint
from single_axis_current_core import verify_register


class WatchdogBus(PositionTestBus):
    def send(self,kind,data=bytes(8)):
        if kind!=24:return super().send(kind,data)
        if data not in (bytes([1,2,3,4,5,6,0,0]),bytes([1,2,3,4,5,6,1,0])):
            raise ValueError('Only active-report switch allowed')
        pause=.003-(time.monotonic()-self._last_tx)
        if pause>0:time.sleep(pause)
        arb=(24<<24)|(self.host<<8)|self.mid
        frame=b'AT'+((arb<<3)|4).to_bytes(4,'big')+b'\x08'+data+b'\r\n'
        if self.transport.serial.write(frame)!=len(frame):raise RuntimeError('Short report-switch write')
        self._last_tx=time.monotonic()


def drain(bus):
    for _ in range(2048):
        if bus.transport.recv(0) is None:return
    raise RuntimeError('Receive backlog')


def observe_silence(bus,duration=.6,clock=time.monotonic):
    start=clock();trace=[]
    while clock()-start<duration:
        msg=bus.transport.recv(max(0,min(.02,duration-(clock()-start))))
        if msg is None:continue
        arb=msg.arbitration_id
        if len(msg.data)!=8 or (arb>>24)&31 not in (2,24) or (arb>>8)&255!=bus.mid or arb&255!=bus.host:continue
        row={'elapsed_s':clock()-start,'state':(arb>>22)&3,'fault_bits':(arb>>16)&63}
        trace.append(row)
        if row['fault_bits']:raise RuntimeError('Fault during PP silence: '+json.dumps(row))
    resets=[r for r in trace if r['state']==0]
    if not resets or trace[-1]['state']!=0:raise RuntimeError('No final Reset in passive PP observation')
    if not .125<=resets[0]['elapsed_s']<=.45:raise RuntimeError('Reset timing does not match 250ms watchdog window')
    if any(r['state']!=0 for r in trace[trace.index(resets[0]):]):raise RuntimeError('Motor returned to Run after Reset')
    return {'first_reset_s':resets[0]['elapsed_s'],'trace':trace}


def experiment(bus,joint,limits,record,clock=time.monotonic,sleep=time.sleep):
    try:
        bus.stop_and_confirm()
        current=bus.read(0x7019,'<f',.15)
        q=joint.measured(current)
        if not joint.lower<=q<=joint.upper:raise RuntimeError('Current joint outside URDF')
        bus.bind_window(current)
        record['initial_motor_rad']=current
        record['preserved_current_limit_a']=bus.read(0x7018,'<f',.15)
        for index,value,fmt in [(0x7028,5000,'<I'),(0x7005,1,'<B')]:
            bus.write(index,value,fmt);verify_register(bus,index,value,fmt,sleep)
        bus.write(0x7024,limits.velocity_rad_s*joint.ratio,'<f')
        bus.write(0x7025,limits.acceleration_rad_s2*joint.ratio,'<f')
        fresh=bus.read(0x7019,'<f',.15)
        if abs(fresh-current)/joint.ratio>limits.goal_tolerance_rad:raise RuntimeError('Axis moved during setup')
        bus.write(0x7016,fresh,'<f');verify_register(bus,0x7016,fresh,'<f',sleep)
        bus.send(24,bytes([1,2,3,4,5,6,1,0]))
        check=bus.read(0x7019,'<f',.15)
        if abs(check-fresh)/joint.ratio>limits.goal_tolerance_rad:raise RuntimeError('Axis moved before enable')
        bus.enable_and_confirm()
        record['run_confirmed']=True
        started=clock();held=[]
        while clock()-started<.15:
            tick=clock()
            position=bus.read(0x7019,'<f',limits.feedback_timeout_s)
            iq=bus.read(0x701a,'<f',limits.feedback_timeout_s)
            speed=bus.read(0x701b,'<f',limits.feedback_timeout_s)/joint.ratio
            if abs(position-current)/joint.ratio>=limits.travel_rad:raise RuntimeError('PP hold travel trip')
            if abs(iq)>=limits.current_trip_a:raise RuntimeError('PP hold current trip')
            if abs(speed)>limits.velocity_rad_s*1.1:raise RuntimeError('PP hold velocity trip')
            held.append({'motor_rad':position,'current_a':iq,'joint_velocity_rad_s':speed})
            bus.write(0x7016,fresh,'<f')
            remaining=limits.period_s-(clock()-tick)
            if remaining<=0:raise RuntimeError('Hold feedback deadline missed')
            sleep(remaining)
        record['hold_samples']=held
        # Establish a fresh final command; then absolutely no TX during observation.
        bus.write(0x7016,fresh,'<f');drain(bus)
        record['passive_observation']=observe_silence(bus,clock=clock)
        after=bus.read(0x7019,'<f',.15)
        iq=bus.read(0x701a,'<f',.15)
        record['final_motor_rad']=after;record['final_current_a']=iq
        record['joint_drift_rad']=(after-current)/joint.ratio
        if abs(record['joint_drift_rad'])>=limits.travel_rad:raise RuntimeError('PP silence/post-reset travel trip')
        if abs(iq)>=limits.current_trip_a:raise RuntimeError('Post-reset current trip')
        record['pp_run_to_reset_observed']=True
        # Transition is not a torque-cutoff certificate or approval for a load.
    finally:
        errors=[]
        try:bus.stop_and_confirm();record['stop_confirmed']=True
        except Exception as error:errors.append('STOP: '+str(error))
        try:bus.send(24,bytes([1,2,3,4,5,6,0,0]))
        except Exception as error:errors.append('report off: '+str(error))
        if errors:raise RuntimeError('Cleanup incomplete: '+'; '.join(errors))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--joint',choices=[f'J{i}' for i in range(1,7)],required=True)
    p.add_argument('--port',default='/dev/ttyUSB0');p.add_argument('--live',action='store_true')
    p.add_argument('--arm-supported',action='store_true');p.add_argument('--record',type=Path)
    args=p.parse_args()
    if not args.live:
        print('OFFLINE: no device access. PP at measured current position; preserve current cap; speed/acceleration0.1; watchdog250ms; passive TX silence600ms; require Reset; STOP.');return
    if not args.arm_supported or args.record is None:p.error('Require actual arm support and a new --record path')
    root=Path(get_package_share_directory('pallet'))
    joint=load_joint(root/'config/hand_guiding.yaml',root/'urdf/pallet_with_gripper.urdf',args.joint)
    # Bounded software trips for this hold-only experiment, not motor ratings.
    limits=Limits(None,.2,.1,.1,.02,.001,hold_s=.15)
    limits.validate()
    result={'joint':args.joint,'motor_id':joint.motor_id,'new_pose_commanded':False,
            'current_limit_policy':'preserve_existing','current_trip_a':.2,
            'travel_trip_joint_rad':.02,'pp_run_to_reset_observed':False,
            'loaded_torque_cutoff_verified':False,'passed':False}
    try:
        with args.record.open('x',buffering=1) as out:
            bus=None
            def interrupted(*_):raise KeyboardInterrupt
            previous=signal.signal(signal.SIGTERM,interrupted)
            try:
                bus=WatchdogBus(args.port,joint,limits)
                experiment(bus,joint,limits,result)
                result['passed']=True
            except BaseException as error:
                result['error']=str(error) or type(error).__name__
                raise
            finally:
                if bus:bus.close()
                signal.signal(signal.SIGTERM,previous)
                out.write(json.dumps(result,indent=2,allow_nan=False)+'\n')
                print(json.dumps(result,indent=2,allow_nan=False),flush=True)
    except KeyboardInterrupt:p.exit(130,'Interrupted; inspect cleanup result.\n')
    except (OSError,RuntimeError,ValueError) as error:p.exit(1,f'PP watchdog test failed: {error}\n')


if __name__=='__main__':main()
