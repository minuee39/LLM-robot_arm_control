#!/usr/bin/env python3
"""Bounded six-axis commissioning; offline unless --live --arm-supported."""
import argparse
import json
from pathlib import Path
import signal
import time
from ament_index_python.packages import get_package_share_directory
from serial_trajectory_core import NAMES,load_config,SerialBackend,DryBackend


def run(backend,joints,config,delta,log,clock=time.monotonic,sleep=time.sleep):
    try:
        start=backend.sample();target=[q+delta for q in start]
        if abs(delta)!=.01:raise ValueError('Commissioning delta must be +/-0.01rad')
        if any(not j.lower<=q<=j.upper for j,q in zip(joints,target)):raise ValueError('Target outside URDF')
        log({'event':'plan','start_rad':start,'target_rad':target})
        backend.prepare(start)
        log({'event':'enabled_all'})
        began=clock();reached=None;last=list(start);last_t=began
        # One-second ramp has only 0.01rad/s average goal motion. Motor PP
        # profile remains capped at velocity/acceleration 0.1.
        while clock()-began<4.:
            tick=clock();elapsed=tick-began
            desired=[s+min(elapsed,1.)*delta for s in start]
            backend.command(desired);actual=backend.sample();now=clock()
            row={'event':'sample','elapsed_s':now-began,'actual_rad':actual,'desired_rad':desired,
                 'values':getattr(backend,'last_values',[]),'cycle_ms':(now-tick)*1000}
            log(row)
            if now-tick>config['period_s']:raise RuntimeError('Control cycle overrun')
            if any(abs(q-s)>=config['max_joint_travel_rad'] for q,s in zip(actual,start)):raise RuntimeError('Travel trip')
            if now>last_t and any(abs(a-b)/(now-last_t)>=.2 for a,b in zip(actual,last)):raise RuntimeError('Angle discontinuity/speed trip')
            last=list(actual);last_t=now
            if elapsed>=1. and max(abs(a-b) for a,b in zip(actual,target))<=config['goal_tolerance_rad']:
                reached=now if reached is None else reached
                if now-reached>=.5:
                    log({'event':'goal_reached_and_hold_observed'});return
            else:reached=None
            sleep(max(0.,config['period_s']-(clock()-tick)))
        raise RuntimeError('Goal/hold timeout')
    finally:
        backend.stop();log({'event':'stop_confirmed_all'})


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--live',action='store_true');p.add_argument('--arm-supported',action='store_true')
    p.add_argument('--log',type=Path);p.add_argument('--delta-rad',type=float,default=.01,choices=[-.01,.01])
    args=p.parse_args();root=Path(get_package_share_directory('pallet'))
    config,joints=load_config(root/'config/serial_trajectory.yaml',root)
    # Explicit first commissioning profile, not a claim that these axes have
    # already passed the production FollowJointTrajectory commissioning gate.
    config.update(max_joint_travel_rad=.02,goal_tolerance_rad=.001,max_start_error_rad=.001,
                  speed_trip_rad_s=.2,velocity_confirmation_samples=2,current_trip_a={n:None for n in NAMES},
                  expected_current_limit_a=dict(zip(NAMES,[8.,28.5,8.,5.5,5.5,5.5])))
    if args.live and (not args.arm_supported or args.log is None):p.error('LIVE requires --arm-supported and a new --log')
    stream=args.log.open('x',buffering=1) if args.log else None
    def log(row):
        text=json.dumps(row,allow_nan=False)
        if stream:stream.write(text+'\n')
        print(text,flush=True)
    backend=None
    def interrupted(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    try:
        log({'event':'configuration','physical_actuation':args.live,'config':config})
        backend=SerialBackend(config,joints) if args.live else DryBackend()
        run(backend,joints,config,args.delta_rad,log)
    except KeyboardInterrupt:p.exit(130,'Interrupted; inspect STOP confirmation.\n')
    except (ValueError,RuntimeError,OSError) as error:
        log({'event':'failure','reason':str(error),'last_values':getattr(backend,'last_values',[])})
        p.exit(1,str(error)+'\n')
    finally:
        if backend:backend.close()
        if stream:stream.close()


if __name__=='__main__':main()
