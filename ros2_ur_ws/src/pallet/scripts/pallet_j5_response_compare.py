#!/usr/bin/env python3
"""Compare one fixed J5 target with 10Hz ramp, holding other arm joints. Offline by default."""
import argparse
import json
import math
from pathlib import Path
import signal
import time
from ament_index_python.packages import get_package_share_directory
from serial_trajectory_core import load_config,SerialBackend


def compare(backend,config,log,clock=time.monotonic,sleep=time.sleep):
    results=[]
    try:
        start=backend.sample();target=list(start);target[4]+=.01
        log({'event':'comparison_plan','start_rad':start,'target_rad':target,'delta_J5_rad':.01})
        backend.prepare(start)
        for mode in ('fixed','stream'):
            # Both trials start with torque enabled at the same captured pose.
            if mode=='stream':
                backend.command(start);began=clock();held=None
                while clock()-began<3.:
                    tick=clock();q=backend.sample()
                    if max(abs(a-b) for a,b in zip(q,start))<=.001:
                        held=clock() if held is None else held
                        if clock()-held>=.3:break
                    else:held=None
                    sleep(max(0.,config['period_s']-(clock()-tick)))
                else:raise RuntimeError('Failed to return to baseline before comparison')
            log({'event':'trial_begin','mode':mode})
            began=clock();rows=[];held=None;settled=None
            if mode=='fixed':backend.command(target) # Exactly ONE target write per motor in this trial.
            while clock()-began<2.5:
                tick=clock();elapsed=tick-began
                desired=list(start);desired[4]+=min(elapsed,1.)*.01
                if mode=='fixed':desired=list(target)
                else:backend.command(desired)
                actual=backend.sample();now=clock()
                error=abs(actual[4]-desired[4]);final_error=abs(actual[4]-target[4])
                row={'event':'trial_sample','mode':mode,'elapsed_s':now-began,'desired_rad':desired,'actual_rad':actual,
                     'J5_tracking_error_rad':error,'J5_final_error_rad':final_error,'feedback':getattr(backend,'last_values',[])}
                rows.append(row);log(row)
                if any(abs(a-b)>.03 for a,b in zip(actual,start)):raise RuntimeError('Comparison position drift exceeds0.03rad')
                if now-tick>config['period_s']:raise RuntimeError('Comparison cycle overrun')
                if final_error<=.001:
                    held=now if held is None else held
                    if settled is None and now-held>=.5:settled=now-began
                else:held=None
                sleep(max(0.,config['period_s']-(clock()-tick)))
            result={'mode':mode,'goal_hold_passed':settled is not None,'settled_by_s':settled,
                    'peak_tracking_error_rad':max(r['J5_tracking_error_rad'] for r in rows),
                    'rms_tracking_error_rad':math.sqrt(sum(r['J5_tracking_error_rad']**2 for r in rows)/len(rows)),
                    'last_final_error_rad':rows[-1]['J5_final_error_rad']}
            results.append(result);log({'event':'trial_result',**result})
            if settled is None:raise RuntimeError(f'{mode}: J5 did not reach and hold goal')
        return results
    finally:
        backend.stop();log({'event':'stop_confirmed_all_6'})


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--live',action='store_true');p.add_argument('--arm-supported',action='store_true')
    p.add_argument('--log',type=Path)
    a=p.parse_args();root=Path(get_package_share_directory('pallet'))
    config,joints=load_config(root/'config/serial_trajectory_live_tested.yaml',root,live=True)
    config.update(max_joint_travel_rad=.03)
    if not a.live:
        print(json.dumps({'mode':'OFFLINE_NO_DEVICE_ACCESS','J5_delta_rad':.01,'fixed_target_writes_per_motor':1,
                          'stream_period_s':config['period_s'],'fixed_watchdog_traffic':'parameter reads only',
                          'other_arm_joints':'hold initial positions','gripper_id7':'not controlled','actuation':False},indent=2));return
    if not a.arm_supported or a.log is None:p.error('LIVE requires --arm-supported and a new --log')
    backend=None
    def interrupted(*_):raise KeyboardInterrupt
    previous=signal.signal(signal.SIGTERM,interrupted)
    try:
        with a.log.open('x',buffering=1) as stream:
            def log(row):text=json.dumps(row,allow_nan=False);stream.write(text+'\n');print(text,flush=True)
            log({'event':'configuration','config':config})
            backend=SerialBackend(config,joints)
            try:compare(backend,config,log)
            except BaseException as error:log({'event':'failure','reason':str(error)});raise
    except KeyboardInterrupt:p.exit(130,'Interrupted; inspect STOP confirmation.\n')
    except (RuntimeError,ValueError,OSError) as error:p.exit(1,str(error)+'\n')
    finally:
        if backend:backend.close()
        signal.signal(signal.SIGTERM,previous)


if __name__=='__main__':main()
