#!/usr/bin/env python3
"""Hold current positions of IDs1..7 in PP mode. OFFLINE by default."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import signal
import time
from ament_index_python.packages import get_package_share_directory
from serial_trajectory_core import NAMES,load_config,SerialBackend
from single_axis_position_core import Joint


def hold(backend,period,duration,log,clock=time.monotonic,sleep=time.sleep):
    """Hold captured positions, never zero targets. STOP on every exit path."""
    try:
        target=backend.sample()
        if len(target)!=7 or not all(math.isfinite(v) for v in target):raise ValueError('Need seven finite measured positions')
        log({'event':'captured_hold_target','target_rad':target})
        backend.prepare(target)
        log({'event':'holding','current_limits_a':getattr(backend,'observed_current_limits',{}),
             'note':'PP current-position hold; not a specified torque or grip force'})
        began=clock();last_log=-1.
        while duration==0 or clock()-began<duration:
            tick=clock();backend.command(target);actual=backend.sample()
            elapsed=clock()-began
            errors=[abs(a-b) for a,b in zip(actual,target)]
            if len(actual)!=7 or max(errors)>.01:
                log({'event':'hold_error','target_rad':target,'actual_rad':actual})
                raise RuntimeError('Hold error exceeds 0.01rad')
            cycle=clock()-tick
            if cycle>period:raise RuntimeError('Hold control cycle overrun')
            if elapsed-last_log>=.5:
                log({'event':'hold_sample','elapsed_s':elapsed,'actual_rad':actual,'max_error_rad':max(errors),
                     'cycle_ms':cycle*1000,'feedback':getattr(backend,'last_values',[])})
                last_log=elapsed
            sleep(max(0.,period-(clock()-tick)))
    finally:
        backend.stop();log({'event':'stop_confirmed_all_7'})


def configuration(root):
    config,joints=load_config(root/'config/serial_trajectory_live_tested.yaml',root,live=True)
    # ID7 is controlled in raw motor radians, NOT URDF finger joint radians.
    # +/-12 is the user's measured calibration range, not a mechanical rating.
    joints.append(Joint('ID7_gripper_raw',7,1,0.,1.,-12.,12.))
    config['current_trip_a']['ID7_gripper_raw']=None
    config['max_joint_travel_rad']=.02
    # Holding is a separate tool; MoveIt speed increases must not silently
    # increase the hold-only tool's monitoring envelope.
    config.update(velocity_rad_s=.1,acceleration_rad_s2=.1,speed_trip_rad_s=.2)
    return config,joints


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live',action='store_true')
    parser.add_argument('--arm-supported',action='store_true')
    parser.add_argument('--duration',type=float,default=0.,help='Seconds; 0 holds until Ctrl+C')
    parser.add_argument('--port',default='/dev/ttyUSB0')
    parser.add_argument('--log',type=Path)
    args=parser.parse_args()
    if not math.isfinite(args.duration) or args.duration<0:parser.error('Duration must be finite and nonnegative')
    config,joints=configuration(Path(get_package_share_directory('pallet')))
    config['serial_port']=args.port
    if not args.live:
        print(json.dumps({'mode':'OFFLINE_NO_DEVICE_ACCESS','motor_ids':list(range(1,8)),
                          'target':'fresh measured angles at LIVE startup, not zero',
                          'velocity_rad_s':.1,'acceleration_rad_s2':.1,
                          'expected_arm_current_limits_a':config['expected_current_limit_a'],
                          'gripper_current_limit':'preserve existing; read at runtime',
                          'motor_profile_policy':config.get('motor_profile_policy','write_command_limits'),
                          'duration_s':args.duration,'actuation':False},indent=2));return
    if not args.arm_supported:parser.error('LIVE requires --arm-supported')
    path=args.log or Path.cwd()/'measurements/torque_hold'/f'hold_all_{datetime.now():%Y%m%d_%H%M%S_%f}.jsonl'
    path.parent.mkdir(parents=True,exist_ok=True)
    backend=None
    def interrupted(*_):raise KeyboardInterrupt
    previous=signal.signal(signal.SIGTERM,interrupted)
    try:
        with path.open('x',buffering=1) as stream:
            def log(row):
                text=json.dumps(row,allow_nan=False);stream.write(text+'\n');print(text,flush=True)
            log({'event':'live_configuration','ids':list(range(1,8)),'log':str(path.resolve()),'duration_s':args.duration,
                 'velocity_rad_s':config['velocity_rad_s'],'acceleration_rad_s2':config['acceleration_rad_s2']})
            backend=SerialBackend(config,joints)
            hold(backend,config['period_s'],args.duration,log)
    except KeyboardInterrupt:print('Interrupted; check stop_confirmed_all_7 in the log.')
    except (ValueError,RuntimeError,OSError) as error:parser.exit(1,f'Error: {error}; log: {path}\n')
    finally:
        if backend:backend.close()
        signal.signal(signal.SIGTERM,previous)


if __name__=='__main__':main()
