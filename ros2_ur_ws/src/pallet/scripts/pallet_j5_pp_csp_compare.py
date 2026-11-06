#!/usr/bin/env python3
"""J2 or J5 PP/CSP comparison with identical 10Hz references; offline unless --live."""
import argparse
import json
import math
from pathlib import Path
import signal
import time
from ament_index_python.packages import get_package_share_directory
from serial_trajectory_core import SerialBackend,load_config
from single_axis_current_core import verify_register


AMPLITUDE_RAD=0.1
LEG_SECONDS=2.0
MOTION_SECONDS=2*LEG_SECONDS
TRIAL_SECONDS=MOTION_SECONDS+2.0
TRAVEL_LIMIT_RAD=AMPLITUDE_RAD+.02

def reference(start,t,axis_index=4):
    # Smooth outward-and-return motion; peak reference speed is about 0.07854 rad/s.
    q=list(start)
    if 0<=t<MOTION_SECONDS:q[axis_index]+=AMPLITUDE_RAD*.5*(1-math.cos(math.pi*t/LEG_SECONDS))
    return q


def compare_modes(backend,config,log,clock=time.monotonic,sleep=time.sleep,*,joint="J5"):
    axis_index={"J2":1,"J5":4}[joint]
    held_indices=[i for i in range(6) if i!=axis_index]
    error_key=f"{joint}_error_rad"
    results=[];baseline=None
    stage='read_baseline'
    mode=None
    try:
        baseline=backend.sample()
        for mode in (1,5):
            stage='check_baseline'
            start=backend.sample()
            if max(abs(a-b) for a,b in zip(start,baseline))>.001:
                raise RuntimeError('Baseline moved between trials; comparison aborted')
            target=reference(baseline,LEG_SECONDS,axis_index)
            for axis,q in zip(backend.axes,target):
                if not axis.joint.lower<=q<=axis.joint.upper:raise ValueError('Test target outside URDF')
            backend.axes[axis_index].position_mode=mode
            stage='prepare'
            backend.prepare(start)
            stage='motion'
            log({'event':'trial_begin','joint':joint,'mode':mode,'baseline_rad':baseline,'period_s':config['period_s'],
                 'PP_profile':'preserved existing 0x7024/0x7025','CSP_speed_limit_rad_s':.1})
            began=clock();rows=[];held=None
            # Indexed schedule sends the exact same references in both trials.
            for tick in range(round(TRIAL_SECONDS/config['period_s'])+1):
                deadline=began+tick*config['period_s']
                sleep(max(0,deadline-clock()))
                if clock()-deadline>config['period_s']:raise RuntimeError('Command scheduling overrun')
                desired=reference(baseline,tick*config['period_s'],axis_index)
                sent=clock();backend.command(desired);actual=backend.sample();now=clock()
                if now-deadline>config['period_s']:raise RuntimeError('Comparison cycle overrun')
                if any(abs(a-b)>TRAVEL_LIMIT_RAD for a,b in zip(actual,baseline)):
                    raise RuntimeError(f'Measured travel exceeds {TRAVEL_LIMIT_RAD}rad')
                if any(abs(actual[i]-baseline[i])>.005 for i in held_indices):
                    raise RuntimeError('Another joint failed to hold position')
                error=abs(actual[axis_index]-desired[axis_index])
                if joint=="J5" and error>0.01:
                    raise RuntimeError(f'{joint} tracking error exceeds 0.01rad')
                row={'event':'trial_sample','mode':mode,'tick':tick,'reference_time_s':tick*config['period_s'],
                     'send_time_s':sent-began,'sample_time_s':now-began,'desired_rad':desired,'actual_rad':actual,
                     error_key:error,'feedback':getattr(backend,'last_values',[])}
                rows.append(row);log(row)
                if tick*config['period_s']>=MOTION_SECONDS and max(abs(a-b) for a,b in zip(actual,baseline))<=.001:
                    held=now if held is None else held
                else:held=None
            if held is None or clock()-held<.5:raise RuntimeError('Failed to return and hold baseline')
            moving=[r for r in rows if r['reference_time_s']<=MOTION_SECONDS]
            result={'event':'trial_result','joint':joint,'mode':mode,'peak_error_rad':max(r[error_key] for r in moving),
                    'rms_error_rad':math.sqrt(sum(r[error_key]**2 for r in moving)/len(moving)),
                    'baseline_hold_passed':True}
            results.append(result);log(result)
            stage='trial_stop'
            backend.stop();log({'event':'stop_confirmed_all_6','mode':mode})
        return results
    except (Exception, KeyboardInterrupt) as error:
        log({'event':'trial_error','stage':stage,'mode':mode,'error':repr(error)})
        raise
    finally:
        try:
            backend.stop()
        except Exception as error:
            log({'event':'stop_unconfirmed','error':str(error)})
            raise
        log({'event':'stop_confirmed_all_6'})
        # Mode changes happen only after all STOP confirmations, never enabled.
        axis=backend.axes[axis_index];axis.position_mode=1
        axis.write(0x7005,1,'<B');verify_register(axis,0x7005,1,'<B')
        log({'event':f'{joint}_PP_mode_restored','CSP_limit_spd_rad_s':.1})


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--joint',choices=('J2','J5'),default='J5')
    access = parser.add_mutually_exclusive_group()
    access.add_argument('--live', action='store_true')
    access.add_argument('--read-modes', action='store_true')
    parser.add_argument('--arm-supported',action='store_true')
    parser.add_argument('--log',type=Path)
    args=parser.parse_args()
    root=Path(get_package_share_directory('pallet'))
    config,joints=load_config(root/'config/serial_trajectory_live_tested.yaml',root,live=True)
    config.update(period_s=.1,max_joint_travel_rad=TRAVEL_LIMIT_RAD,velocity_rad_s=.1,acceleration_rad_s2=.1,
                  speed_trip_rad_s=.2,motor_profile_policy='preserve_existing')
    peak_speed=AMPLITUDE_RAD/2*(math.pi/LEG_SECONDS)
    peak_acceleration=AMPLITUDE_RAD/2*(math.pi/LEG_SECONDS)**2
    if args.joint=='J2':
        config['acceleration_rad_s2']=.125
        if (peak_speed>config['velocity_rad_s'] or
                peak_acceleration>config['acceleration_rad_s2']):
            parser.error('Reference exceeds comparison speed or acceleration limit')
    if args.read_modes:
        backend = None
        try:
            backend = SerialBackend(config, joints)
            modes = {axis.joint.name: axis.read(0x7005, '<B', .1) for axis in backend.axes}
            j2 = next(axis for axis in backend.axes if axis.joint.name == 'J2')
            print(json.dumps({
                'run_modes': modes,
                'J2_PP': {
                    'vel_max_rad_s': j2.read(0x7024, '<f', .1) / j2.joint.ratio,
                    'acc_set_rad_s2': j2.read(0x7025, '<f', .1) / j2.joint.ratio,
                },
            }, allow_nan=False))
        except (RuntimeError, ValueError, OSError) as error:
            parser.exit(1, f'Read failed: {error}\n')
        finally:
            if backend is not None:
                backend.close()
        return
    if not args.live:
        print(json.dumps({'actuation':False,'device_access':False,'modes':[1,5],'moving_joint':args.joint,
                          'other_arm_joints':'PP hold current position','gripper_id7':'not controlled',
                          'delta_rad':AMPLITUDE_RAD,'out_and_back_s':MOTION_SECONDS,'trial_s':TRIAL_SECONDS,'period_s':.1,
                          'reference_peak_speed_rad_s':peak_speed,'reference_peak_acceleration_rad_s2':peak_acceleration,
                          'CSP_speed_limit_rad_s':.1,'exit':f'STOP all six, restore {args.joint} PP; torque released'},indent=2))
        return
    if not args.arm_supported or args.log is None:parser.error('--live requires --arm-supported and a new --log')
    backend=None
    def interrupted(*_):raise KeyboardInterrupt
    previous=signal.signal(signal.SIGTERM,interrupted)
    try:
        with args.log.open('x',buffering=1) as stream:
            def log(row):
                text=json.dumps(row,allow_nan=False);stream.write(text+'\n');print(text,flush=True)
            backend=SerialBackend(config,joints)
            # Accept MIT or PP at startup. Backend preparation confirms STOP before writes.
            initial_modes = {axis.joint.name: axis.read(0x7005, '<B', .1)
                             for axis in backend.axes}
            log({'event': 'initial_modes', 'run_modes': initial_modes})
            unexpected = [f'{name}={mode}' for name, mode in initial_modes.items() if mode not in (0, 1)]
            if unexpected:
                raise RuntimeError('Expected MIT (0) or PP (1) at startup; found '+', '.join(unexpected))
            log({'event':'configuration','config':config})
            compare_modes(backend,config,log,joint=args.joint)
    except KeyboardInterrupt:parser.exit(130,'Interrupted; inspect STOP confirmation.\n')
    except (RuntimeError,ValueError,OSError) as error:parser.exit(1,str(error)+'\n')
    finally:
        if backend:backend.close()
        signal.signal(signal.SIGTERM,previous)


if __name__=='__main__':main()
