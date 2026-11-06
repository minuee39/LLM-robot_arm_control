#!/usr/bin/env python3
"""Compare one fixed J5 target with 10Hz ramp, holding other arm joints. Offline by default."""
from dataclasses import replace
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
    except BaseException as error:
        log({'event':'operation_failure','reason':str(error)})
        raise
    finally:
        backend.stop();log({'event':'stop_confirmed_all_6'})



def visual_move(backend,config,joints,log,clock=time.monotonic,sleep=time.sleep):
    """J5 +20deg cosine reference over 10s, then measured 5s final hold."""
    delta=math.radians(20);duration=10.;hold_s=5.
    try:
        start=backend.sample();target=list(start);target[4]+=delta
        if not all(j.lower<=q<=j.upper for j,q in zip(joints,target)):
            raise ValueError('J5 +20deg target exceeds URDF limits')
        log({'event':'visual_plan','start_rad':start,'target_rad':target,'delta_J5_deg':20,
             'move_s':duration,'hold_s':hold_s,'collision_check':False})
        backend.prepare(start);began=clock();reached=None;last_log=-1.
        while clock()-began<=duration+config['settle_timeout_s']+hold_s:
            tick=clock();elapsed=tick-began
            fraction=(1-math.cos(math.pi*min(elapsed/duration,1.)))/2
            desired=list(start);desired[4]+=delta*fraction
            backend.command(desired);actual=backend.sample();now=clock()
            if max(abs(a-d) for a,d in zip(actual,desired))>config['max_path_error_rad']:
                raise RuntimeError('Visual move tracking error exceeds configured tolerance')
            if any(abs(actual[i]-start[i])>.005 for i in (0,1,2,3,5)):
                raise RuntimeError('Other arm axis drift exceeds 0.005rad')
            if abs(actual[4]-start[4])>delta+.02:raise RuntimeError('J5 measured travel exceeds visual test envelope')
            if now-tick>config['period_s']:raise RuntimeError('Visual move cycle overrun')
            in_goal=max(abs(a-t) for a,t in zip(actual,target))<=config['goal_tolerance_rad']
            if elapsed>=duration and in_goal:
                if reached is None:
                    reached=now;log({'event':'holding_final_position','hold_s':hold_s})
                if now-reached>=hold_s:
                    log({'event':'visual_result','held_s':now-reached,'actual_rad':actual});return
            elif reached is not None:raise RuntimeError('Final hold goal tolerance violated')
            if reached is None and now-began>duration+config['settle_timeout_s']:
                raise RuntimeError('Visual goal settling timeout')
            if elapsed-last_log>=.5:
                log({'event':'visual_sample','elapsed_s':elapsed,'desired_rad':desired,'actual_rad':actual,
                     'feedback':getattr(backend,'last_values',[])})
                last_log=elapsed
            sleep(max(0.,config['period_s']-(clock()-tick)))
        raise RuntimeError('Visual move timeout')
    except BaseException as error:
        # Record the original fault before cleanup; a missing STOP ACK must
        # not hide the feedback/setup failure which triggered cleanup.
        log({'event':'operation_failure','reason':str(error)})
        raise
    finally:
        backend.stop();log({'event':'stop_confirmed_all_6'})



class SlowJ5ProfileBackend(SerialBackend):
    """Temporary J5-only PP .1/.1; restore recorded 10/10 after confirmed STOP.

    PP profile registers are write-only on the target hardware: restoration
    records transmission only, never claims a verified readback or snapshot.
    """
    def __init__(self,config,joints,log):
        if config.get('motor_profile_policy')!='preserve_existing':
            raise ValueError('Profile experiment requires preserve_existing')
        self.profile_changed=False;self.profile_log=log
        super().__init__(config,joints)

    def prepare(self,start,should_cancel=lambda:False):
        # Change profile only while all axes are confirmed stopped.
        self.stop()
        axis=self.axes[4]
        if axis.joint.name!='J5':raise ValueError('Expected J5 at index4')
        # Allow restoration of the recorded motor-raw 10/10 values through the
        # existing command whitelist; software speed monitors stay unchanged.
        axis.limits=replace(axis.limits,velocity_rad_s=max(axis.limits.velocity_rad_s,10/axis.joint.ratio),
                            acceleration_rad_s2=max(axis.limits.acceleration_rad_s2,10/axis.joint.ratio))
        self.profile_changed=True  # Partial writes must also trigger restoration.
        axis.write(0x7024,.1*axis.joint.ratio,'<f')
        axis.write(0x7025,.1*axis.joint.ratio,'<f')
        self.profile_log({'event':'J5_profile_written','joint_velocity_rad_s':.1,'joint_acceleration_rad_s2':.1,
                          'readback_verified':False,'restore_raw_values':[10.,10.]})
        super().prepare(start,should_cancel)

    def stop(self):
        # A failed STOP must prevent profile writes on an unconfirmed motor.
        super().stop()
        if self.profile_changed:
            axis=self.axes[4]
            axis.write(0x7024,10.,'<f');axis.write(0x7025,10.,'<f')
            self.profile_changed=False
            self.profile_log({'event':'J5_profile_restore_transmitted','raw_velocity_rad_s':10.,
                              'raw_acceleration_rad_s2':10.,'readback_verified':False})


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--live',action='store_true');p.add_argument('--arm-supported',action='store_true')
    p.add_argument('--log',type=Path)
    p.add_argument('--visual-20deg',action='store_true',help='J5 +20deg over 10s then hold 5s; skips fixed-step comparison')
    p.add_argument('--j5-pp-slow',action='store_true',help='Temporary J5 PP .1/.1, then restore recorded raw10/10; visual test only')
    a=p.parse_args()
    if a.j5_pp_slow and not a.visual_20deg:p.error('--j5-pp-slow requires --visual-20deg')
    root=Path(get_package_share_directory('pallet'))
    config,joints=load_config(root/'config/serial_trajectory_live_tested.yaml',root,live=True)
    config.update(max_joint_travel_rad=math.radians(20)+.02 if a.visual_20deg else .03)
    if not a.live and a.visual_20deg:
        print(json.dumps({'mode':'OFFLINE_NO_DEVICE_ACCESS','J5_delta_deg':20,'move_s':10,'hold_s':5,'j5_pp_slow':a.j5_pp_slow,'profile_readback_verified':False,
                          'other_arm_joints':'hold initial positions','gripper_id7':'not controlled','collision_check':False}));return
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
            backend=SlowJ5ProfileBackend(config,joints,log) if a.j5_pp_slow else SerialBackend(config,joints)
            try:
                if a.visual_20deg:visual_move(backend,config,joints,log)
                else:compare(backend,config,log)
            except BaseException as error:log({'event':'failure','reason':str(error)});raise
            finally:
                # Profile restoration can emit log events. Finish cleanup while
                # the measurement stream is still open, including on failure.
                try:
                    if getattr(backend,'profile_changed',False):
                        backend.stop()
                        log({'event':'stop_confirmed_all_6'})
                except BaseException as error:
                    log({'event':'cleanup_failure','reason':str(error)})
                    raise
                finally:backend.close()
    except KeyboardInterrupt:p.exit(130,'Interrupted; inspect STOP confirmation.\n')
    except (RuntimeError,ValueError,OSError) as error:p.exit(1,str(error)+'\n')
    finally:
        signal.signal(signal.SIGTERM,previous)


if __name__=='__main__':main()
