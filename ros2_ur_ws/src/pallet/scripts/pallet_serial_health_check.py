#!/usr/bin/env python3
"""Read-only six-axis telemetry qualification. No STOP, enable or settings writes."""
import argparse
import json
from pathlib import Path
import time
from ament_index_python.packages import get_package_share_directory
from serial_trajectory_core import load_config,ObserveBackend,FeedbackUnavailable,read_feedback


def qualify(backend,config,samples,log,clock=time.monotonic,sleep=time.sleep):
    complete=0;missed=0;consecutive=0;longest=0;retries=0;peak=0.
    budget=config['period_s']-.018
    for index in range(samples):
        tick=clock()
        try:
            q=backend.sample();elapsed=clock()-tick
            if elapsed>budget:raise FeedbackUnavailable('Telemetry exceeded execution read budget')
            complete+=1;consecutive+=1;longest=max(longest,consecutive)
            count=getattr(backend,'last_feedback_retries',0);retries+=count;peak=max(peak,elapsed)
            log({'event':'sample','index':index,'cycle_s':elapsed,'retries':count,'position_rad':q,
                 'feedback':getattr(backend,'last_values',[])})
        except FeedbackUnavailable as error:
            missed+=1;consecutive=0
            log({'event':'missing_sample','index':index,'reason':str(error),'cycle_s':clock()-tick,
                 'completed_joints':[v['joint'] for v in getattr(backend,'last_values',[])],
                 'read_retries_before_failure':getattr(backend,'last_feedback_retries',0)})
        # Fault frames, bad bounds and serial errors abort immediately.
        sleep(max(0.,config['period_s']-(clock()-tick)))
    result={'event':'health_summary','samples':samples,'complete':complete,'missing':missed,
            'longest_healthy_run':longest,'read_retries':retries,'peak_read_cycle_s':peak,
            'read_budget_s':budget,'passed':missed==0 and longest>=config['readiness_samples'],
            'actuation':False,'loaded_motion_verified':False}
    log(result);return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    root=Path(get_package_share_directory('pallet'))
    p.add_argument('--config',type=Path,default=root/'config/serial_trajectory_live_tested.yaml')
    p.add_argument('--samples',type=int,default=100)
    p.add_argument('--read',action='store_true')
    p.add_argument('--output',type=Path)
    args=p.parse_args()
    if not 20<=args.samples<=10000:p.error('--samples must be 20..10000')
    config,joints=load_config(args.config,root)
    if not args.read:
        print(json.dumps({'mode':'OFFLINE_NO_DEVICE_ACCESS','samples':args.samples,'actuation':False,
                          'period_s':config['period_s'],'read_budget_s':config['period_s']-.018}));return
    if args.output is None:p.error('--read requires a new --output JSONL path')
    backend=None
    try:
        with args.output.open('x',buffering=1) as stream:
            def log(row):
                stream.write(json.dumps(row,allow_nan=False)+'\n')
                if row['event']!='sample':print(json.dumps(row,allow_nan=False),flush=True)
            backend=ObserveBackend(config,joints)
            try:
                # Check actual hardware current limits without modifying them.
                for joint in joints:
                    backend.reader.mid=joint.motor_id;backend.reader.joint=joint
                    actual,retries=read_feedback(backend.reader,0x7018,time.monotonic()+.2,timeout=.1)
                    expected=config.get('expected_current_limit_a',{}).get(joint.name)
                    log({'event':'current_limit_read','joint':joint.name,'actual_a':actual,'expected_a':expected,'read_retries':retries})
                    if expected is not None and abs(actual-expected)>max(1e-6,abs(expected)*1e-6):
                        raise RuntimeError(f'{joint.name}: current limit mismatch, expected {expected}A, actual {actual}A')
                result=qualify(backend,config,args.samples,log)
            except Exception as error:
                log({'event':'health_failure','reason':str(error)});raise
            finally:backend.close();backend=None
        if not result['passed']:p.exit(1,'Telemetry qualification failed; inspect missing_sample events.\n')
    except (OSError,RuntimeError,ValueError) as error:p.exit(1,str(error)+'\n')
    finally:
        if backend:backend.close()


if __name__=='__main__':main()
