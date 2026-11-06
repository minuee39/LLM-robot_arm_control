#!/usr/bin/env python3
"""Read-only serial timing diagnosis. Default OFFLINE; never actuates motors."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time
from ament_index_python.packages import get_package_share_directory
from pallet_position_preflight import ReadOnlyParameters
from serial_trajectory_core import load_config,check_arm_fault
from single_axis_current_core import FeedbackTimeout


PHASES=('isolated_position','full_forward_3ms','full_reverse_3ms','full_forward_6ms')
TELEMETRY=(0x7019,0x701a,0x701b)
READ_TIMEOUT_S=.05


def validate_inputs(joints,samples):
    if type(samples) is not int or not 3<=samples<=50:
        raise ValueError('Diagnostic samples must be 3..50')
    ids=[joint.motor_id for joint in joints]
    if len(ids)!=6 or any(type(mid) is not int for mid in ids) or set(ids)!=set(range(1,7)):
        raise ValueError('Diagnosis requires exactly arm motor IDs 1..6')


def run_phase(reader,phase,joints,samples,log,clock=time.monotonic,sleep=time.sleep,context=None):
    """Run a bounded read sequence; only missing replies may continue the phase."""
    validate_inputs(joints,samples)
    if phase not in PHASES:raise ValueError('Unknown diagnostic phase')
    context={} if context is None else context
    ordered=sorted(joints,key=lambda joint:joint.motor_id,reverse=phase=='full_reverse_3ms')
    isolated=phase=='isolated_position'
    interval=.006 if phase=='full_forward_6ms' else .003
    period=.1 if isolated else .25
    reader._read_interval_s=interval
    cycles=([joint] for joint in ordered for _ in range(samples)) if isolated else (ordered for _ in range(samples))
    total=0;complete=0;timeouts=Counter();peak=0.;cycle_count=0
    log({'event':'diagnostic_phase_start','phase':phase,'samples_per_axis' if isolated else 'samples':samples,
         'request_interval_s':interval,'minimum_cycle_period_s':period,'read_timeout_s':READ_TIMEOUT_S,
         'attempts_per_read':1,'registers':[f'0x{index:04x}' for index in ((0x7019,) if isolated else TELEMETRY)]})
    for cycle_index,axes in enumerate(cycles):
        tick=clock();missing=0;request_index=0
        for joint in axes:
            reader.mid=joint.motor_id;reader.joint=joint
            for index in ((0x7019,) if isolated else TELEMETRY):
                context.clear()
                context.update(phase=phase,cycle_index=cycle_index,sample_index=cycle_index%samples,
                               joint=joint.name,motor_id=joint.motor_id,register=f'0x{index:04x}',
                               request_index=request_index)
                start=clock();total+=1;request_index+=1
                try:
                    value=reader.read(index,'<f',READ_TIMEOUT_S)
                    if not math.isfinite(value):raise RuntimeError(f'{joint.name}: non-finite diagnostic feedback')
                except FeedbackTimeout as error:
                    missing+=1;timeouts[(joint.name,index)]+=1
                    log(dict(context,event='diagnostic_timeout',time_s=start,elapsed_s=clock()-start,reason=str(error)))
                else:
                    log(dict(context,event='diagnostic_read',time_s=start,elapsed_s=clock()-start,value=value))
        elapsed=clock()-tick;peak=max(peak,elapsed);cycle_count+=1
        if not missing:complete+=1
        log({'event':'diagnostic_cycle','phase':phase,'cycle_index':cycle_index,
             'cycle_s':elapsed,'complete':missing==0,'timeouts':missing})
        sleep(max(0.,period-(clock()-tick)))
    context.clear()
    result={'event':'diagnostic_phase_summary','phase':phase,'cycles':cycle_count,
            'complete_cycles':complete,'total_reads':total,'timeouts':sum(timeouts.values()),
            'timeouts_by_register':[{'joint':joint,'register':f'0x{index:04x}','count':count}
                                    for (joint,index),count in sorted(timeouts.items())],
            'peak_cycle_s':peak,'request_interval_s':interval,'actuation':False}
    log(result);return result


def diagnose(reader,joints,samples,log,clock=time.monotonic,sleep=time.sleep):
    """Compare axis isolation, ordering and read spacing without motion approval."""
    validate_inputs(joints,samples)
    context={};transport=getattr(reader,'transport',None)
    trace_supported=transport is not None and hasattr(transport,'trace')
    prior_trace=transport.trace if trace_supported else None
    if trace_supported:transport.trace=lambda row:log(dict(row,**context))
    try:
        phases=[run_phase(reader,phase,joints,samples,log,clock,sleep,context) for phase in PHASES]
    finally:
        if trace_supported:transport.trace=prior_trace
    result={'event':'diagnostic_summary','phases':phases,'timeouts':sum(row['timeouts'] for row in phases),
            'all_reads_received':all(row['timeouts']==0 for row in phases),'raw_trace_supported':trace_supported,
            'qualified_for_motion':False,'actuation':False,'settings_modified':False,
            'note':'Relaxed single-read diagnostics do not qualify 100ms trajectory execution.'}
    log(result);return result


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    root=Path(get_package_share_directory('pallet'))
    p.add_argument('--config',type=Path,default=root/'config/serial_trajectory_live_tested.yaml')
    p.add_argument('--samples',type=int,default=10)
    p.add_argument('--read',action='store_true',help='Send parameter reads only; default is offline')
    p.add_argument('--output',type=Path)
    args=p.parse_args(argv)
    if not 3<=args.samples<=50:p.error('--samples must be 3..50')
    config,joints=load_config(args.config,root)
    validate_inputs(joints,args.samples)
    if not args.read:
        print(json.dumps({'mode':'OFFLINE_NO_DEVICE_ACCESS','samples':args.samples,'phases':PHASES,
                          'read_timeout_s':READ_TIMEOUT_S,'qualified_for_motion':False,'actuation':False}))
        return
    if args.output is None:p.error('--read requires a new --output JSONL path')
    reader=None
    try:
        with args.output.open('x',buffering=1) as stream:
            def log(row):
                text=json.dumps(row,allow_nan=False);stream.write(text+'\n')
                if row['event'] in ('diagnostic_phase_start','diagnostic_phase_summary','diagnostic_summary','diagnostic_failure'):
                    print(text,flush=True)
            try:
                reader=ReadOnlyParameters(config['serial_port'],1,config['baud'])
                reader.check_fault=check_arm_fault
                result=diagnose(reader,joints,args.samples,log)
            except Exception as error:
                log({'event':'diagnostic_failure','reason':str(error),'qualified_for_motion':False,'actuation':False})
                raise
            finally:
                if reader:reader.close();reader=None
        if not result['all_reads_received']:p.exit(1,'Read replies missing; inspect diagnostic_timeout events.\n')
    except (OSError,RuntimeError,ValueError) as error:p.exit(1,str(error)+'\n')
    finally:
        if reader:reader.close()


if __name__=='__main__':main()
