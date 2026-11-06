#!/usr/bin/env python3
"""Verify one arm motor's watchdog with zero Iq; never sends nonzero current."""
import argparse
import json
from pathlib import Path
import time
from single_axis_current_core import CurrentTestBus, prepare_current_mode


def drain(bus):
    for _ in range(2048):
        if bus.transport.recv(0) is None:return
    raise RuntimeError('Receive backlog')


def read_state_reply(bus,timeout=.15,expected_state=None):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        msg=bus.transport.recv(max(0,deadline-time.monotonic()))
        if msg is None:continue
        arb=msg.arbitration_id
        if (arb>>24)&31 in (2,24) and (arb>>8)&255==bus.mid and arb&255==bus.host:
            # Watchdog may set fault bits. Do not suppress them in the report.
            result={'state':(arb>>22)&3,'fault_bits':(arb>>16)&63}
            if expected_state is None or result['state']==expected_state or result['fault_bits']:
                return result
    raise RuntimeError('No type-2/type-24 status response; watchdog remains UNVERIFIED')



def passive_watch(bus,duration=.6,clock=time.monotonic):
    """Receive only. Do not query or write while observing the watchdog."""
    started=clock();trace=[]
    while clock()-started<duration:
        msg=bus.transport.recv(max(0,min(.02,duration-(clock()-started))))
        if msg is None:continue
        arb=msg.arbitration_id
        if (arb>>24)&31 not in (2,24) or (arb>>8)&255!=bus.mid or arb&255!=bus.host:continue
        state=(arb>>22)&3
        trace.append({'elapsed_s':clock()-started,'state':state,'fault_bits':(arb>>16)&63})
    if not trace:raise RuntimeError('No passive status reports; watchdog UNVERIFIED')
    resets=[row for row in trace if row['state']==0]
    if not resets or trace[-1]['state']!=0:
        raise RuntimeError(f'No stable Reset during TX silence; final state={trace[-1]["state"]}; watchdog UNVERIFIED')
    if resets[0]['elapsed_s']>.45:
        raise RuntimeError('Reset occurred too late for configured 250ms watchdog; UNVERIFIED')
    return {'first_reset_s':resets[0]['elapsed_s'],'last_status':trace[-1],'reports_received':len(trace)}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--motor',type=int,choices=range(1,8),required=True)
    p.add_argument('--port',default='/dev/ttyUSB0')
    p.add_argument('--live',action='store_true')
    p.add_argument('--arm-supported',action='store_true')
    p.add_argument('--record',default='/tmp/pallet_watchdog_test.json')
    args=p.parse_args()
    if not args.live:
        print('OFFLINE, no device access. Live procedure: STOP -> watchdog 250ms readback -> mode3, Iq0 readback -> ENABLE and verify Run(2) -> active reports enabled -> no TX for 600ms, receive only -> require Reset(0) -> STOP.');return
    if not args.arm_supported:p.error('--arm-supported required: support entire arm and secure other axes, physical stop available')
    record={'motor_id':args.motor,'passed':False,'nonzero_current_sent':False,'timeout_s':.25,'silent_s':.6}
    bus=CurrentTestBus(args.port,args.motor,0x7028,.1,allow_gripper=args.motor==7)
    try:
        bus.stop();time.sleep(.05)
        prepare_current_mode(bus,5000)
        bus.send(24,bytes([1,2,3,4,5,6,1,0]))
        # Confirm autonomous reporting while stopped before enabling.
        time.sleep(.02);drain(bus)
        read_state_reply(bus,.2)
        drain(bus);bus.send(3)
        before=read_state_reply(bus,expected_state=2)
        record['before']=before;print('Before silence:',before,flush=True)
        if before['state']!=2 or before['fault_bits']!=0:raise RuntimeError('Motor did not enter fault-free Run state')
        observation=passive_watch(bus)
        record['passive_observation']=observation
        print('During TX silence:',observation,flush=True)
        record['passed']=True
        print('PASS: Run(2) -> Reset(0) observed passively during TX silence. Iq command remained 0. Output-current removal under load was NOT measured.')
    finally:
        try:
            try:bus.stop()
            finally:bus.send(24,bytes([1,2,3,4,5,6,0,0]))
        finally:
            bus.close()
            Path(args.record).write_text(json.dumps(record,indent=2)+'\n')


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:print('Interrupted; cleanup attempted. Watchdog not verified.')
