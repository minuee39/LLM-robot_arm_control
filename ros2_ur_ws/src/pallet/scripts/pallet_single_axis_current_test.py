#!/usr/bin/env python3
"""Single-arm-axis current pulse test. Default is offline, without device access."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import signal
import sys
from single_axis_current_core import CurrentTestBus,Limits,pulse_current,run_experiment


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--motor',type=int,choices=range(1,7),required=True)
    p.add_argument('--sign',type=int,choices=(-1,1),required=True,help='Raw motor Iq sign, not URDF joint sign')
    p.add_argument('--current-a',type=float,default=.1)
    p.add_argument('--measured-current-limit-a',type=float,default=.2)
    p.add_argument('--slew-a-s',type=float,default=.1)
    p.add_argument('--duration-s',type=float,default=2.)
    p.add_argument('--travel-rad',type=float,default=.035)
    p.add_argument('--speed-rad-s',type=float,default=.15)
    p.add_argument('--period-s',type=float,default=.02,help='Control period; commissioning tool permits 20..50ms')
    p.add_argument('--port',default='/dev/ttyUSB0')
    p.add_argument('--live',action='store_true')
    p.add_argument('--arm-supported',action='store_true',help='Arm externally supported, other axes secured, physical stop available')
    p.add_argument('--watchdog-tested',action='store_true',help='Motor-side timeout behavior physically verified for this firmware')
    p.add_argument('--log',default='/tmp/pallet_single_axis_current_test.jsonl')
    args=p.parse_args()
    if not .02<=args.period_s<=.05:p.error('period-s must be 0.02..0.05')
    limits=Limits(period_s=args.period_s,current_a=args.current_a,measured_current_a=args.measured_current_limit_a,slew_a_s=args.slew_a_s,duration_s=args.duration_s,travel_rad=args.travel_rad,speed_rad_s=args.speed_rad_s)
    limits.validate()
    watchdog_index=0x7028
    print(json.dumps({'mode':'LIVE' if args.live else 'OFFLINE: no device access','motor':args.motor,'raw_iq_sign':args.sign,'limits':asdict(limits),'watchdog_index':hex(watchdog_index)},indent=2),flush=True)
    if not args.live:
        for i in range(11):
            t=limits.duration_s*i/10
            print(f't={t:.2f}s command_iq={pulse_current(t,limits,args.sign):+.4f}A')
        return
    if not args.arm_supported or not args.watchdog_tested:
        p.error('Live requires --arm-supported and --watchdog-tested; flags must reflect actual checks')
    # Open log before device, so filesystem errors cannot prevent cleanup after enable.
    with Path(args.log).open('w',buffering=1) as stream:
        def log(row):stream.write(json.dumps(row)+'\n');print(json.dumps(row),flush=True)
        bus=CurrentTestBus(args.port,args.motor,watchdog_index,limits.current_a)
        def interrupted(*_):raise KeyboardInterrupt
        previous=signal.signal(signal.SIGTERM,interrupted)
        try:run_experiment(bus,limits,args.sign,log)
        finally:
            bus.close();signal.signal(signal.SIGTERM,previous)


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:
        print('Interrupted; zero-current and STOP attempted.',file=sys.stderr)
        sys.exit(130)
