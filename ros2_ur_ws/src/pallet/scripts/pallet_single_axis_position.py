#!/usr/bin/env python3
"""Single-arm-joint PP commissioning. Default OFFLINE; no serial device opened."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import signal
import yaml
from ament_index_python.packages import get_package_share_directory
from single_axis_position_core import load_joint,Limits,make_plan,PositionTestBus,run


def main():
    root=Path(get_package_share_directory('pallet'))
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--joint',choices=[f'J{i}' for i in range(1,7)],required=True)
    p.add_argument('--delta-rad',type=float,required=True,help='URDF joint delta; zero for hold-only test')
    p.add_argument('--start-joint-rad',type=float,default=0.,help='OFFLINE simulation only; LIVE always measures current angle')
    p.add_argument('--mapping',type=Path,default=root/'config/hand_guiding.yaml')
    p.add_argument('--limits',type=Path,default=root/'config/single_axis_position.yaml')
    p.add_argument('--port',default='/dev/ttyUSB0')
    p.add_argument('--live',action='store_true')
    p.add_argument('--arm-supported',action='store_true')
    p.add_argument('--log',type=Path)
    args=p.parse_args()
    try:
        joint=load_joint(args.mapping,root/'urdf/pallet_with_gripper.urdf',args.joint)
        doc=yaml.safe_load(args.limits.read_text())
        if not args.live:
            # Use selected settings, with explicitly marked offline fallbacks
            # for physical limits that have not yet been measured.
            defaults=asdict(Limits(.1,.2,.1,.1,.03,.001))
            simulated_fields=[]
            for key,value in doc['limits'].items():
                if value is not None:defaults[key]=value
                else:simulated_fields.append(key)
            if doc.get('current_limit_policy')=='preserve_existing':
                defaults['current_limit_a']=None
                simulated_fields.remove('current_limit_a')
                if doc['limits'].get('current_trip_a') is None:
                    defaults['current_trip_a']=None
                    simulated_fields.remove('current_trip_a')
            limits=Limits(**defaults)
            plan=make_plan(joint,joint.raw(args.start_joint_rad),args.delta_rad,limits)
            print(json.dumps({'mode':'OFFLINE_NO_DEVICE_ACCESS','actuation':False,'current_limit_policy':doc.get('current_limit_policy','write_explicit'),'simulated_limits':asdict(limits),'offline_fallback_fields':simulated_fields,**plan},indent=2))
            return
        if doc.get('joint_scope')!=args.joint:
            p.error('LIVE requires a joint-scoped commissioning profile matching --joint')
        if not args.arm_supported or doc.get('pp_watchdog_verified') is not True:
            p.error('LIVE requires externally supported arm and separately verified PP watchdog; current-mode watchdog results do not qualify')
        policy=doc.get('current_limit_policy','write_explicit')
        if policy not in ('preserve_existing','write_explicit'):p.error('Unknown current limit policy')
        if policy=='preserve_existing' and doc['limits'].get('current_limit_a') is not None:
            p.error('Preserve policy requires current_limit_a: null; existing value is read at runtime')
        if policy=='write_explicit' and doc['limits'].get('current_limit_a') is None:
            p.error('Explicit current-limit policy requires a value')
        limits=Limits(**doc['limits']);limits.validate()
        if args.log is None:p.error('LIVE requires a new --log file')
        if not math.isfinite(args.delta_rad) or abs(args.delta_rad)>=limits.travel_rad or (args.delta_rad!=0 and abs(args.delta_rad)<=2*limits.goal_tolerance_rad):
            p.error('Delta must be finite, fit the travel bound, and exceed twice tolerance (or be zero)')
        # Open log before hardware, no overwrite of previous experiment data.
        with args.log.open('x',buffering=1) as stream:
            def log(row):stream.write(json.dumps(row,allow_nan=False)+'\n');print(json.dumps(row),flush=True)
            log({'event':'live_configuration','joint':args.joint,'limits':asdict(limits)})
            bus=PositionTestBus(args.port,joint,limits)
            def interrupted(*_):raise KeyboardInterrupt
            previous=signal.signal(signal.SIGTERM,interrupted)
            try:run(bus,joint,args.delta_rad,limits,log,expected_current_limit_a=doc.get('expected_preserved_current_limit_a'))
            finally:bus.close();signal.signal(signal.SIGTERM,previous)
    except KeyboardInterrupt:
        p.exit(130,'Interrupted; inspect STOP confirmation in log.\n')
    except (OSError,ValueError,RuntimeError,TypeError,KeyError) as error:
        p.exit(1,f'Error: {error}\n')


if __name__=='__main__':main()
