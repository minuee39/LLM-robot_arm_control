#!/usr/bin/env python3
"""Set stopped motors' hardware mechanical zero via RobStride AT USB-CAN."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parent/'ros2_ur_ws/src/pallet/scripts'))
from robstride_serial_readonly import ReadOnlySerialMotorBus


def action_frame(kind,mid,host=253):
    if kind not in (4,6) or not 1<=mid<=127:
        raise ValueError('Only STOP and SET_ZERO are allowed')
    payload=bytes([1,0,0,0,0,0,0,0]) if kind==6 else bytes(8)
    arb=(kind<<24)|(host<<8)|mid
    return b'AT'+((arb<<3)|4).to_bytes(4,'big')+b'\x08'+payload+b'\r\n'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',default='/dev/ttyUSB0')
    p.add_argument('--motor',type=int,nargs='+',default=[1,2,3,4,5,6])
    p.add_argument('--apply',action='store_true',help='STOP all selected motors and set mechanical zero; externally support arm first')
    p.add_argument('--record',default='/tmp/robstride_hardware_zero_result.json')
    args=p.parse_args()
    if len(set(args.motor))!=len(args.motor) or any(not 1<=i<=127 for i in args.motor):
        p.error('Unique IDs 1..127 required')
    reader=ReadOnlySerialMotorBus(args.port)
    record={'ids':args.motor,'completed_ids':[],'before_rad':{},'after_rad':{},'verified':False}
    def send(kind,mid):
        frame=action_frame(kind,mid)
        if reader.bus.serial.write(frame)!=len(frame):raise RuntimeError('Short serial write')
        time.sleep(.05)
    try:
        record['before_rad']=reader.read(args.motor,3.)
        print('Before:',record['before_rad'],flush=True)
        if not args.apply:
            print('Read-only preview. Use --apply to STOP and set hardware zero.');return
        for mid in args.motor:send(4,mid)
        time.sleep(.2)
        for mid in args.motor:
            send(6,mid)
            time.sleep(.1)
            value=reader.read([mid],1.)[mid]
            record['after_rad'][mid]=value
            if not math.isfinite(value) or abs(value)>.02:
                raise RuntimeError(f'ID {mid}: zero not verified ({value} rad). Stop; check firmware/mode. No further motors changed.')
            record['completed_ids'].append(mid)
            print(f'ID {mid}: hardware zero verified, raw={value:.6f} rad',flush=True)
        record['verified']=True
        print('All selected motors verified. No ENABLE command sent. Power-cycle persistence not tested.')
    finally:
        Path(args.record).write_text(json.dumps(record,indent=2)+'\n')
        reader.close()


if __name__=='__main__':
    main()
