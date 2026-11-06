#!/usr/bin/env python3
"""Snapshot ID1..6 current limits, then apply exactly half from that snapshot."""
import argparse
import json
import math
from pathlib import Path
import struct
import time
from types import SimpleNamespace
from pallet_position_preflight import ReadOnlyParameters


class LimitBus(ReadOnlyParameters):
    def send(self,kind,data):
        if len(data)!=8 or int.from_bytes(data[:2],'little')!=0x7018:
            raise ValueError('Only current-limit register 0x7018 allowed')
        if kind==17:return super().send(kind,data)
        if kind!=18 or data[2:4]!=bytes(2):raise ValueError('Only limit read/write allowed')
        value=struct.unpack('<f',data[4:])[0]
        if not math.isfinite(value) or value<=0:raise ValueError('Invalid current limit')
        pause=.003-(time.monotonic()-self._last_tx)
        if pause>0:time.sleep(pause)
        arb=(18<<24)|(self.host<<8)|self.mid
        frame=b'AT'+((arb<<3)|4).to_bytes(4,'big')+b'\x08'+data+b'\r\n'
        if self.transport.serial.write(frame)!=len(frame):raise RuntimeError('Short write')
        self._last_tx=time.monotonic()


def read_all(bus):
    result={}
    for mid in range(1,7):
        bus.mid=mid;value=bus.read(0x7018,'<f',.15)
        if not math.isfinite(value) or value<=0:raise ValueError(f'ID{mid}: invalid limit')
        result[str(mid)]=value
    return result


def apply(bus,plan,save):
    before=plan['before_a'];targets=plan['target_a']
    if set(before)!=set(map(str,range(1,7))) or set(targets)!=set(before):raise ValueError('Expected IDs1..6')
    for mid in before:
        if not math.isfinite(before[mid]) or before[mid]<=0 or targets[mid]!=before[mid]*.5:raise ValueError('Invalid half-limit plan')
    current=read_all(bus)
    # Each axis must be at the original value or already at the planned value.
    # Re-running the same plan never halves the value a second time.
    for mid,value in current.items():
        if not any(math.isclose(value,v,rel_tol=1e-6,abs_tol=1e-6) for v in (before[mid],targets[mid])):
            raise RuntimeError(f'ID{mid}: limit changed since snapshot; refusing writes')
    plan['verified_a']={};save()
    for mid,target in targets.items():
        bus.mid=int(mid)
        if not math.isclose(current[mid],target,rel_tol=1e-6,abs_tol=1e-6):
            bus.send(18,b'\x18\x70\x00\x00'+struct.pack('<f',target))
            time.sleep(.01)
        value=bus.read(0x7018,'<f',.15)
        if not math.isclose(value,target,rel_tol=1e-6,abs_tol=1e-6):raise RuntimeError(f'ID{mid}: limit readback mismatch')
        plan['verified_a'][mid]=value;save()
    plan['complete']=True;save()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',default='/dev/ttyUSB0');p.add_argument('--baud',type=int,default=921600)
    p.add_argument('--snapshot',type=Path,required=True)
    p.add_argument('--apply',action='store_true')
    args=p.parse_args();bus=None
    try:
        if args.apply:
            plan=json.loads(args.snapshot.read_text())
        else:
            # Refuse to overwrite baseline before opening hardware.
            with args.snapshot.open('x') as f:f.write('{}\n')
        bus=LimitBus(args.port,1,args.baud)
        def save():args.snapshot.write_text(json.dumps(plan,indent=2)+'\n')
        if args.apply:apply(bus,plan,save)
        else:
            before=read_all(bus)
            plan={'register':'0x7018','before_a':before,'target_a':{k:v*.5 for k,v in before.items()},
                  'complete':False,'motion_commands_sent':False,'persistence_after_power_cycle':'not tested'}
            save()
        print(json.dumps(plan,indent=2))
    finally:
        if bus:bus.close()


if __name__=='__main__':main()
