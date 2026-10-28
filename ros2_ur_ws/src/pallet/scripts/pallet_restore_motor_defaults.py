#!/usr/bin/env python3
"""Restore original arm current limits and PP defaults, stopped, IDs1..7."""
import argparse
import json
import math
from pathlib import Path
import struct
import time
from pallet_position_preflight import ReadOnlyParameters
from single_axis_current_core import CurrentTestBus


class RestoreBus(ReadOnlyParameters):
    stop_and_confirm=CurrentTestBus.stop_and_confirm
    def __init__(self,port,targets):
        super().__init__(port,1,921600);self.targets=targets
    def stop(self):self.send(4,bytes(8))
    def send(self,kind,data):
        if len(data)!=8 or kind not in (4,17,18):raise ValueError('Restore permits only STOP and limit/profile access')
        index=int.from_bytes(data[:2],'little')
        if kind==4:
            if data!=bytes(8):raise ValueError('Invalid STOP')
        else:
            if index not in (0x7018,0x7024,0x7025):raise ValueError('Register not permitted')
            if kind==17 and data[2:]!=bytes(6):raise ValueError('Invalid read')
            if kind==18:
                if data[2:4]!=bytes(2):raise ValueError('Invalid write')
                value=struct.unpack('<f',data[4:])[0]
                expected=self.targets[str(self.mid)] if index==0x7018 else 10.
                if not math.isfinite(value) or value!=expected:raise ValueError('Only recorded current limits and PP10/10 defaults permitted')
        pause=.003-(time.monotonic()-self._last_tx)
        if pause>0:time.sleep(pause)
        arb=(kind<<24)|(self.host<<8)|self.mid
        frame=b'AT'+((arb<<3)|4).to_bytes(4,'big')+b'\x08'+data+b'\r\n'
        if self.transport.serial.write(frame)!=len(frame):raise RuntimeError('Short write')
        self._last_tx=time.monotonic()
    def write(self,index,value):self.send(18,index.to_bytes(2,'little')+bytes(2)+struct.pack('<f',value))


def restore(bus,report,save):
    # Confirm every motor stopped before changing any setting.
    for mid in range(1,8):bus.mid=mid;bus.stop_and_confirm()
    report['all_stop_confirmed']=True;save()
    before={}
    for mid in range(1,8):
        bus.mid=mid;before[str(mid)]=bus.read(0x7018,'<f',.15)
        if not math.isfinite(before[str(mid)]) or before[str(mid)]<=0:raise ValueError('Invalid original current limit')
    bus.targets['7']=before['7'] # ID7 was not halved; preserve its actual value.
    report.update(before_current_a=before,target_current_a=dict(bus.targets),verified_current_a={},profile_writes={});save()
    for mid in range(1,8):
        bus.mid=mid;key=str(mid)
        bus.write(0x7018,bus.targets[key]);time.sleep(.01)
        actual=bus.read(0x7018,'<f',.15)
        if not math.isclose(actual,bus.targets[key],abs_tol=1e-6,rel_tol=1e-6):raise RuntimeError(f'ID{mid}: current readback mismatch')
        report['verified_current_a'][key]=actual;save()
        bus.write(0x7024,10.);bus.write(0x7025,10.)
        # RS05/RS06 PP fields are documented W: record transmission, not readback.
        report['profile_writes'][key]={'velocity_rad_s':10.,'acceleration_rad_s2':10.,'readback_verified':False};save()
    report['complete']=True;save()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--apply',action='store_true');p.add_argument('--port',default='/dev/ttyUSB0')
    p.add_argument('--output',type=Path)
    a=p.parse_args();before=json.loads(a.baseline.read_text())['before_a']
    if set(before)!=set(map(str,range(1,7))) or any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in before.values()):p.error('Invalid original ID1..6 baseline')
    report={'complete':False,'mode':'APPLY' if a.apply else 'OFFLINE','original_arm_current_a':before,
            'pp_default_velocity_rad_s':10.,'pp_default_acceleration_rad_s2':10.,'enable_or_motion_commands_sent':False,
            'flash_save_sent':False,'power_cycle_persistence':'not tested'}
    if not a.apply:print(json.dumps(report,indent=2));return
    if a.output is None:p.error('--apply requires new --output')
    bus=None
    with a.output.open('x') as stream:
        def save():
            stream.seek(0);json.dump(report,stream,indent=2);stream.write('\n');stream.truncate();stream.flush()
        save()
        try:
            bus=RestoreBus(a.port,dict(before));restore(bus,report,save)
        except BaseException as e:
            report['failure']=str(e);save();raise
        finally:
            if bus:bus.close()
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
