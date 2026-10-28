#!/usr/bin/env python3
"""Read-only PP preflight; never enables, stops, zeros, or writes motor settings."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import time
from types import SimpleNamespace
from robstride_serial_readonly import SerialTransport
from single_axis_current_core import CurrentTestBus

# Do not read write-only PP velocity/acceleration settings.
REGISTERS = {
    0x7005: ('run_mode','<B'), 0x7016: ('loc_ref_motor_rad','<f'),
    0x7018: ('configured_current_limit_a','<f'),
    0x7019: ('measured_motor_rad','<f'),0x701a: ('measured_current_a','<f'),
    0x701b: ('measured_motor_velocity_rad_s','<f'),0x7028: ('watchdog_ticks','<I'),
}


class ReadOnlyParameters:
    # Reuse response filtering, source/host checks and fault rejection, without
    # inheriting any actuation/stop/write API.
    read = CurrentTestBus.read
    check_fault = CurrentTestBus.check_fault

    def __init__(self,port,motor,baud):
        if type(motor) is not int or not 1<=motor<=6:raise ValueError('Arm IDs1..6 only')
        self.mid,self.host=motor,253
        self._last_tx=float('-inf')
        self.transport=SerialTransport(port,baud)

    def send(self,kind,data):
        if kind!=17 or len(data)!=8 or int.from_bytes(data[:2],'little') not in REGISTERS or data[2:]!=bytes(6):
            raise ValueError('Preflight permits parameter-read requests only')
        pause=.003-(time.monotonic()-self._last_tx)
        if pause>0:time.sleep(pause)
        arb=(17<<24)|(self.host<<8)|self.mid
        self.transport.send(SimpleNamespace(arbitration_id=arb,data=data),.1)
        self._last_tx=time.monotonic()

    def close(self):self.transport.shutdown()


def collect(reader,motor):
    result={'motor_id':motor,'actuation':False,'settings_modified':False,
            'timestamp_utc':datetime.now(timezone.utc).isoformat(),'values':{},'read_errors':{}}
    for index,(name,fmt) in REGISTERS.items():
        try:result['values'][name]=reader.read(index,fmt,.15)
        except RuntimeError as error:
            # A missing register is explicit. Fault/backlog errors abort reads.
            if not str(error).startswith('Feedback timeout:'):raise
            result['read_errors'][name]=str(error)
    ticks=result['values'].get('watchdog_ticks')
    if ticks is not None:result['values']['watchdog_configured_s']=ticks/20000.
    result['complete']=not result['read_errors']
    result['pp_watchdog_verified']=False
    result['note']='Observed settings are not certified operating limits. PP TX-silence behavior is not tested. No commissioning config is modified.'
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--motor',type=int,choices=range(1,7),required=True)
    p.add_argument('--port',default='/dev/ttyUSB0');p.add_argument('--baud',type=int,default=921600)
    p.add_argument('--read',action='store_true',help='Open serial and send read requests only; default is offline')
    p.add_argument('--output',type=Path)
    args=p.parse_args()
    if not args.read:
        print(json.dumps({'mode':'OFFLINE_NO_DEVICE_ACCESS','motor_id':args.motor,'actuation':False,
                          'registers':{hex(k):v[0] for k,v in REGISTERS.items()}},indent=2));return
    stream=None;reader=None
    try:
        if args.output:stream=args.output.open('x')
        reader=ReadOnlyParameters(args.port,args.motor,args.baud)
        result=collect(reader,args.motor)
        text=json.dumps(result,indent=2,allow_nan=False)
        if stream:stream.write(text+'\n')
        print(text)
        if not result['complete']:p.exit(1,'Incomplete read; see read_errors. No settings changed.\n')
    except (OSError,RuntimeError,ValueError) as error:
        p.exit(1,f'Error: {error}. No motor settings were written.\n')
    finally:
        if reader:reader.close()
        if stream:stream.close()


if __name__=='__main__':main()
