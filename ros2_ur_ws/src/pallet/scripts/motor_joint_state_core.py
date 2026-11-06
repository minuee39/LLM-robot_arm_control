"""Read-only motor feedback and calibrated URDF joint-state conversion."""
import math
from pathlib import Path
import struct
import time
import xml.etree.ElementTree as ET
import yaml

NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'g_r']


class MotorJointMapping:
    def __init__(self, config_path, urdf_path, include_gripper=False):
        cfg = yaml.safe_load(Path(config_path).read_text())
        self.names = NAMES if include_gripper else NAMES[:6]
        self.joints = cfg['joints']
        self.interface = cfg['interface']
        self.host_id = cfg['host_id']
        if type(self.host_id) is not int or not 0 <= self.host_id <= 255:
            raise ValueError('Invalid host_id')
        robot = ET.parse(urdf_path).getroot()
        self.limits = {}
        ids = []
        for name in self.names:
            c = self.joints[name]
            values = [c.get(k) for k in ('direction','zero_rad','ratio')]
            if any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
                raise ValueError(f'{name}: direction, zero_rad, ratio calibration required')
            if c['direction'] not in (-1,1) or c['ratio'] <= 0:
                raise ValueError(f'{name}: invalid direction or ratio')
            mid = c['motor_id']
            if type(mid) is not int or not 1 <= mid <= 127:
                raise ValueError(f'{name}: invalid motor ID')
            ids.append(mid)
            limit = robot.find(f"joint[@name='{name}']/limit")
            self.limits[name] = (float(limit.get('lower')),float(limit.get('upper')))
        if len(set(ids)) != len(ids):
            raise ValueError('Duplicate motor IDs')
        self.ids = ids

    def convert(self, positions):
        if set(positions) != set(self.ids):
            raise ValueError('Require one complete motor sample for all selected joints')
        result = []
        for name in self.names:
            c = self.joints[name]
            raw = positions[c['motor_id']]
            if not math.isfinite(raw):
                raise ValueError(f'{name}: non-finite motor angle')
            q = c['direction']*(raw-c['zero_rad'])/c['ratio']
            lo,hi = self.limits[name]
            if not lo-1e-6 <= q <= hi+1e-6:
                raise ValueError(f'{name}: {q:.4f} rad outside URDF limits; check zero, direction, turns')
            result.append(q)
        # Explicit display-only open gripper when its calibration is not selected.
        if len(result) == 6:
            result.append(0.0)
        return result


class MissingFeedbackError(RuntimeError):
    def __init__(self, ids, received):
        self.received = dict(received)
        self.missing = sorted(set(ids)-set(received))
        super().__init__(f'Missing feedback: {self.missing}')


class ReadOnlyMotorBus:
    """Only sends parameter-read requests; no enable, stop, zero, or write API."""
    def __init__(self, interface, host_id):
        import can
        self.can = can
        self.bus = can.interface.Bus(channel=interface,interface='socketcan')
        self.host_id = host_id

    def read(self, ids, timeout):
        for _ in range(2048):
            if self.bus.recv(timeout=0) is None:
                break
        else:
            raise RuntimeError('CAN backlog')
        for mid in ids:
            self.bus.send(self.can.Message(arbitration_id=(17<<24)|(self.host_id<<8)|mid,is_extended_id=True,data=b'\x19\x70'+bytes(6)),timeout=timeout)
        deadline = time.monotonic()+timeout
        result = {}
        while time.monotonic() < deadline and len(result) < len(ids):
            msg = self.bus.recv(timeout=max(0,deadline-time.monotonic()))
            if msg is None:
                continue
            if msg.is_error_frame:
                raise RuntimeError('CAN error frame')
            if not msg.is_extended_id or msg.is_remote_frame:
                continue
            arb = msg.arbitration_id
            mid = (arb>>8)&255
            if (arb>>24)&31 != 17 or arb&255 != self.host_id or mid not in ids:
                continue
            if len(msg.data) == 8 and bytes(msg.data[:2]) == b'\x19\x70':
                status=(arb>>16)&255
                if status:raise RuntimeError(f'Parameter read rejected: 0x7019, status={status}')
                value = struct.unpack('<f',msg.data[4:8])[0]
                if not math.isfinite(value):
                    raise ValueError('Non-finite feedback')
                result[mid] = value
        if set(result) != set(ids):
            raise MissingFeedbackError(ids,result)
        return result

    def close(self):
        self.bus.shutdown()
