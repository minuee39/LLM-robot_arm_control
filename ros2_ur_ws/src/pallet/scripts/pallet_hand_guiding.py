#!/usr/bin/env python3
"""Offline gravity preview or explicitly commissioned SocketCAN current control."""
import argparse
import json
from pathlib import Path
import struct
import time
import yaml
from hand_guiding_core import GravityModel, command
from robstride_trajectory_core import RobStrideBus, read_payload


class FeedbackBus(RobStrideBus):
    def check_fault(self, msg, ids):
        if msg.is_error_frame:
            raise RuntimeError('CAN error frame')
        mid = (msg.arbitration_id >> 8) & 255
        typ = (msg.arbitration_id >> 24) & 31
        if msg.is_extended_id and mid in ids and (typ == 21 or (typ in (2,24) and ((msg.arbitration_id >> 16) & 63))):
            raise RuntimeError('Motor fault: '+str(mid))

    def verify_parameter(self, mid, index, expected, timeout, byte=False):
        self._send(17, mid, read_payload(index))
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            msg = self._bus.recv(timeout=max(0, deadline-time.monotonic()))
            if msg is None:
                break
            self.check_fault(msg, [mid])
            if msg.is_extended_id and (msg.arbitration_id >> 24) & 31 == 17 and (msg.arbitration_id >> 8) & 255 == mid and msg.arbitration_id & 255 == self.host_id and len(msg.data) == 8 and bytes(msg.data[:2]) == index.to_bytes(2, 'little'):
                actual = msg.data[4] if byte else struct.unpack('<f', msg.data[4:8])[0]
                if actual != expected:
                    raise RuntimeError(f'Parameter verification failed: motor {mid}, index {index:x}')
                return
        raise RuntimeError('Parameter readback timeout')

    def positions(self, ids, timeout):
        # Discard queued old samples; only accept replies to this polling cycle.
        for _ in range(2048):
            queued = self._bus.recv(timeout=0)
            if queued is None:
                break
            self.check_fault(queued, ids)
        else:
            raise RuntimeError('CAN receive backlog')
        for mid in ids:
            self._send(17, mid, read_payload(0x7019))
        result = {}
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline and len(result) < len(ids):
            msg = self._bus.recv(timeout=max(0, deadline-time.monotonic()))
            if msg is None:
                continue
            self.check_fault(msg, ids)
            if not msg.is_extended_id:
                continue
            typ = (msg.arbitration_id >> 24) & 31
            mid = (msg.arbitration_id >> 8) & 255
            if mid not in ids:
                continue
            if typ == 21 or (typ in (2,24) and ((msg.arbitration_id >> 16) & 63)):
                raise RuntimeError('Motor fault: '+str(mid))
            if typ == 17 and (msg.arbitration_id & 255) == self.host_id and len(msg.data) == 8 and bytes(msg.data[:2]) == b'\x19\x70':
                result[mid] = struct.unpack('<f', msg.data[4:8])[0]
        if set(result) != set(ids):
            raise RuntimeError('Feedback timeout: '+str(set(ids)-set(result)))
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    base = Path(__file__).resolve().parents[1]
    if not (base/'urdf').is_dir():
        from ament_index_python.packages import get_package_share_directory
        base = Path(get_package_share_directory('pallet'))
    parser.add_argument('--config', default=str(base/'config/hand_guiding.yaml'))
    parser.add_argument('--urdf', default=str(base/'urdf/pallet_with_gripper.urdf'))
    parser.add_argument('--positions', nargs=7, type=float, default=[0.0]*7, help='Offline J1..J6,g_r angles in rad')
    parser.add_argument('--live', action='store_true')
    args = parser.parse_args()
    if args.live:
        parser.error('Full-arm live control is unavailable: serial transport, session watchdog setup and measured torque calibration are not commissioned. Use read-only gravity preview or bounded single-axis commissioning.')
    model = GravityModel(args.urdf)
    cfg = yaml.safe_load(Path(args.config).read_text())
    if not args.live:
        q = dict(zip(['J1','J2','J3','J4','J5','J6','g_r'], args.positions))
        tau, potential = model.evaluate(q, cfg['gravity_root_m_s2'])
        print(json.dumps({'mode':'OFFLINE: no CAN access', 'joint_rad':q, 'gravity_nm':tau, 'potential_j':potential}, indent=2))
        return
    if cfg.get('commissioned') is not True:
        raise ValueError('Commissioning required: mapping, zero, direction, ratio, current/torque limits, gravity, gripper homing')
    if cfg.get('motor_watchdog_verified') is not True or cfg.get('gripper_homed') is not True:
        raise ValueError('Verify motor-side communication watchdog and home gripper each session before live use')
    joints = cfg['joints']
    if set(joints) != set(model.names):
        raise ValueError('Configuration must cover all six arm joints and g_r')
    ids = [c['motor_id'] for c in joints.values()]
    if len(set(ids)) != 7 or any(type(i) is not int or not 1 <= i <= 127 for i in ids):
        raise ValueError('Seven unique motor IDs required')
    period, timeout = float(cfg['period_s']), float(cfg['feedback_timeout_s'])
    if not 0 < timeout <= period <= 0.1:
        raise ValueError('Require 0 < feedback_timeout_s <= period_s <= 0.1')
    bus = FeedbackBus(cfg['interface'], cfg['host_id'])
    try:
        previous = bus.positions(ids, timeout)
        # Validate calibration and initial position before any enable command.
        command(model, joints, previous, dict.fromkeys(joints, 0), cfg['gravity_root_m_s2'])
        for mid in ids:
            bus.stop(mid)
            bus.write_u8(mid, 0x7005, 3)  # current mode
            bus.write_float(mid, 0x7006, 0.0)
            bus.verify_parameter(mid, 0x7005, 3, timeout, byte=True)
            bus.verify_parameter(mid, 0x7006, 0.0, timeout)
            bus.enable(mid)
        started = last = time.monotonic()
        while True:
            tick = time.monotonic()
            positions = bus.positions(ids, timeout)
            now = time.monotonic()
            dt = now-last
            if dt <= 0 or dt > 2*period:
                raise RuntimeError('Control deadline missed')
            velocities = {n:c['direction']*(positions[c['motor_id']]-previous[c['motor_id']])/(dt*c['ratio']) for n,c in joints.items()}
            _, _, currents = command(model, joints, positions, velocities, cfg['gravity_root_m_s2'])
            ramp = min(1.0, (now-started)/2.0)
            for mid, current in currents.items():
                bus.write_float(mid, 0x7006, ramp*current)
            previous, last = positions, now
            remaining = period-(time.monotonic()-tick)
            if remaining <= 0:
                raise RuntimeError('Control cycle overrun')
            time.sleep(remaining)
    finally:
        # Best effort for every motor, even when one stop transmission fails.
        for mid in ids:
            try:
                bus.stop(mid)
            except Exception as exc:
                print(f'STOP FAILED motor {mid}: {exc}')
        bus.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
