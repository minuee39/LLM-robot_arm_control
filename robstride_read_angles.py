#!/usr/bin/env python3
"""Read current mechanical angles from RobStride motors over SocketCAN."""

import argparse
import json
import math
import struct
import sys
import time
from pathlib import Path

import can


HOST_ID = 0xFD
TYPE_READ_PARAM = 0x11
PARAM_MECH_POS = 0x7019


def request_angle(bus, motor_id):
    arbitration_id = (
        (TYPE_READ_PARAM << 24)
        | (HOST_ID << 8)
        | motor_id
    )
    payload = PARAM_MECH_POS.to_bytes(2, "little") + bytes(6)
    bus.send(
        can.Message(
            arbitration_id=arbitration_id,
            data=payload,
            is_extended_id=True,
        ),
        timeout=0.2,
    )


def receive_angles(bus, expected_motor_ids, timeout):
    deadline = time.monotonic() + timeout
    parameter_index = PARAM_MECH_POS.to_bytes(2, "little")
    pending_ids = set(expected_motor_ids)
    angles = {}

    while pending_ids and time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        message = bus.recv(timeout=min(0.1, remaining))
        if message is None or not message.is_extended_id:
            continue

        communication_type = (message.arbitration_id >> 24) & 0x1F
        motor_id = (message.arbitration_id >> 8) & 0xFF
        data = bytes(message.data)

        if (
            communication_type == TYPE_READ_PARAM
            and motor_id in pending_ids
            and len(data) == 8
            and data[:2] == parameter_index
        ):
            angles[motor_id] = struct.unpack("<f", data[4:8])[0]
            pending_ids.remove(motor_id)

    return angles


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read RobStride motor angles without enabling or moving them."
    )
    parser.add_argument(
        "--motor",
        type=int,
        nargs="+",
        default=list(range(1, 8)),
        help="motor IDs to read (default: 1 2 3 4 5 6 7)",
    )
    parser.add_argument("--relative-to", help="JSON motor_zero_rad file; subtract software reference for display only")
    parser.add_argument("--transport", choices=["socketcan","serial"], default="socketcan")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--retries", type=int, default=2, help="Extra serial read attempts for missing motors (0..5)")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="refresh rate in Hz (default: 10)",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="read each motor once instead of continuously",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    references = {}
    if args.relative_to:
        references = {int(k):float(v) for k,v in json.loads(Path(args.relative_to).read_text())['motor_zero_rad'].items()}
        if any(mid not in references for mid in args.motor):
            print('Reference missing for requested motor; select calibrated IDs only',file=sys.stderr)
            return 2
        if not all(math.isfinite(v) for v in references.values()):
            raise ValueError('Non-finite zero reference')
        if args.transport != 'serial':
            print('--relative-to currently supports serial transport only',file=sys.stderr)
            return 2

    invalid_ids = [motor_id for motor_id in args.motor if not 1 <= motor_id <= 255]
    if invalid_ids:
        print(f"Invalid motor ID(s): {invalid_ids}; expected 1..255", file=sys.stderr)
        return 2
    if not 0 <= args.retries <= 5 or not math.isfinite(args.timeout) or args.timeout <= 0:
        print("Require 0..5 retries and a finite positive timeout",file=sys.stderr)
        return 2
    if args.rate <= 0:
        print("--rate must be greater than zero", file=sys.stderr)
        return 2

    if args.transport == "serial":
        sys.path.insert(0,str(Path(__file__).resolve().parent/'ros2_ur_ws/src/pallet/scripts'))
        from robstride_serial_readonly import ReadOnlySerialMotorBus
        from motor_joint_state_core import MissingFeedbackError
        reader = None
        try:
            reader = ReadOnlySerialMotorBus(args.port, HOST_ID, args.baud)
            while True:
                try:
                    values = reader.read(args.motor, args.timeout)
                except MissingFeedbackError as exc:
                    values = exc.received
                    print(f"Initial reply IDs: {sorted(values)}; missing: {exc.missing}",flush=True)
                retried = set()
                for attempt in range(args.retries):
                    missing = sorted(set(args.motor)-set(values))
                    if not missing:
                        break
                    for mid in missing:
                        print(f"Retry {attempt+1}/{args.retries}: ID {mid} (individual request)",flush=True)
                        try:
                            values.update(reader.read([mid], args.timeout))
                            retried.add(mid)
                        except MissingFeedbackError:
                            pass
                for mid, angle in sorted(values.items()):
                    angle -= references.get(mid,0.0)
                    suffix = " [received on retry]" if mid in retried else ""
                    if references: suffix += " [relative to software zero]"
                    print(f"ID {mid}: {angle:.6f} rad ({math.degrees(angle):.3f} deg){suffix}",flush=True)
                missing = sorted(set(args.motor)-set(values))
                for mid in missing:
                    print(f"ID {mid}: NO RESPONSE after {1+args.retries} attempts",flush=True)
                if args.once:
                    return 1 if missing else 0
                time.sleep(1/args.rate)
        except KeyboardInterrupt:
            return 0
        except Exception as exc:
            print(f"Serial read failed: {exc}",file=sys.stderr)
            return 1
        finally:
            if reader is not None:
                reader.close()

    try:
        bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    except Exception as error:
        print(f"Failed to open {args.channel}: {error}", file=sys.stderr)
        return 1

    failures = 0
    interval = 1.0 / args.rate
    try:
        while True:
            cycle_started = time.monotonic()
            readings = []
            requested_ids = []

            for motor_id in args.motor:
                try:
                    request_angle(bus, motor_id)
                except can.CanError as error:
                    readings.append(f"ID {motor_id}: CAN error ({error})")
                    failures += 1
                    continue
                requested_ids.append(motor_id)

            receive_timeout = args.timeout if args.once else min(args.timeout, interval)
            angles = receive_angles(bus, requested_ids, receive_timeout)

            for motor_id in requested_ids:
                angle_rad = angles.get(motor_id)
                if angle_rad is None:
                    readings.append(f"ID {motor_id}: no response")
                    failures += 1
                    continue

                angle_deg = math.degrees(angle_rad)
                readings.append(
                    f"ID {motor_id}: {angle_deg: .4f} deg ({angle_rad: 6.2f} rad)"
                )

            if args.once:
                for reading in readings:
                    print(reading)
                break

            wall_time = time.time()
            timestamp = (
                time.strftime("%H:%M:%S", time.localtime(wall_time))
                + f".{int(wall_time % 1 * 1000):03d}"
            )
            print(f"\r{timestamp} | " + " | ".join(readings), end="", flush=True)

            elapsed = time.monotonic() - cycle_started
            time.sleep(max(0.0, interval - elapsed))
    except KeyboardInterrupt:
        if not args.once:
            print("\nStopped.")
    finally:
        bus.shutdown()

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
