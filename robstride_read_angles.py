#!/usr/bin/env python3
"""Read current mechanical angles from RobStride motors over SocketCAN."""

import argparse
import math
import struct
import sys
import time

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
        default=list(range(1, 7)),
        help="motor IDs to read (default: 1 2 3 4 5 6)",
    )
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

    invalid_ids = [motor_id for motor_id in args.motor if not 1 <= motor_id <= 255]
    if invalid_ids:
        print(f"Invalid motor ID(s): {invalid_ids}; expected 1..255", file=sys.stderr)
        return 2
    if args.rate <= 0:
        print("--rate must be greater than zero", file=sys.stderr)
        return 2

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
