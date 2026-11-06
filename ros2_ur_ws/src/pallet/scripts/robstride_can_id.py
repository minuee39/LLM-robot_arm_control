#!/usr/bin/env python3
"""Discover RobStride motors and change their private-protocol CAN IDs.

The tool talks directly to a Linux SocketCAN interface and therefore has no
third-party Python dependency.  RobStride's private protocol uses 1 Mbit/s
classical CAN with 29-bit extended identifiers.
"""

from __future__ import annotations

import argparse
import json
import select
import socket
import struct
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Optional


CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_FRAME = struct.Struct("=IB3x8s")

COMM_GET_DEVICE_ID = 0
COMM_SET_CAN_ID = 7
DISCOVERY_REPLY_DESTINATION = 0xFE
DEFAULT_HOST_ID = 0xFD


class RobStrideError(RuntimeError):
    """Raised when discovery or an ID update cannot be completed safely."""


@dataclass(frozen=True)
class DeviceIdentity:
    motor_id: int
    uid: bytes

    @property
    def uid_hex(self) -> str:
        return self.uid.hex().upper()


def byte_id(value: str | int) -> int:
    """Parse a decimal/hexadecimal CAN node ID and enforce one-byte range."""
    parsed = int(value, 0) if isinstance(value, str) else int(value)
    if not 0 <= parsed <= 0xFF:
        raise argparse.ArgumentTypeError("ID must be in the range 0..255")
    return parsed


def motor_id(value: str | int) -> int:
    parsed = byte_id(value)
    if parsed == 0:
        raise argparse.ArgumentTypeError("motor ID must be in the range 1..255")
    return parsed


def arbitration_id(comm_type: int, data_area_2: int, destination: int) -> int:
    """Pack RobStride's [type:5][data2:16][destination:8] extended ID."""
    if not 0 <= comm_type <= 0x1F:
        raise ValueError("communication type must fit in 5 bits")
    if not 0 <= data_area_2 <= 0xFFFF:
        raise ValueError("data area 2 must fit in 16 bits")
    if not 0 <= destination <= 0xFF:
        raise ValueError("destination must fit in 8 bits")
    return (comm_type << 24) | (data_area_2 << 8) | destination


def get_device_id_arbitration_id(host_id: int, target_id: int) -> int:
    return arbitration_id(COMM_GET_DEVICE_ID, host_id, target_id)


def set_can_id_arbitration_id(host_id: int, old_id: int, new_id: int) -> int:
    data_area_2 = (new_id << 8) | host_id
    return arbitration_id(COMM_SET_CAN_ID, data_area_2, old_id)


def encode_can_frame(arbitration: int, payload: bytes = bytes(8)) -> bytes:
    if len(payload) > 8:
        raise ValueError("classical CAN payload cannot exceed 8 bytes")
    return CAN_FRAME.pack(
        arbitration | CAN_EFF_FLAG,
        len(payload),
        payload.ljust(8, b"\x00"),
    )


def decode_device_identity(frame: bytes) -> Optional[DeviceIdentity]:
    """Return an identity for a valid type-0 broadcast response."""
    if len(frame) != CAN_FRAME.size:
        return None
    raw_can_id, dlc, payload = CAN_FRAME.unpack(frame)
    if not raw_can_id & CAN_EFF_FLAG:
        return None
    if raw_can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG):
        return None

    extended_id = raw_can_id & CAN_EFF_MASK
    comm_type = (extended_id >> 24) & 0x1F
    data_area_2 = (extended_id >> 8) & 0xFFFF
    destination = extended_id & 0xFF
    if comm_type != COMM_GET_DEVICE_ID or destination != DISCOVERY_REPLY_DESTINATION:
        return None

    # In a type-0 response the current motor ID occupies bits 15..8.  The
    # upper byte of data area 2 is reserved and should be zero.
    if data_area_2 >> 8:
        return None
    return DeviceIdentity(motor_id=data_area_2 & 0xFF, uid=payload[:dlc])


class SocketCanBus:
    def __init__(self, interface: str):
        self.interface = interface
        self.socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)

    def __enter__(self) -> "SocketCanBus":
        try:
            self.socket.bind((self.interface,))
        except OSError:
            self.socket.close()
            raise
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.socket.close()

    def send(self, arbitration: int, payload: bytes = bytes(8)) -> None:
        self.socket.send(encode_can_frame(arbitration, payload))

    def receive(self, timeout: float) -> Optional[bytes]:
        readable, _, _ = select.select([self.socket], [], [], max(0.0, timeout))
        return self.socket.recv(CAN_FRAME.size) if readable else None

    def drain(self) -> None:
        while True:
            readable, _, _ = select.select([self.socket], [], [], 0.0)
            if not readable:
                return
            self.socket.recv(CAN_FRAME.size)


def wait_for_identity(
    bus: SocketCanBus, expected_id: int, timeout: float
) -> Optional[DeviceIdentity]:
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        frame = bus.receive(remaining)
        if frame is None:
            return None
        identity = decode_device_identity(frame)
        if identity is not None and identity.motor_id == expected_id:
            return identity


def query_motor(
    bus: SocketCanBus, target_id: int, host_id: int, timeout: float
) -> Optional[DeviceIdentity]:
    bus.drain()
    bus.send(get_device_id_arbitration_id(host_id, target_id))
    return wait_for_identity(bus, target_id, timeout)


def scan_motors(
    bus: SocketCanBus,
    ids: Iterable[int],
    host_id: int,
    timeout: float,
) -> list[DeviceIdentity]:
    found: list[DeviceIdentity] = []
    for target_id in ids:
        identity = query_motor(bus, target_id, host_id, timeout)
        if identity is not None:
            found.append(identity)
    return found


def change_motor_id(
    bus: SocketCanBus,
    old_id: int,
    new_id: int,
    host_id: int,
    timeout: float,
    use_uid_payload: bool = True,
) -> DeviceIdentity:
    if old_id == new_id:
        raise RobStrideError("old and new motor IDs are identical")

    old_identity = query_motor(bus, old_id, host_id, timeout)
    if old_identity is None:
        raise RobStrideError(f"motor ID {old_id} did not answer")

    collision = query_motor(bus, new_id, host_id, timeout)
    if collision is not None:
        raise RobStrideError(
            f"new motor ID {new_id} is already used by UID {collision.uid_hex}"
        )

    # Current RobStride host software returns the UID token from the preceding
    # type-0 query in the type-7 payload.  Older manuals mark these bytes as
    # unused, so --zero-payload is available for older firmware.
    payload = old_identity.uid if use_uid_payload else bytes(8)
    bus.drain()
    bus.send(set_can_id_arbitration_id(host_id, old_id, new_id), payload)

    acknowledgement = wait_for_identity(bus, new_id, timeout)
    if acknowledgement is None:
        # Some firmware does not emit the documented immediate broadcast.
        acknowledgement = query_motor(bus, new_id, host_id, timeout)
    if acknowledgement is None:
        raise RobStrideError(
            f"ID update was sent, but motor ID {new_id} did not answer verification"
        )
    if old_identity.uid and acknowledgement.uid != old_identity.uid:
        raise RobStrideError(
            "new ID answered with a different UID; another motor may be using that ID"
        )
    return acknowledgement


def add_common_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--interface", default="can0", help="SocketCAN interface (default: can0)"
    )
    parser.add_argument(
        "--host-id", type=byte_id, default=DEFAULT_HOST_ID, help="host ID (default: 0xFD)"
    )
    parser.add_argument(
        "--timeout-ms",
        type=float,
        default=20.0,
        help="reply timeout for each request (default: 20 ms)",
    )


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Find RobStride private-protocol motors and change their CAN IDs.",
        epilog=(
            "Safety: if multiple motors still share the same factory ID, connect and "
            "rename them one at a time. A type-7 update addresses every motor with "
            "the specified old ID."
        ),
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    scan_parser = subparsers.add_parser("scan", help="probe a range of motor IDs")
    add_common_arguments(scan_parser)
    scan_parser.add_argument("--start-id", type=motor_id, default=1)
    scan_parser.add_argument("--end-id", type=motor_id, default=255)
    scan_parser.add_argument("--json", action="store_true", help="print JSON output")

    set_parser = subparsers.add_parser("set-id", help="change one motor ID")
    add_common_arguments(set_parser)
    set_parser.add_argument("old_id", type=motor_id, help="current motor ID")
    set_parser.add_argument("new_id", type=motor_id, help="desired motor ID")
    set_parser.add_argument(
        "--yes", action="store_true", help="perform the update without an interactive prompt"
    )
    set_parser.add_argument(
        "--zero-payload",
        action="store_true",
        help="send zero data bytes instead of the discovered UID (older firmware fallback)",
    )
    return parser


def run(args: argparse.Namespace) -> int:
    timeout = args.timeout_ms / 1000.0
    if timeout <= 0:
        raise RobStrideError("--timeout-ms must be greater than zero")

    if args.command == "scan" and args.start_id > args.end_id:
        raise RobStrideError("--start-id must not be greater than --end-id")

    with SocketCanBus(args.interface) as bus:
        if args.command == "scan":
            found = scan_motors(
                bus, range(args.start_id, args.end_id + 1), args.host_id, timeout
            )
            if args.json:
                print(
                    json.dumps(
                        [
                            {"motor_id": item.motor_id, "uid": item.uid_hex}
                            for item in found
                        ],
                        indent=2,
                    )
                )
            elif found:
                print("motor_id  uid")
                for item in found:
                    print(f"{item.motor_id:8d}  {item.uid_hex}")
            else:
                print("No RobStride motors answered.")
            return 0 if found else 2

        if not args.yes:
            answer = input(
                f"Change RobStride motor ID {args.old_id} -> {args.new_id}? [y/N] "
            )
            if answer.strip().lower() not in {"y", "yes"}:
                print("Cancelled.")
                return 1

        identity = change_motor_id(
            bus,
            args.old_id,
            args.new_id,
            args.host_id,
            timeout,
            use_uid_payload=not args.zero_payload,
        )
        print(f"ID changed: {args.old_id} -> {identity.motor_id}, UID={identity.uid_hex}")
        return 0


def main() -> int:
    parser = build_argument_parser()
    args = parser.parse_args()
    try:
        return run(args)
    except (RobStrideError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
