#!/usr/bin/env python3
import argparse
import math
import signal
import struct
import sys
import time
import can

HOST_ID = 0xFD

TYPE_GET_ID = 0x00
TYPE_ENABLE = 0x03
TYPE_STOP = 0x04
TYPE_SET_ZERO = 0x06
TYPE_WRITE_PARAM = 0x12

PARAM_RUN_MODE = 0x7005
PARAM_SPD_REF = 0x700A
PARAM_LOC_REF = 0x7016
PARAM_LIMIT_CUR = 0x7018
PARAM_ACC_RAD = 0x7022
PARAM_VEL_MAX = 0x7024
PARAM_ACC_SET = 0x7025

RUN_MODE_POSITION_PP = 1
RUN_MODE_VELOCITY = 2

def ext_id(comm_type, motor_id, data_area2=HOST_ID):
    return ((comm_type & 0x1F) << 24) | ((data_area2 & 0xFFFF) << 8) | (motor_id & 0xFF)

def send_ext(bus, arbitration_id, data, label=""):
    if len(data) != 8:
        raise ValueError("Payload must be 8 bytes")
    msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
    bus.send(msg, timeout=0.2)
    print(f"TX {label:24s} ID={arbitration_id:08X} DATA={data.hex(' ').upper()}")

def send_type(bus, comm_type, motor_id, data, label):
    send_ext(bus, ext_id(comm_type, motor_id), data, label)

def get_device_id(bus, motor_id):
    send_type(bus, TYPE_GET_ID, motor_id, bytes(8), f"M{motor_id} GET_ID")

def stop_motor(bus, motor_id):
    send_type(bus, TYPE_STOP, motor_id, bytes(8), f"M{motor_id} STOP")

def clear_fault(bus, motor_id):
    send_type(bus, TYPE_STOP, motor_id, bytes([1,0,0,0,0,0,0,0]), f"M{motor_id} CLEAR_FAULT")

def enable_motor(bus, motor_id):
    send_type(bus, TYPE_ENABLE, motor_id, bytes(8), f"M{motor_id} ENABLE")

def set_zero_position(bus, motor_id):
    send_type(bus, TYPE_SET_ZERO, motor_id, bytes([1,0,0,0,0,0,0,0]), f"M{motor_id} SET_ZERO")

def write_u8(bus, motor_id, index, value, label):
    data = index.to_bytes(2, "little") + b"\x00\x00" + bytes([value & 0xFF,0,0,0])
    send_type(bus, TYPE_WRITE_PARAM, motor_id, data, label)

def write_float(bus, motor_id, index, value, label):
    data = index.to_bytes(2, "little") + b"\x00\x00" + struct.pack("<f", float(value))
    send_type(bus, TYPE_WRITE_PARAM, motor_id, data, label)

def set_current_limit(bus, motor_id, current):
    write_float(bus, motor_id, PARAM_LIMIT_CUR, current, f"M{motor_id} CUR_LIMIT={current:.2f}A")

def set_velocity_mode(bus, motor_id):
    write_u8(bus, motor_id, PARAM_RUN_MODE, RUN_MODE_VELOCITY, f"M{motor_id} RUN_MODE=VELOCITY")

def set_velocity_acceleration(bus, motor_id, acc):
    write_float(bus, motor_id, PARAM_ACC_RAD, acc, f"M{motor_id} ACC={acc:.2f}")

def set_speed(bus, motor_id, speed):
    write_float(bus, motor_id, PARAM_SPD_REF, speed, f"M{motor_id} SPD={speed:.3f}")

def set_position_pp_mode(bus, motor_id):
    write_u8(bus, motor_id, PARAM_RUN_MODE, RUN_MODE_POSITION_PP, f"M{motor_id} RUN_MODE=PP")

def set_pp_velocity(bus, motor_id, velocity):
    write_float(bus, motor_id, PARAM_VEL_MAX, velocity, f"M{motor_id} PP_VEL={velocity:.3f}")

def set_pp_acceleration(bus, motor_id, acc):
    write_float(bus, motor_id, PARAM_ACC_SET, acc, f"M{motor_id} PP_ACC={acc:.3f}")

def set_target_angle_deg(bus, motor_id, degrees):
    write_float(bus, motor_id, PARAM_LOC_REF, math.radians(degrees), f"M{motor_id} POS={degrees:.2f}deg")

def receive_for(bus, seconds):
    end = time.time() + seconds
    while time.time() < end:
        msg = bus.recv(timeout=0.05)
        if msg is None or not msg.is_extended_id:
            continue
        comm_type = (msg.arbitration_id >> 24) & 0x1F
        if comm_type == 2:
            motor_id = (msg.arbitration_id >> 8) & 0xFF
            print(f"RX FEEDBACK M{motor_id}: ID={msg.arbitration_id:08X} DATA={bytes(msg.data).hex(' ').upper()}")
        else:
            print(f"RX ID={msg.arbitration_id:08X} DATA={bytes(msg.data).hex(' ').upper()}")

def safe_stop(bus, motor_ids):
    print("\n--- Safe stop ---")
    for motor_id in motor_ids:
        try:
            set_speed(bus, motor_id, 0.0)
            time.sleep(0.03)
        except Exception:
            pass
        try:
            stop_motor(bus, motor_id)
            time.sleep(0.03)
        except Exception as e:
            print(f"M{motor_id} stop failed: {e}")

def init_velocity(bus, motor_id, current, acc):
    stop_motor(bus, motor_id); time.sleep(0.1)
    set_velocity_mode(bus, motor_id); time.sleep(0.1)
    set_current_limit(bus, motor_id, current); time.sleep(0.1)
    set_velocity_acceleration(bus, motor_id, acc); time.sleep(0.1)
    set_speed(bus, motor_id, 0.0); time.sleep(0.1)
    enable_motor(bus, motor_id); time.sleep(0.2)

def init_pp(bus, motor_id, current, pp_speed, acc):
    stop_motor(bus, motor_id); time.sleep(0.1)
    set_position_pp_mode(bus, motor_id); time.sleep(0.1)
    set_current_limit(bus, motor_id, current); time.sleep(0.1)
    set_pp_velocity(bus, motor_id, pp_speed); time.sleep(0.1)
    set_pp_acceleration(bus, motor_id, acc); time.sleep(0.1)
    enable_motor(bus, motor_id); time.sleep(0.2)

def main():
    p = argparse.ArgumentParser(description="RobStride RS06 integrated controller")
    p.add_argument("--motor", type=int, nargs="+", required=True)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--zero", action="store_true")
    mode.add_argument("--speed", type=float)
    mode.add_argument("--angle", type=float)
    p.add_argument("--duration", type=float, default=3.0)
    p.add_argument("--wait", type=float, default=5.0)
    p.add_argument("--current", type=float, default=2.0)
    p.add_argument("--acceleration", type=float, default=1.0)
    p.add_argument("--pp-speed", type=float, default=0.5)
    p.add_argument("--channel", default="can0")
    p.add_argument("--clear-fault", action="store_true")
    args = p.parse_args()

    bus = can.interface.Bus(channel=args.channel, interface="socketcan")

    def on_sigint(signum, frame):
        safe_stop(bus, args.motor)
        bus.shutdown()
        sys.exit(130)
    signal.signal(signal.SIGINT, on_sigint)

    try:
        if args.check:
            for m in args.motor:
                get_device_id(bus, m)
                time.sleep(0.1)
            receive_for(bus, 1.0)
            return

        if args.clear_fault:
            for m in args.motor:
                clear_fault(bus, m)
                time.sleep(0.1)

        if args.zero:
            print("WARNING: current position becomes 0 degrees")
            for m in args.motor:
                stop_motor(bus, m); time.sleep(0.1)
                set_zero_position(bus, m); time.sleep(0.2)
            receive_for(bus, 0.5)
            return

        if args.speed is not None:
            for m in args.motor:
                init_velocity(bus, m, args.current, args.acceleration)
            for m in args.motor:
                set_speed(bus, m, args.speed)
            receive_for(bus, args.duration)
            safe_stop(bus, args.motor)
            return

        if args.angle is not None:
            for m in args.motor:
                init_pp(bus, m, args.current, args.pp_speed, args.acceleration)
            for m in args.motor:
                set_target_angle_deg(bus, m, args.angle)
            receive_for(bus, args.wait)
            safe_stop(bus, args.motor)
            return
    finally:
        bus.shutdown()

if __name__ == "__main__":
    main()
