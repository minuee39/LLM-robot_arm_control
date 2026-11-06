#!/usr/bin/env python3
"""Read-only six-arm-axis gravity torque diagnostic. Never enables motors."""
import argparse
import json
import math
import sys
from pathlib import Path
import time
import yaml
from hand_guiding_core import GravityModel
from motor_joint_state_core import MotorJointMapping, ReadOnlyMotorBus, NAMES
from robstride_serial_readonly import ReadOnlySerialMotorBus


def evaluate_preview(model,mapping,positions,joints,gravity):
    q=mapping.convert(positions)
    torque,potential=model.evaluate(dict(zip(NAMES,q)),gravity)
    rows={}
    for name in NAMES[:6]:
        c=joints[name]
        tau=torque[name]
        motor_tau=c['direction']*tau/c['ratio']
        kt=c.get('output_nm_per_a')
        valid=type(kt) in (int,float) and math.isfinite(kt) and kt>0
        current=motor_tau/kt if valid else None
        issues=[]
        if not valid:issues.append('Iq-to-output-torque conversion unconfirmed')
        for key,value in [('torque_limit_nm',abs(tau)),('current_limit_a',None if current is None else abs(current))]:
            limit=c.get(key)
            if type(limit) not in (int,float) or not math.isfinite(limit) or limit<=0:
                issues.append(key+' unconfirmed')
            elif value is not None and value>limit:
                issues.append(key+' EXCEEDED')
        rows[name]={'motor_id':c['motor_id'],'joint_rad':q[NAMES.index(name)],'gravity_joint_nm':tau,'motor_output_nm':motor_tau,'predicted_iq_a':current,'issues':issues}
    return {'mode':'READ ONLY: no drive commands','gripper_assumption':'g_r=0 (open reference); no payload; actual gripper pose is not read','potential_j':potential,'arm':rows}


def main():
    root=Path(__file__).resolve().parents[1]
    if not (root/'urdf').is_dir():
        from ament_index_python.packages import get_package_share_directory
        root=Path(get_package_share_directory('pallet'))
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default=str(root/'config/hand_guiding.yaml'))
    p.add_argument('--urdf',default=str(root/'urdf/pallet_with_gripper.urdf'))
    p.add_argument('--transport',choices=['serial','socketcan'],default='serial')
    p.add_argument('--port',default='/dev/ttyUSB0')
    p.add_argument('--from-joint-states',action='store_true',help='Subscribe to existing RViz motor bridge; no USB access')
    p.add_argument('--once',action='store_true')
    p.add_argument('--wait-timeout',type=float,default=10.,help='Startup wait for joint_states (seconds)')
    p.add_argument('--rate',type=float,default=2.)
    p.add_argument('--motor-positions',nargs=6,type=float,help='Offline raw motor radians for IDs 1..6; no device access')
    cli_args=sys.argv[1:]
    if '--ros-args' in cli_args:
        from rclpy.utilities import remove_ros_args
        cli_args=remove_ros_args(args=sys.argv)[1:]
    args=p.parse_args(cli_args)
    if not math.isfinite(args.wait_timeout) or args.wait_timeout<=0:p.error('wait-timeout must be positive')
    if not math.isfinite(args.rate) or not 0<args.rate<=10:p.error('rate must be 0..10 Hz')
    cfg=yaml.safe_load(Path(args.config).read_text())
    mapping=MotorJointMapping(args.config,args.urdf,False)
    model=GravityModel(args.urdf)
    if args.from_joint_states:
        if args.motor_positions is not None:p.error('Choose joint_states or motor positions')
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        rclpy.init(args=sys.argv)
        node=Node('pallet_gravity_preview')
        done=False
        last_print=0.
        last_received=time.monotonic()
        received_any=False
        def receive(msg):
            nonlocal done,last_print,last_received,received_any
            if len(msg.name)!=len(msg.position) or len(set(msg.name))!=len(msg.name):return
            q=dict(zip(msg.name,msg.position))
            if any(n not in q for n in NAMES[:6]):return
            last_received=time.monotonic()
            received_any=True
            if time.monotonic()-last_print<1/args.rate:return
            raw={mapping.joints[n]['motor_id']:mapping.joints[n]['zero_rad']+mapping.joints[n]['direction']*mapping.joints[n]['ratio']*q[n] for n in NAMES[:6]}
            result=evaluate_preview(model,mapping,raw,cfg['joints'],cfg['gravity_root_m_s2'])
            result['input']='existing /joint_states; verify that motor bridge is its only publisher'
            print(json.dumps(result,indent=2),flush=True)
            last_print=time.monotonic()
            if args.once:done=True
        sub=node.create_subscription(JointState,'/joint_states',receive,10)
        try:
            while rclpy.ok() and not done:
                rclpy.spin_once(node,timeout_sec=.1)
                timeout=3.0 if received_any else args.wait_timeout
                if time.monotonic()-last_received>timeout:
                    import os
                    raise RuntimeError(f'No joint_states for {timeout:g}s in ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID","0")}. Start display_motor_feedback.launch.py with gravity_preview:=true; slider/closed RViz is not live motor feedback.')
        finally:
            node.destroy_node();rclpy.shutdown()
        return
    bus=None
    try:
        if args.motor_positions is None:
            bus=ReadOnlySerialMotorBus(args.port,mapping.host_id) if args.transport=='serial' else ReadOnlyMotorBus(mapping.interface,mapping.host_id)
        while True:
            started=time.monotonic()
            raw=dict(zip(mapping.ids,args.motor_positions)) if args.motor_positions is not None else bus.read(mapping.ids,.3)
            result=evaluate_preview(model,mapping,raw,cfg['joints'],cfg['gravity_root_m_s2'])
            print(json.dumps(result,indent=2),flush=True)
            if args.once or args.motor_positions is not None:break
            time.sleep(max(0,1/args.rate-(time.monotonic()-started)))
    finally:
        if bus:bus.close()


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:pass
