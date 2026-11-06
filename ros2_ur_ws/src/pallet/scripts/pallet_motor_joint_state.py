#!/usr/bin/env python3
"""Publish calibrated motor feedback to RViz, without actuating any motor."""
from pathlib import Path
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from ament_index_python.packages import get_package_share_directory
from robstride_serial_readonly import ReadOnlySerialMotorBus
from motor_joint_state_core import MotorJointMapping, ReadOnlyMotorBus, NAMES


class MotorStateNode(Node):
    def __init__(self):
        super().__init__('pallet_motor_joint_state')
        root = Path(get_package_share_directory('pallet'))
        for name,value in dict(transport='socketcan',serial_port='/dev/ttyUSB0',serial_baud=921600,mode='manual',config_file=str(root/'config/hand_guiding.yaml'),urdf_file=str(root/'urdf/pallet_with_gripper.urdf'),include_gripper=False,rate_hz=10.0,timeout_s=0.08,stale_timeout_s=0.5).items():
            self.declare_parameter(name,value)
        self.mode = self.get_parameter('mode').value
        if self.mode not in ('manual','can','joint_preview'):
            raise ValueError('mode must be manual, can, or joint_preview')
        self.rate = float(self.get_parameter('rate_hz').value)
        self.timeout = float(self.get_parameter('timeout_s').value)
        self.stale = float(self.get_parameter('stale_timeout_s').value)
        if not all(math.isfinite(v) for v in (self.rate,self.timeout,self.stale)) or not 0 < self.rate <= 100 or not 0 < self.timeout <= 1/self.rate or self.stale <= 1/self.rate:
            raise ValueError('Invalid polling/timeout settings')
        self.mapping = None
        self.bus = None
        self.last_sample = 0.0
        self.last_warn = 0.0
        self.pub = self.create_publisher(JointState,'/joint_states',10)
        self.status = self.create_publisher(String,'/pallet_motor_state/status',10)
        if self.mode != 'joint_preview':
            self.mapping = MotorJointMapping(self.get_parameter('config_file').value,self.get_parameter('urdf_file').value,self.get_parameter('include_gripper').value)
        if self.mode == 'can':
            transport = self.get_parameter('transport').value
            if transport == 'serial':
                self.bus = ReadOnlySerialMotorBus(self.get_parameter('serial_port').value,self.mapping.host_id,self.get_parameter('serial_baud').value)
            elif transport == 'socketcan':
                self.bus = ReadOnlyMotorBus(self.mapping.interface,self.mapping.host_id)
            else:
                raise ValueError('transport must be serial or socketcan')
            self.timer = self.create_timer(1/self.rate,self.poll)
        else:
            topic = '/pallet_motor_positions' if self.mode == 'manual' else '/pallet_preview_positions'
            self.subscription = self.create_subscription(Float64MultiArray,topic,self.receive,10)
            self.get_logger().info(f'Input {topic}; order '+(', '.join(map(str,self.mapping.ids)) if self.mapping else ', '.join(NAMES)))
        self.watchdog = self.create_timer(0.2,self.check_stale)
        self.get_logger().info(f'READ-ONLY mode={self.mode}; include_gripper={self.get_parameter("include_gripper").value}. No drive commands.')

    def report(self,text,error=False):
        self.status.publish(String(data=text))
        if error and time.monotonic()-self.last_warn > 1:
            self.get_logger().warn(text); self.last_warn=time.monotonic()

    def publish(self,q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = NAMES+['g_l']
        msg.position = list(q)+[-q[6]]
        self.pub.publish(msg)
        self.last_sample = time.monotonic()
        self.report('OK: URDF joint preview' if self.mode == 'joint_preview' else ('OK' if self.get_parameter('include_gripper').value else 'OK; gripper held at URDF zero for display'))

    def receive(self,msg):
        try:
            values = list(msg.data)
            if self.mapping:
                if len(values) != len(self.mapping.ids):
                    raise ValueError(f'Expected {len(self.mapping.ids)} motor angles in rad')
                q = self.mapping.convert(dict(zip(self.mapping.ids,values)))
            else:
                if len(values)!=7 or not all(math.isfinite(v) for v in values):
                    raise ValueError('Expected seven finite URDF joint angles in rad')
                q = values
            self.publish(q)
        except (ValueError,RuntimeError) as exc:
            self.report(str(exc),True)

    def poll(self):
        try:
            self.publish(self.mapping.convert(self.bus.read(self.mapping.ids,self.timeout)))
        except Exception as exc:
            self.report('Feedback rejected: '+str(exc),True)

    def check_stale(self):
        if time.monotonic()-self.last_sample > self.stale:
            self.report('STALE: no fresh valid motor sample; joint states are not republished',True)


def main():
    rclpy.init()
    node = None
    try:
        node = MotorStateNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            if node.bus: node.bus.close()
            node.destroy_node()
        # SIGINT may already have shut the context down.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
