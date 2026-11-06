#!/usr/bin/env python3
"""Send a raw ID7 angle to the shared bridge, without MoveIt planning."""
import argparse
import math
import time
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


def make_goal(start, target):
    if not all(math.isfinite(q) and -12 <= q <= 12 for q in (start, target)):
        raise ValueError('Raw motor angle must be finite and within [-12, 12] rad')
    # Cubic with zero endpoint velocity: vmax=1.5*d/T, amax=6*d/T^2.
    distance = abs(target-start)
    # Conservative arrival budget for the drive's 2 rad/s, 5 rad/s² ramp.
    # The bridge sends the final angle directly for a gripper-only goal.
    duration = max(1., 1.5*distance/2., math.sqrt(6*distance/5.))
    if duration > 60:
        raise ValueError('Move exceeds 60 seconds; choose an intermediate target')
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ['gripper_motor']
    first = JointTrajectoryPoint(positions=[start], velocities=[0.])
    last = JointTrajectoryPoint(positions=[target], velocities=[0.])
    ns = math.ceil(duration*1e9)
    last.time_from_start.sec, last.time_from_start.nanosec = divmod(ns, 1000000000)
    goal.trajectory.points = [first, last]
    return goal, duration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('angle', type=float, help='Absolute raw motor angle in radians, -12..12')
    args = parser.parse_args()
    make_goal(0., args.angle)  # Validate input before connecting.
    rclpy.init()
    node = rclpy.create_node('gripper_direct')
    client = ActionClient(node, FollowJointTrajectory, '/pallet_gripper_controller/follow_joint_trajectory')
    handle = None
    try:
        if not client.wait_for_server(timeout_sec=5.):
            raise RuntimeError('Gripper bridge unavailable; start serial_bridge.launch.py first')
        samples = []
        def feedback(msg):
            if 'gripper_motor' in msg.name:
                index = msg.name.index('gripper_motor')
                age = (node.get_clock().now().nanoseconds - (msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec))/1e9
                if index < len(msg.position) and 0 <= age <= .25:
                    samples.append(msg.position[index])
        subscription = node.create_subscription(JointState, '/joint_states', feedback, 10)
        deadline = time.monotonic()+5
        while not samples and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not samples:raise RuntimeError('No fresh ID7 joint feedback')
        goal, duration = make_goal(samples[-1], args.angle)
        node.get_logger().info(f'Direct ID7 target {args.angle:.3f} rad; no MoveIt collision planning')
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future)
        handle = future.result()
        if not handle.accepted:raise RuntimeError('Bridge rejected target; check bridge status')
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, future, timeout_sec=duration+20.)
        if not future.done():raise RuntimeError('Execution timed out')
        result = future.result()
        if result.status != 4 or result.result.error_code != 0:
            raise RuntimeError(result.result.error_string or f'Action status {result.status}')
        print('Target reached; bridge hold policy remains active')
        handle = None
    finally:
        if handle is not None:
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=3.)
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
