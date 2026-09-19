#!/usr/bin/env python3
"""Measured-feedback MoveIt/Isaac adapter. Positions are radians."""
import math
import threading
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

ARM = [f'J{i}' for i in range(1, 7)]
JOINTS = ARM + ['J8']


class Bridge(Node):
    def __init__(self):
        super().__init__('pallet_gripper_isaac_bridge')
        config = Path(get_package_share_directory('pallet_gripper_moveit_config'))
        initial = yaml.safe_load((config / 'config/initial_positions.yaml').read_text())[
            'initial_positions'
        ]
        self.positions = {joint: float(initial[joint]) for joint in JOINTS}
        self.stamp = 0.0
        self.have_feedback = False
        self.lock = threading.Lock()
        self.busy = set()
        self.pub = self.create_publisher(JointState, '/isaac_joint_commands', 10)
        self.states = self.create_publisher(JointState, '/joint_states', 10)
        group = ReentrantCallbackGroup()
        self.create_subscription(JointState, '/isaac_joint_states', self.feedback,
                                 qos_profile_sensor_data, callback_group=group)
        # Give robot_state_publisher, MoveIt and RViz the exact same initial
        # state used by isaac_sim.py.  Stop doing this as soon as measured
        # Isaac feedback arrives; all subsequent state is measured feedback.
        self.initial_state_timer = self.create_timer(0.05, self.publish_initial_state,
                                                     callback_group=group)
        self.servers = [ActionServer(self, action, path, callback_group=group,
                       goal_callback=lambda req, k=kind: self.accept(req, k),
                       cancel_callback=lambda _: CancelResponse.ACCEPT,
                       execute_callback=lambda goal, k=kind: self.execute(goal, k))
                       for kind, action, path in [
                           ('arm', FollowJointTrajectory, '/pallet_arm_controller/follow_joint_trajectory'),
                           ('gripper', GripperCommand, '/gripper_controller/gripper_cmd')]]

    def publish_state(self, values):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name, out.position = JOINTS, [values[j] for j in JOINTS]
        self.states.publish(out)

    def publish_initial_state(self):
        with self.lock:
            if self.have_feedback:
                return
            values = dict(self.positions)
        self.publish_state(values)

    def feedback(self, msg):
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        values = dict(zip(msg.name, msg.position))
        if not all(j in values and math.isfinite(values[j]) for j in JOINTS):
            return
        with self.lock:
            self.positions, self.stamp = values, time.monotonic()
            self.have_feedback = True
        self.publish_state(values)

    def accept(self, req, kind):
        if kind == 'arm':
            names, points = req.trajectory.joint_names, req.trajectory.points
            if len(names) != 6 or set(names) != set(ARM) or not points:
                return GoalResponse.REJECT
            previous = -1.0
            for p in points:
                t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                if t < 0 or t <= previous or len(p.positions) != 6 or not all(map(math.isfinite, p.positions)):
                    return GoalResponse.REJECT
                previous = t
        elif not math.isfinite(req.command.position) or not 0 <= req.command.position <= 0.229:
            return GoalResponse.REJECT
        with self.lock:
            if kind in self.busy or time.monotonic() - self.stamp > 1:
                return GoalResponse.REJECT
            self.busy.add(kind)
        return GoalResponse.ACCEPT

    def current(self, names):
        with self.lock:
            if time.monotonic() - self.stamp > 1:
                raise RuntimeError('Isaac feedback timeout')
            return [self.positions[j] for j in names]

    def command(self, names, values):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position = list(names), list(values)
        self.pub.publish(msg)

    def execute(self, goal, kind):
        result = FollowJointTrajectory.Result() if kind == 'arm' else GripperCommand.Result()
        names = list(goal.request.trajectory.joint_names) if kind == 'arm' else ['J8']
        try:
            start = self.current(names)
            if kind == 'arm':
                points = [(p.time_from_start.sec + p.time_from_start.nanosec * 1e-9, list(p.positions))
                          for p in goal.request.trajectory.points]
            else:
                target = goal.request.command.position
                points = [(max(0.1, abs(target - start[0]) / 0.03), [target])]
            began, previous = time.monotonic(), 0.0
            for t, target in points:
                while time.monotonic() - began < t:
                    actual = self.current(names)
                    if goal.is_cancel_requested:
                        self.command(names, actual)
                        goal.canceled()
                        return result
                    ratio = min(1.0, max(0.0, (time.monotonic() - began - previous) / max(t - previous, 1e-9)))
                    self.command(names, [a + ratio * (b - a) for a, b in zip(start, target)])
                    time.sleep(0.01)
                self.command(names, target)
                start, previous = target, t
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                actual = self.current(names)
                if goal.is_cancel_requested:
                    self.command(names, actual)
                    goal.canceled()
                    return result
                if max(abs(a - b) for a, b in zip(actual, target)) <= 0.01:
                    if kind == 'gripper':
                        result.position, result.reached_goal = actual[0], True
                    goal.succeed()
                    return result
                time.sleep(0.01)
            raise RuntimeError('Isaac target not reached within 5 seconds')
        except Exception as exc:
            if kind == 'arm':
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = str(exc)
            self.get_logger().error(str(exc))
            goal.abort()
            return result
        finally:
            with self.lock:
                self.busy.discard(kind)


def main():
    rclpy.init()
    node = Bridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
