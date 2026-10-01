#!/usr/bin/env python3
"""Measured-feedback MoveIt/Isaac adapter. Arm radians; J8 metres."""
import math
import json
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

ARM = [f'J{i}' for i in range(1, 7)]
JOINTS = ARM + ['J8']
JAWS = ['J8', 'J9']
FEEDBACK_JOINTS = ARM + JAWS
SAFETY_STOP_CODES = {
    'SLIP_DETECTED', 'BLOCK_DROP_DETECTED', 'TILT_DETECTED', 'TELEMETRY_STALE',
}


class SafetyStop:
    """Validate and latch fresh, explicitly enabled safety events."""

    def __init__(self, enabled, max_age_seconds=2.0):
        self.enabled = enabled
        self.max_age_seconds = max_age_seconds
        self.event = None

    def ingest(self, raw, now):
        try:
            event = json.loads(raw)
        except (TypeError, json.JSONDecodeError):
            return False, 'invalid_json'
        if not isinstance(event, dict) or event.get('code') not in SAFETY_STOP_CODES:
            return False, 'unsupported_event'
        published = event.get('published_at_unix')
        if (not isinstance(published, (int, float)) or not math.isfinite(published) or
                published > now + 1.0 or now - published > self.max_age_seconds):
            return False, 'stale_event'
        if not self.enabled:
            return False, 'disabled'
        if self.event is not None:
            return False, 'already_latched'
        self.event = event
        return True, 'latched'

    def reset(self):
        self.event = None


class BilateralContact:
    """Require stationary, loaded jaws over distinct measured samples."""

    def __init__(self):
        self.positions = None
        self.since = None

    def update(self, positions, efforts, stamp):
        if any(effort is None or effort < 1.0 for effort in efforts):
            self.positions, self.since = None, None
            return False
        if (self.positions is None or
                any(abs(a - b) > 0.0003 for a, b in zip(positions, self.positions))):
            self.positions, self.since = tuple(positions), stamp
            return False
        return stamp - self.since >= 0.4


class StableTarget:
    """Require fresh feedback to remain at rest inside a goal tolerance."""

    def __init__(self, duration, velocity_tolerance):
        self.duration = duration
        self.velocity_tolerance = velocity_tolerance
        self.since = None
        self.last_stamp = None

    def update(self, within_tolerance, velocities, stamp):
        if (not within_tolerance or any(value is None for value in velocities) or
                any(abs(value) > self.velocity_tolerance for value in velocities)):
            self.since, self.last_stamp = None, stamp
            return False
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.since = None
        self.last_stamp = stamp
        if self.since is None:
            self.since = stamp
            return False
        return stamp - self.since >= self.duration - 1e-9


class Bridge(Node):
    def __init__(self):
        super().__init__('pallet_gripper_isaac_bridge')
        config = Path(get_package_share_directory('pallet_gripper_moveit_config'))
        initial = yaml.safe_load((config / 'config/initial_positions.yaml').read_text())[
            'initial_positions'
        ]
        gripper_limit = yaml.safe_load((config / 'config/joint_limits.yaml').read_text())[
            'joint_limits'
        ]['J8']
        physics = yaml.safe_load((config / 'config/isaac_physics.yaml').read_text())
        self.gripper_lower = float(gripper_limit['min_position'])
        self.gripper_upper = float(gripper_limit['max_position'])
        self.gripper_goal_tolerance = 0.001
        self.gripper_open_velocity = float(physics['gripper_open_velocity_m_s'])
        self.gripper_close_velocity = float(physics['gripper_close_velocity_m_s'])
        self.gripper_settle_velocity = float(physics['gripper_settle_velocity_m_s'])
        self.gripper_settle_seconds = float(physics['gripper_settle_seconds'])
        for name, value in (
                ('gripper_open_velocity_m_s', self.gripper_open_velocity),
                ('gripper_close_velocity_m_s', self.gripper_close_velocity),
                ('gripper_settle_velocity_m_s', self.gripper_settle_velocity),
                ('gripper_settle_seconds', self.gripper_settle_seconds)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        self.positions = {joint: float(initial[joint]) for joint in JOINTS}
        self.positions['J9'] = self.positions['J8']
        self.efforts = {}
        self.velocities = {}
        self.stamp = 0.0
        self.have_feedback = False
        self.lock = threading.Lock()
        self.busy = set()
        self.declare_parameter('safety_cancel_enabled', False)
        self.declare_parameter('safety_event_max_age_seconds', 2.0)
        self.safety_stop = SafetyStop(
            self.get_parameter('safety_cancel_enabled').value,
            float(self.get_parameter('safety_event_max_age_seconds').value),
        )
        self.pub = self.create_publisher(JointState, '/isaac_joint_commands', 10)
        self.states = self.create_publisher(JointState, '/joint_states', 10)
        group = ReentrantCallbackGroup()
        event_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(JointState, '/isaac_joint_states', self.feedback,
                                 qos_profile_sensor_data, callback_group=group)
        self.create_subscription(
            String, '/pallet/safety_events', self.on_safety_event,
            event_qos, callback_group=group)
        self.safety_status = self.create_publisher(
            String, '/pallet/safety_status', event_qos)
        self.create_service(
            Trigger, '/pallet/reset_safety_stop', self.reset_safety_stop,
            callback_group=group)
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

    def publish_safety_status(self, state, event=None, detail=None):
        payload = {
            'state': state,
            'safety_cancel_enabled': self.safety_stop.enabled,
            'published_at_unix': time.time(),
        }
        if event:
            payload['event'] = event
        if detail:
            payload['detail'] = detail
        self.safety_status.publish(String(data=json.dumps(payload, separators=(',', ':'))))

    def hold_measured_state(self):
        with self.lock:
            if not self.have_feedback or time.monotonic() - self.stamp > 1:
                return False
            arm = [self.positions[joint] for joint in ARM]
            jaws = [self.positions[joint] for joint in JAWS]
        self.command(ARM, arm)
        self.command(JAWS, jaws)
        return True

    def on_safety_event(self, msg):
        with self.lock:
            latched, reason = self.safety_stop.ingest(msg.data, time.time())
            event = dict(self.safety_stop.event) if latched else None
        if not latched:
            if reason not in {'disabled', 'already_latched'}:
                self.get_logger().warning(f'Ignored safety event: {reason}')
            return
        held = self.hold_measured_state()
        self.publish_safety_status('STOPPED', event, 'measured_hold' if held else 'feedback_stale')
        self.get_logger().error(
            f'Safety stop latched for {event["code"]}; '
            f'measured hold {"commanded" if held else "unavailable"}')

    def reset_safety_stop(self, _request, response):
        with self.lock:
            fresh = self.have_feedback and time.monotonic() - self.stamp <= 1
            idle = not self.busy
            if fresh and idle:
                self.safety_stop.reset()
                response.success = True
                response.message = 'Safety stop reset; recovery goals may be accepted'
            else:
                response.success = False
                response.message = 'Reset denied: fresh feedback and idle controllers are required'
        self.publish_safety_status('ARMED' if response.success else 'STOPPED', detail=response.message)
        return response

    def safety_latched(self):
        with self.lock:
            return self.safety_stop.event is not None

    def publish_state(self, values):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        # MoveIt derives J9 from its URDF mimic relation. Keep that planning
        # contract separate from the independent physical measurements used
        # below; both measured jaws remain available on /isaac_joint_states.
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
        if not all(j in values and math.isfinite(values[j]) for j in FEEDBACK_JOINTS):
            return
        # PhysX float feedback can overshoot the exact URDF endpoint by a few
        # nanometres. Normalize only sub-micrometre boundary roundoff so MoveIt
        # does not reject an otherwise valid measured start state.
        for jaw in JAWS:
            if self.gripper_lower - 1e-6 <= values[jaw] < self.gripper_lower:
                values[jaw] = self.gripper_lower
            elif self.gripper_upper < values[jaw] <= self.gripper_upper + 1e-6:
                values[jaw] = self.gripper_upper
        with self.lock:
            self.positions, self.stamp = values, time.monotonic()
            self.efforts = (
                dict(zip(msg.name, msg.effort))
                if len(msg.effort) == len(msg.name)
                and all(math.isfinite(value) for value in msg.effort)
                else {}
            )
            self.velocities = (
                dict(zip(msg.name, msg.velocity))
                if len(msg.velocity) == len(msg.name)
                and all(math.isfinite(value) for value in msg.velocity)
                else {}
            )
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
        elif (not math.isfinite(req.command.position) or
              not self.gripper_lower <= req.command.position <= self.gripper_upper):
            return GoalResponse.REJECT
        with self.lock:
            if (self.safety_stop.event is not None or kind in self.busy or not self.have_feedback or
                    time.monotonic() - self.stamp > 1):
                return GoalResponse.REJECT
            self.busy.add(kind)
        return GoalResponse.ACCEPT

    def current(self, names):
        with self.lock:
            if not self.have_feedback or time.monotonic() - self.stamp > 1:
                raise RuntimeError('Isaac feedback timeout')
            return [self.positions[j] for j in names]

    def gripper_feedback(self):
        """Return one fresh snapshot; never mix jaw positions and old efforts."""
        with self.lock:
            if not self.have_feedback or time.monotonic() - self.stamp > 1:
                raise RuntimeError('Isaac feedback timeout')
            return (
                [self.positions[jaw] for jaw in JAWS],
                [abs(self.efforts[jaw]) if jaw in self.efforts else None for jaw in JAWS],
                [self.velocities[jaw] if jaw in self.velocities else None for jaw in JAWS],
                self.stamp,
            )

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
                initial_jaws, _, _, _ = self.gripper_feedback()
                closing = all(target < position - 1e-6 for position in initial_jaws)
                gripper_travel = max(abs(target - position) for position in initial_jaws)
                gripper_velocity = (
                    self.gripper_close_velocity if closing else self.gripper_open_velocity)
                points = [(max(0.1, gripper_travel / gripper_velocity), [target])]
            began, previous = time.monotonic(), 0.0
            for t, target in points:
                while time.monotonic() - began < t:
                    actual = self.current(names)
                    if self.safety_latched():
                        self.hold_measured_state()
                        if kind == 'arm':
                            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                            result.error_string = 'Safety stop latched'
                        goal.abort()
                        return result
                    if goal.is_cancel_requested:
                        hold_names = JAWS if kind == 'gripper' else names
                        self.command(hold_names, self.current(hold_names))
                        goal.canceled()
                        return result
                    ratio = min(1.0, max(0.0, (time.monotonic() - began - previous) / max(t - previous, 1e-9)))
                    self.command(names, [a + ratio * (b - a) for a, b in zip(start, target)])
                    time.sleep(0.01)
                self.command(names, target)
                start, previous = target, t
            # The simulated linear jaw settles at about 5 mm/s. Keep a finite
            # distance-based deadline so a full 33.5 mm stroke is not reported
            # failed while measured feedback is still converging.
            settle_timeout = (
                max(5.0, gripper_travel / 0.004 + 2.0)
                if kind == 'gripper' else 5.0
            )
            deadline = time.monotonic() + settle_timeout
            contact = BilateralContact()
            stable_target = StableTarget(
                self.gripper_settle_seconds, self.gripper_settle_velocity)
            while time.monotonic() < deadline:
                if kind == 'gripper':
                    actual, efforts, velocities, feedback_stamp = self.gripper_feedback()
                    expected = [target[0], target[0]]
                    effort = min(efforts) if all(e is not None for e in efforts) else 0.0
                else:
                    actual = self.current(names)
                    expected = target
                if self.safety_latched():
                    self.hold_measured_state()
                    if kind == 'arm':
                        result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        result.error_string = 'Safety stop latched'
                    goal.abort()
                    return result
                if goal.is_cancel_requested:
                    self.command(JAWS if kind == 'gripper' else names, actual)
                    goal.canceled()
                    return result
                tolerance = self.gripper_goal_tolerance if kind == 'gripper' else 0.01
                within_tolerance = max(
                    abs(a - b) for a, b in zip(actual, expected)) <= tolerance
                settled = (
                    stable_target.update(within_tolerance, velocities, feedback_stamp)
                    if kind == 'gripper' else within_tolerance)
                if settled:
                    if kind == 'gripper':
                        result.position, result.reached_goal = actual[0], True
                        result.effort = effort
                    goal.succeed()
                    return result
                if kind == 'gripper' and closing:
                    if contact.update(actual, efforts, feedback_stamp):
                        result.position = actual[0]
                        result.effort = effort
                        result.stalled = True
                        result.reached_goal = False
                        self.get_logger().info(
                            f'Isaac bilateral gripper contact at '
                            f'J8={actual[0]:.6f}, J9={actual[1]:.6f} m '
                            f'(target {target[0]:.6f} m, '
                            f'efforts {efforts[0]:.2f}/{efforts[1]:.2f} N)'
                        )
                        goal.succeed()
                        return result
                time.sleep(0.01)
            raise RuntimeError(
                f'Isaac target not reached within {settle_timeout:.1f} seconds'
            )
        except Exception as exc:
            if kind == 'arm':
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = str(exc)
            # If feedback is still fresh, stop pursuing the failed target.
            # With stale feedback do not fabricate a hold position or success.
            hold_names = JAWS if kind == 'gripper' else names
            try:
                hold_positions = self.current(hold_names)
            except RuntimeError:
                hold_positions = None
            if hold_positions is not None:
                self.command(hold_names, hold_positions)
                if kind == 'gripper':
                    result.position = hold_positions[0]
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
