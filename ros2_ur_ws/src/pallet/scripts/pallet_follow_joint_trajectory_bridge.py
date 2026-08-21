#!/usr/bin/env python3
"""Expose a five-axis FollowJointTrajectory action for the Isaac Pallet arm."""

from __future__ import annotations

import threading
import time
from typing import Iterable

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINTS = ("j1", "j2", "j3", "j4", "j5")
ACTION_NAME = "/pallet_arm_controller/follow_joint_trajectory"
COMMAND_TOPIC = "/isaac_joint_commands"
ISAAC_STATE_TOPIC = "/isaac_joint_states"


def duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


class PalletTrajectoryBridge(Node):
    def __init__(self) -> None:
        super().__init__("pallet_follow_joint_trajectory_bridge")
        self.declare_parameter("command_period", 0.01)
        self.declare_parameter("goal_tolerance", 0.08)
        self.declare_parameter("settle_timeout", 2.0)
        self.command_period = float(self.get_parameter("command_period").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.settle_timeout = float(self.get_parameter("settle_timeout").value)

        self._lock = threading.Lock()
        self._positions: dict[str, float] = {}
        self._velocities: dict[str, float] = {}
        self._command_pub = self.create_publisher(JointState, COMMAND_TOPIC, 10)
        self._joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, ISAAC_STATE_TOPIC, self._on_isaac_state, 10)
        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            ACTION_NAME,
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.get_logger().info(f"Action ready: {ACTION_NAME}")
        self.get_logger().info(f"Isaac topics: {COMMAND_TOPIC} -> {ISAAC_STATE_TOPIC}")

    def _on_isaac_state(self, msg: JointState) -> None:
        position_map = dict(zip(msg.name, msg.position))
        velocity_map = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        with self._lock:
            for name in JOINTS:
                if name in position_map:
                    self._positions[name] = float(position_map[name])
                    self._velocities[name] = float(velocity_map.get(name, 0.0))
        output = JointState()
        output.header = msg.header
        output.name = list(JOINTS)
        with self._lock:
            if not all(name in self._positions for name in JOINTS):
                return
            output.position = [self._positions[name] for name in JOINTS]
            output.velocity = [self._velocities.get(name, 0.0) for name in JOINTS]
        self._joint_state_pub.publish(output)

    def _publish_command(self, names: Iterable[str], positions: Iterable[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(names)
        msg.position = [float(value) for value in positions]
        self._command_pub.publish(msg)

    def _current(self, names: Iterable[str]) -> list[float] | None:
        with self._lock:
            if not all(name in self._positions for name in names):
                return None
            return [self._positions[name] for name in names]

    def _result(self, code: int, message: str) -> FollowJointTrajectory.Result:
        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = message
        return result

    def _execute(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        names = list(trajectory.joint_names)
        if len(names) != len(JOINTS) or set(names) != set(JOINTS):
            goal_handle.abort()
            return self._result(
                FollowJointTrajectory.Result.INVALID_JOINTS,
                f"Expected exactly {JOINTS}; received {tuple(names)}",
            )
        if not trajectory.points:
            goal_handle.abort()
            return self._result(FollowJointTrajectory.Result.INVALID_GOAL, "Trajectory is empty")

        current = self._current(names)
        if current is None:
            goal_handle.abort()
            return self._result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                "No joint feedback has been received from Isaac Sim",
            )

        previous_time = 0.0
        previous_positions = current
        started = time.monotonic()
        for point in trajectory.points:
            target_time = duration_seconds(point.time_from_start)
            target = [float(value) for value in point.positions]
            if len(target) != len(names) or target_time < previous_time:
                goal_handle.abort()
                return self._result(
                    FollowJointTrajectory.Result.INVALID_GOAL,
                    "Trajectory point size or time_from_start is invalid",
                )
            while time.monotonic() - started < target_time:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(FollowJointTrajectory.Result.SUCCESSFUL, "Canceled")
                elapsed = time.monotonic() - started
                interval = max(target_time - previous_time, 1e-9)
                ratio = max(0.0, min(1.0, (elapsed - previous_time) / interval))
                command = [
                    start + ratio * (end - start)
                    for start, end in zip(previous_positions, target)
                ]
                self._publish_command(names, command)
                time.sleep(self.command_period)
            self._publish_command(names, target)
            previous_time = target_time
            previous_positions = target

        deadline = time.monotonic() + self.settle_timeout
        errors: list[float] = []
        while time.monotonic() < deadline:
            actual = self._current(names)
            if actual is not None:
                errors = [abs(expected - observed) for expected, observed in zip(previous_positions, actual)]
                if max(errors) <= self.goal_tolerance:
                    goal_handle.succeed()
                    return self._result(
                        FollowJointTrajectory.Result.SUCCESSFUL,
                        f"Isaac reached the goal; max error={max(errors):.6f} rad",
                    )
            time.sleep(self.command_period)

        goal_handle.abort()
        max_error = max(errors) if errors else float("inf")
        return self._result(
            FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
            f"Isaac did not settle; max error={max_error:.6f} rad",
        )

    def destroy_node(self) -> None:
        self._server.destroy()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = PalletTrajectoryBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
