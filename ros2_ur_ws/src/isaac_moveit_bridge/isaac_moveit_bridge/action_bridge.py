import math
import time
from typing import Dict, Iterable, List

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

MOVEIT_GRIPPER_JOINT = "robotiq_finger_joint"
ISAAC_GRIPPER_JOINT = "finger_joint"
MOVEIT_JOINTS = ARM_JOINTS + [MOVEIT_GRIPPER_JOINT]

MOVEIT_TO_ISAAC = {
    **{joint_name: joint_name for joint_name in ARM_JOINTS},
    MOVEIT_GRIPPER_JOINT: ISAAC_GRIPPER_JOINT,
}
ISAAC_TO_MOVEIT = {isaac: moveit for moveit, isaac in MOVEIT_TO_ISAAC.items()}
COMMAND_PERIOD_SEC = 0.01
REVOLUTE_JOINTS = set(ARM_JOINTS)
INITIAL_MOVEIT_POSITIONS = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -1.57,
    "elbow_joint": 2.0,
    "wrist_1_joint": -1.57,
    "wrist_2_joint": 1.57,
    "wrist_3_joint": 3.14,
    "robotiq_finger_joint": 0.01,
}


def duration_to_sec(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def nearest_equivalent_angle(current: float, target: float) -> float:
    delta = target - current
    shortest_delta = math.atan2(math.sin(delta), math.cos(delta))
    return current + shortest_delta


def unwrap_revolute_targets(
    joint_names: Iterable[str],
    current_positions: Iterable[float],
    target_positions: Iterable[float],
) -> List[float]:
    unwrapped = []
    for joint_name, current, target in zip(joint_names, current_positions, target_positions):
        if joint_name in REVOLUTE_JOINTS:
            unwrapped.append(nearest_equivalent_angle(float(current), float(target)))
        else:
            unwrapped.append(float(target))
    return unwrapped


class IsaacMoveItActionBridge(Node):
    def __init__(self) -> None:
        super().__init__("isaac_moveit_action_bridge")
        self.command_pub = self.create_publisher(JointState, "/isaac_joint_commands", 10)
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.isaac_state_sub = self.create_subscription(
            JointState,
            "/isaac_joint_states",
            self.on_isaac_joint_states,
            10,
        )
        self.joint_state_timer = self.create_timer(0.02, self.publish_moveit_joint_states)

        self.latest_positions: Dict[str, float] = {
            joint_name: INITIAL_MOVEIT_POSITIONS[joint_name]
            for joint_name in MOVEIT_JOINTS
        }
        self.latest_velocities: Dict[str, float] = {joint_name: 0.0 for joint_name in MOVEIT_JOINTS}

        self.arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/ur_manipulator_controller/follow_joint_trajectory",
            execute_callback=self.execute_arm,
            goal_callback=self.accept_goal,
            cancel_callback=self.accept_cancel,
        )
        self.gripper_server = ActionServer(
            self,
            GripperCommand,
            "/gripper_controller/gripper_cmd",
            execute_callback=self.execute_gripper,
            goal_callback=self.accept_goal,
            cancel_callback=self.accept_cancel,
        )

        self.get_logger().info("MoveIt -> Isaac action bridge started")
        self.get_logger().info("Action: /ur_manipulator_controller/follow_joint_trajectory")
        self.get_logger().info("Action: /gripper_controller/gripper_cmd")
        self.get_logger().info("Topic out: /isaac_joint_commands")
        self.get_logger().info("Topic out: /joint_states")
        self.get_logger().info("Optional topic in: /isaac_joint_states")

    def accept_goal(self, goal_request):
        return GoalResponse.ACCEPT

    def accept_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def on_isaac_joint_states(self, msg: JointState) -> None:
        velocity_by_name = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        position_by_name = dict(zip(msg.name, msg.position))

        for isaac_name, position in position_by_name.items():
            moveit_name = ISAAC_TO_MOVEIT.get(isaac_name)
            if moveit_name is None:
                continue
            self.latest_positions[moveit_name] = float(position)
            self.latest_velocities[moveit_name] = float(velocity_by_name.get(isaac_name, 0.0))

        self.publish_moveit_joint_states()

    def publish_moveit_joint_states(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(MOVEIT_JOINTS)
        msg.position = [self.latest_positions[name] for name in msg.name]
        msg.velocity = [self.latest_velocities[name] for name in msg.name]
        self.joint_state_pub.publish(msg)

    def execute_arm(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        if not self.joints_supported(trajectory.joint_names):
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = "Trajectory contains unsupported joints"
            return result

        if not trajectory.points:
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "Trajectory has no points"
            return result

        start_time = time.monotonic()
        previous_time = 0.0
        previous_positions = [self.latest_positions[name] for name in trajectory.joint_names]

        for point in trajectory.points:
            target_time = duration_to_sec(point.time_from_start)
            target_positions = unwrap_revolute_targets(
                trajectory.joint_names,
                previous_positions,
                point.positions,
            )

            if target_time <= previous_time:
                self.publish_command(trajectory.joint_names, target_positions)
                previous_time = target_time
                previous_positions = target_positions
                continue

            while True:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = FollowJointTrajectory.Result()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "Trajectory canceled"
                    return result

                elapsed = time.monotonic() - start_time
                if elapsed >= target_time:
                    break

                ratio = (elapsed - previous_time) / (target_time - previous_time)
                ratio = max(0.0, min(1.0, ratio))
                interpolated_positions = [
                    start + (target - start) * ratio
                    for start, target in zip(previous_positions, target_positions)
                ]
                self.publish_command(trajectory.joint_names, interpolated_positions)
                time.sleep(COMMAND_PERIOD_SEC)

            self.publish_command(trajectory.joint_names, target_positions)
            previous_time = target_time
            previous_positions = target_positions

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Trajectory forwarded to Isaac Sim"
        return result

    def execute_gripper(self, goal_handle):
        target_position = float(goal_handle.request.command.position)
        self.publish_command([MOVEIT_GRIPPER_JOINT], [target_position])
        time.sleep(0.7)

        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = target_position
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = True
        return result

    def joints_supported(self, joint_names: Iterable[str]) -> bool:
        return all(joint_name in MOVEIT_TO_ISAAC for joint_name in joint_names)

    def publish_command(self, moveit_joint_names: Iterable[str], positions: Iterable[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []

        for moveit_joint_name, position in zip(moveit_joint_names, positions):
            msg.name.append(MOVEIT_TO_ISAAC[moveit_joint_name])
            msg.position.append(float(position))
            self.latest_positions[moveit_joint_name] = float(position)
            self.latest_velocities[moveit_joint_name] = 0.0

        self.command_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = IsaacMoveItActionBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
