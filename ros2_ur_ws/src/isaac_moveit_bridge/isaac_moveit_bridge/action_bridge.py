import fcntl
import math
import time
from pathlib import Path
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
GRIPPER_FEEDBACK_TIMEOUT_SEC = 2.5
GRIPPER_MIN_MOTION_RAD = 0.01
GRIPPER_POSITION_TOLERANCE_RAD = 0.01
GRIPPER_CLOSE_POSITION_TOLERANCE_RAD = 0.02
GRIPPER_BLOCK_CONTACT_MIN_POSITION_RAD = 0.47
GRIPPER_STALL_VELOCITY_RAD_SEC = 0.03
GRIPPER_STALL_POSITION_DELTA_RAD = 0.002
GRIPPER_STALL_HOLD_SEC = 0.25
GRIPPER_FINAL_SETTLE_SEC = 0.25
GRIPPER_OPEN_POSITION_MAX = 0.05
MOVEIT_GRIPPER_POSITION_MIN = 0.0
MOVEIT_GRIPPER_POSITION_MAX = 0.7
REVOLUTE_JOINTS = set(ARM_JOINTS)
INITIAL_MOVEIT_POSITIONS = {
    "shoulder_pan_joint": 0.56,
    "shoulder_lift_joint": -1.78,
    "elbow_joint": 1.41,
    "wrist_1_joint": 1.94,
    "wrist_2_joint": 1.57,
    "wrist_3_joint": -2.58,
    "robotiq_finger_joint": 0.01,
}
LOCK_FILE = Path("/tmp/isaac_moveit_action_bridge.lock")


def duration_to_sec(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def _clamp_moveit_gripper_position(position: float) -> float:
    return max(
        MOVEIT_GRIPPER_POSITION_MIN,
        min(MOVEIT_GRIPPER_POSITION_MAX, float(position)),
    )


def moveit_gripper_position_to_isaac(position: float) -> float:
    # IsaacArticulationController.positionCommand uses radians.  This bridge
    # talks to that controller directly, not to ParallelGripper.forward(),
    # whose public examples use degree-valued convenience commands.
    return _clamp_moveit_gripper_position(position)


def isaac_gripper_position_to_moveit(position: float) -> float:
    # /isaac_joint_states is populated from get_joint_positions(), so feedback
    # is already in radians and must not be converted from degrees again.
    return _clamp_moveit_gripper_position(position)


def isaac_gripper_velocity_to_moveit(velocity: float) -> float:
    return float(velocity)


def gripper_motion_complete(
    initial_position: float,
    current_position: float,
    target_position: float,
    current_velocity: float,
    stable_for: float,
) -> tuple[bool, bool]:
    """Return (complete, stalled) from measured articulation feedback.

    A close command can legitimately stop before its target when the pads contact
    an object, so stable low velocity after measurable motion counts as completion.
    """
    closing = target_position > initial_position
    position_tolerance = (
        GRIPPER_CLOSE_POSITION_TOLERANCE_RAD
        if closing
        else GRIPPER_POSITION_TOLERANCE_RAD
    )
    reached = abs(current_position - target_position) <= position_tolerance
    # The 51.5 mm blocks stop the 2F-140 finger joint around 0.478--0.485 rad.
    # Contact varies slightly with pose and physics steps, so target error alone
    # is not a reliable grasp-completion signal.  Crossing 0.47 rad means the
    # pads have entered the block contact range; the caller then keeps squeezing
    # for the final settle period before allowing lift.
    block_contact = closing and current_position >= GRIPPER_BLOCK_CONTACT_MIN_POSITION_RAD
    moved = abs(current_position - initial_position) >= GRIPPER_MIN_MOTION_RAD
    stalled = (
        moved
        and abs(current_velocity) <= GRIPPER_STALL_VELOCITY_RAD_SEC
        and stable_for >= GRIPPER_STALL_HOLD_SEC
    )
    return reached or block_contact or stalled, stalled and not reached and not block_contact


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
        self.latest_feedback_at: Dict[str, float] = {joint_name: 0.0 for joint_name in MOVEIT_JOINTS}

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
        self.get_logger().info(
            "Gripper commands and Isaac articulation feedback use radians"
        )

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
            velocity = float(velocity_by_name.get(isaac_name, 0.0))
            if isaac_name == ISAAC_GRIPPER_JOINT:
                self.latest_positions[moveit_name] = isaac_gripper_position_to_moveit(position)
                self.latest_velocities[moveit_name] = isaac_gripper_velocity_to_moveit(velocity)
            else:
                self.latest_positions[moveit_name] = float(position)
                self.latest_velocities[moveit_name] = velocity
            self.latest_feedback_at[moveit_name] = time.monotonic()

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
        isaac_target_position = moveit_gripper_position_to_isaac(target_position)
        self.get_logger().info(
            f"Gripper command: MoveIt={target_position:.4f} rad -> "
            f"Isaac={isaac_target_position:.4f} rad"
        )
        self.publish_command([MOVEIT_GRIPPER_JOINT], [target_position])

        initial_position = self.latest_positions[MOVEIT_GRIPPER_JOINT]
        command_started_at = time.monotonic()
        low_velocity_started_at = None
        low_velocity_anchor_position = initial_position
        stalled = False
        contact_confirmed = False
        current_position = initial_position
        while time.monotonic() - command_started_at < GRIPPER_FEEDBACK_TIMEOUT_SEC:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = GripperCommand.Result()
                result.position = self.latest_positions[MOVEIT_GRIPPER_JOINT]
                result.stalled = False
                result.reached_goal = False
                return result

            # Re-publish while waiting so the Isaac controller continues to hold
            # the requested position even if another stage updates its inputs.
            self.publish_command([MOVEIT_GRIPPER_JOINT], [target_position])
            current_position = self.latest_positions[MOVEIT_GRIPPER_JOINT]
            current_velocity = self.latest_velocities[MOVEIT_GRIPPER_JOINT]
            now = time.monotonic()
            position_stable = (
                abs(current_position - low_velocity_anchor_position)
                <= GRIPPER_STALL_POSITION_DELTA_RAD
            )
            if abs(current_velocity) <= GRIPPER_STALL_VELOCITY_RAD_SEC and position_stable:
                if low_velocity_started_at is None:
                    low_velocity_started_at = now
            else:
                low_velocity_started_at = None
                low_velocity_anchor_position = current_position
            stable_for = 0.0 if low_velocity_started_at is None else now - low_velocity_started_at
            complete, stalled = gripper_motion_complete(
                initial_position,
                current_position,
                target_position,
                current_velocity,
                stable_for,
            )
            if complete:
                contact_confirmed = (
                    target_position > initial_position
                    and current_position >= GRIPPER_BLOCK_CONTACT_MIN_POSITION_RAD
                )
                time.sleep(GRIPPER_FINAL_SETTLE_SEC)
                break
            time.sleep(COMMAND_PERIOD_SEC)
        else:
            self.get_logger().error(
                f"Gripper feedback timeout: target={target_position:.4f}, "
                f"measured={current_position:.4f} rad"
            )
            goal_handle.abort()
            result = GripperCommand.Result()
            result.position = current_position
            result.effort = 0.0
            result.stalled = False
            result.reached_goal = False
            return result

        self.get_logger().info(
            f"Gripper motion confirmed: measured={current_position:.4f} rad, "
            f"target={target_position:.4f} rad, contact={contact_confirmed}, stalled={stalled}"
        )

        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = current_position
        result.effort = 0.0
        result.stalled = stalled
        result.reached_goal = not stalled
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
            isaac_position = (
                moveit_gripper_position_to_isaac(position)
                if moveit_joint_name == MOVEIT_GRIPPER_JOINT
                else float(position)
            )
            msg.position.append(isaac_position)
        self.command_pub.publish(msg)


def main() -> None:
    lock_file = LOCK_FILE.open("w", encoding="utf-8")
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print(
            f"[ERROR] Another Isaac MoveIt action bridge is already running ({LOCK_FILE}).",
            flush=True,
        )
        lock_file.close()
        return

    rclpy.init()
    node = IsaacMoveItActionBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
