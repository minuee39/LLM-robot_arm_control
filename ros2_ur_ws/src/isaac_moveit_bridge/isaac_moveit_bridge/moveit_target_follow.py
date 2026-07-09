import argparse
import math
from types import SimpleNamespace
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

from isaac_moveit_bridge.moveit_pose_goal import ERROR_CODE_NAMES, build_goal
from isaac_moveit_bridge.tcp_pose import tcp_target_to_tool0_target


TOP_DOWN_TCP_QUATERNION = (0.0, 0.70710678, 0.0, 0.70710678)
ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class MoveItTargetFollow(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("moveit_target_follow")
        self.args = args
        self.client = ActionClient(self, MoveGroup, "/move_action")
        self.latest_pose: Optional[PoseStamped] = None
        self.latest_joint_positions: dict[str, float] = {}
        self.last_goal_pose: Optional[Pose] = None
        self.has_successful_goal = False
        self.suppress_next_continuity_guard = False
        self.current_goal_used_continuity_guard = False
        self.goal_active = False

        self.create_subscription(PoseStamped, args.target_topic, self.on_target_pose, 10)
        self.create_subscription(TFMessage, args.tf_topic, self.on_tf, 10)
        self.create_subscription(JointState, args.joint_state_topic, self.on_joint_state, 10)
        self.create_timer(args.replan_period, self.tick)

    def on_target_pose(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def on_joint_state(self, msg: JointState) -> None:
        self.latest_joint_positions.update(
            {
                joint_name: float(position)
                for joint_name, position in zip(msg.name, msg.position)
                if joint_name in ARM_JOINTS
            }
        )

    def on_tf(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            if not frame_matches(transform.child_frame_id, self.args.target_frame):
                continue

            pose_msg = PoseStamped()
            pose_msg.header = transform.header
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            pose_msg.pose.orientation = transform.transform.rotation
            self.latest_pose = pose_msg
            return

    def tick(self) -> None:
        if self.goal_active or self.latest_pose is None:
            return

        tcp_target = self.make_tcp_target(self.latest_pose.pose)
        if self.last_goal_pose is not None and pose_distance(tcp_target, self.last_goal_pose) < self.args.min_replan_distance:
            return

        if not self.client.server_is_ready():
            self.get_logger().debug("Waiting for /move_action...")
            return

        tool0_target = tcp_target_to_tool0_target(tcp_target)
        use_continuity_guard = (
            self.args.joint_continuity_guard
            and self.has_successful_goal
            and not self.suppress_next_continuity_guard
        )
        goal_msg = build_goal(
            make_move_group_args(
                self.args,
                self.latest_joint_positions,
                use_continuity_guard,
            ),
            tool0_target,
        )
        self.goal_active = True
        self.last_goal_pose = tcp_target
        self.current_goal_used_continuity_guard = use_continuity_guard
        self.suppress_next_continuity_guard = False

        self.get_logger().info(
            "Sending MoveIt target-follow goal "
            f"tcp=({tcp_target.position.x:.3f}, {tcp_target.position.y:.3f}, {tcp_target.position.z:.3f}), "
            f"joint_continuity_guard={use_continuity_guard}"
        )
        send_future = self.client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.goal_active = False
            self.get_logger().error("MoveIt target-follow goal rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future) -> None:
        self.goal_active = False
        result = future.result().result
        error_code = result.error_code.val
        error_name = ERROR_CODE_NAMES.get(error_code, f"UNKNOWN_{error_code}")

        if error_code == MoveItErrorCodes.SUCCESS:
            self.has_successful_goal = True
            self.get_logger().info(
                f"MoveIt target-follow goal succeeded ({error_name}), planning_time={result.planning_time:.3f}s"
            )
            return

        self.get_logger().warning(f"MoveIt target-follow goal failed ({error_name}, {error_code})")
        self.last_goal_pose = None
        if self.current_goal_used_continuity_guard:
            self.suppress_next_continuity_guard = True
            self.get_logger().warning(
                "Retrying next cycle without joint continuity guard because the guarded plan failed"
            )

    def make_tcp_target(self, target_pose: Pose) -> Pose:
        pose = Pose()
        pose.position.x = target_pose.position.x + self.args.offset_x
        pose.position.y = target_pose.position.y + self.args.offset_y
        pose.position.z = target_pose.position.z + self.args.offset_z

        if self.args.use_target_orientation:
            pose.orientation = target_pose.orientation
        else:
            pose.orientation.x = self.args.qx
            pose.orientation.y = self.args.qy
            pose.orientation.z = self.args.qz
            pose.orientation.w = self.args.qw

        return pose


def pose_distance(left: Pose, right: Pose) -> float:
    dx = left.position.x - right.position.x
    dy = left.position.y - right.position.y
    dz = left.position.z - right.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def frame_matches(frame_id: str, expected_frame: str) -> bool:
    normalized = frame_id.strip("/")
    expected = expected_frame.strip("/")
    normalized_leaf = normalized.split("/")[-1].lower()
    expected_lower = expected.lower()
    return (
        normalized == expected
        or normalized.endswith("/" + expected)
        or normalized_leaf == expected_lower
        or expected_lower in normalized_leaf
    )


def make_move_group_args(
    args: argparse.Namespace,
    latest_joint_positions: dict[str, float],
    use_continuity_guard: bool,
) -> SimpleNamespace:
    return SimpleNamespace(
        frame=args.frame,
        group=args.group,
        link=args.link,
        planning_attempts=args.planning_attempts,
        planning_time=args.planning_time,
        velocity_scaling=args.velocity_scaling,
        accel_scaling=args.accel_scaling,
        position_tolerance=args.position_tolerance,
        orientation_tolerance=args.orientation_tolerance,
        floor_collision=args.floor_collision,
        floor_z=args.floor_z,
        floor_size=args.floor_size,
        floor_thickness=args.floor_thickness,
        workspace_min_z=args.workspace_min_z,
        shoulder_lift_guard=args.shoulder_lift_guard,
        shoulder_lift_min=args.shoulder_lift_min,
        shoulder_lift_max=args.shoulder_lift_max,
        execute=True,
        replan=args.replan,
        replan_attempts=args.replan_attempts,
        path_joint_constraints=make_continuity_constraints(
            args,
            latest_joint_positions,
            use_continuity_guard,
        ),
    )


def make_continuity_constraints(
    args: argparse.Namespace,
    latest_joint_positions: dict[str, float],
    use_continuity_guard: bool,
) -> list[JointConstraint]:
    if not use_continuity_guard or not latest_joint_positions:
        return []

    constraints = []
    for joint_name in ARM_JOINTS:
        if joint_name not in latest_joint_positions:
            continue

        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.position = latest_joint_positions[joint_name]
        constraint.tolerance_above = args.joint_continuity_tolerance
        constraint.tolerance_below = args.joint_continuity_tolerance
        constraint.weight = 1.0
        constraints.append(constraint)
    return constraints


def parse_args(args: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Follow /target_pose by repeatedly planning and executing MoveIt2 trajectories."
    )
    parser.add_argument("--target-topic", default="/target_pose")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--target-frame", default="TargetCube")
    parser.add_argument("--frame", default="base_link")
    parser.add_argument("--group", default="ur_manipulator")
    parser.add_argument("--link", default="tool0")
    parser.add_argument("--offset-x", type=float, default=0.0)
    parser.add_argument("--offset-y", type=float, default=0.0)
    parser.add_argument("--offset-z", type=float, default=0.0)
    parser.add_argument("--use-target-orientation", action="store_true")
    parser.add_argument("--qx", type=float, default=TOP_DOWN_TCP_QUATERNION[0])
    parser.add_argument("--qy", type=float, default=TOP_DOWN_TCP_QUATERNION[1])
    parser.add_argument("--qz", type=float, default=TOP_DOWN_TCP_QUATERNION[2])
    parser.add_argument("--qw", type=float, default=TOP_DOWN_TCP_QUATERNION[3])
    parser.add_argument("--replan-period", type=float, default=0.5)
    parser.add_argument("--min-replan-distance", type=float, default=0.03)
    parser.add_argument("--planning-time", type=float, default=2.0)
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--velocity-scaling", type=float, default=0.15)
    parser.add_argument("--accel-scaling", type=float, default=0.10)
    parser.add_argument("--position-tolerance", type=float, default=0.02)
    parser.add_argument("--orientation-tolerance", type=float, default=0.5)
    parser.add_argument("--no-floor-collision", dest="floor_collision", action="store_false")
    parser.set_defaults(floor_collision=True)
    parser.add_argument("--floor-z", type=float, default=0.0)
    parser.add_argument("--floor-size", type=float, default=4.0)
    parser.add_argument("--floor-thickness", type=float, default=0.10)
    parser.add_argument("--workspace-min-z", type=float, default=0.0)
    parser.add_argument("--no-shoulder-lift-guard", dest="shoulder_lift_guard", action="store_false")
    parser.set_defaults(shoulder_lift_guard=True)
    parser.add_argument("--shoulder-lift-min", type=float, default=-3.14)
    parser.add_argument("--shoulder-lift-max", type=float, default=0.15)
    parser.add_argument("--replan", action="store_true")
    parser.add_argument("--replan-attempts", type=int, default=3)
    parser.add_argument(
        "--no-joint-continuity-guard",
        dest="joint_continuity_guard",
        action="store_false",
        help="Disable guarded follow-up plans that keep each arm joint near the current joint state.",
    )
    parser.set_defaults(joint_continuity_guard=True)
    parser.add_argument(
        "--joint-continuity-tolerance",
        type=float,
        default=1.0,
        help=(
            "Allowed radians away from the current joint state for each arm joint during guarded "
            "follow-up target following plans."
        ),
    )
    return parser.parse_args(args)


def main(args: Optional[list[str]] = None) -> None:
    parsed_args = parse_args(args)
    rclpy.init(args=args)
    node = MoveItTargetFollow(parsed_args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
