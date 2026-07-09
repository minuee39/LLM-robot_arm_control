import argparse
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    CollisionObject,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from isaac_moveit_bridge.tcp_pose import tcp_target_to_tool0_target


ERROR_CODE_NAMES = {
    value: name
    for name, value in MoveItErrorCodes.__dict__.items()
    if name.isupper() and isinstance(value, int)
}


class MoveItPoseGoalClient(Node):
    def __init__(self) -> None:
        super().__init__("moveit_pose_goal")
        self.client = ActionClient(self, MoveGroup, "/move_action")

    def send_goal(self, args: argparse.Namespace) -> int:
        tcp_target = make_pose(args)
        tool0_target = tcp_target_to_tool0_target(tcp_target)

        self.log_pose("TCP target pose in base_link", tcp_target)
        self.log_pose("MoveIt tool0 target pose in base_link", tool0_target)
        self.get_logger().info("Waiting for /move_action...")

        if not self.client.wait_for_server(timeout_sec=args.action_timeout):
            self.get_logger().error("Timed out waiting for /move_action. Is move_group running?")
            return 1

        goal_msg = build_goal(args, tool0_target)
        self.get_logger().info(
            "Sending MoveGroup goal "
            f"(group={args.group}, link={args.link}, plan_only={not args.execute})"
        )

        send_future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal was rejected")
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        error_code = result.error_code.val
        error_name = ERROR_CODE_NAMES.get(error_code, f"UNKNOWN_{error_code}")

        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(
                f"MoveGroup succeeded: {error_name}, planning_time={result.planning_time:.3f}s"
            )
            return 0

        self.get_logger().error(f"MoveGroup failed: {error_name} ({error_code})")
        return 1

    def log_pose(self, label: str, pose: Pose) -> None:
        self.get_logger().info(
            "\n"
            f"{label}:\n"
            f"  position: x={pose.position.x:.6f}, y={pose.position.y:.6f}, z={pose.position.z:.6f}\n"
            "  orientation: "
            f"x={pose.orientation.x:.6f}, y={pose.orientation.y:.6f}, "
            f"z={pose.orientation.z:.6f}, w={pose.orientation.w:.6f}"
        )


def make_pose(args: argparse.Namespace) -> Pose:
    pose = Pose()
    pose.position.x = args.x
    pose.position.y = args.y
    pose.position.z = args.z
    pose.orientation.x = args.qx
    pose.orientation.y = args.qy
    pose.orientation.z = args.qz
    pose.orientation.w = args.qw
    return pose


def build_goal(args: argparse.Namespace, tool0_target: Pose) -> MoveGroup.Goal:
    goal = MoveGroup.Goal()
    request = goal.request
    request.group_name = args.group
    request.num_planning_attempts = args.planning_attempts
    request.allowed_planning_time = args.planning_time
    request.max_velocity_scaling_factor = args.velocity_scaling
    request.max_acceleration_scaling_factor = args.accel_scaling
    request.start_state.is_diff = True
    request.workspace_parameters.header.frame_id = args.frame
    request.workspace_parameters.min_corner.x = -2.0
    request.workspace_parameters.min_corner.y = -2.0
    request.workspace_parameters.min_corner.z = args.workspace_min_z
    request.workspace_parameters.max_corner.x = 2.0
    request.workspace_parameters.max_corner.y = 2.0
    request.workspace_parameters.max_corner.z = 2.0

    constraints = Constraints()
    constraints.name = "tool0_pose_goal"
    constraints.position_constraints.append(
        make_position_constraint(args, tool0_target)
    )
    constraints.orientation_constraints.append(
        make_orientation_constraint(args, tool0_target)
    )
    request.goal_constraints.append(constraints)

    if args.shoulder_lift_guard:
        request.path_constraints.joint_constraints.append(
            make_shoulder_lift_guard(args)
        )

    goal.planning_options.plan_only = not args.execute
    goal.planning_options.look_around = False
    goal.planning_options.replan = args.replan
    goal.planning_options.replan_attempts = args.replan_attempts
    goal.planning_options.replan_delay = 0.2
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    if args.floor_collision:
        goal.planning_options.planning_scene_diff.world.collision_objects.append(
            make_floor_collision_object(args)
        )
    return goal


def make_position_constraint(args: argparse.Namespace, pose: Pose) -> PositionConstraint:
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [args.position_tolerance]

    constraint = PositionConstraint()
    constraint.header.frame_id = args.frame
    constraint.link_name = args.link
    constraint.constraint_region.primitives.append(sphere)
    constraint.constraint_region.primitive_poses.append(pose)
    constraint.weight = 1.0
    return constraint


def make_orientation_constraint(args: argparse.Namespace, pose: Pose) -> OrientationConstraint:
    constraint = OrientationConstraint()
    constraint.header.frame_id = args.frame
    constraint.link_name = args.link
    constraint.orientation = pose.orientation
    constraint.absolute_x_axis_tolerance = args.orientation_tolerance
    constraint.absolute_y_axis_tolerance = args.orientation_tolerance
    constraint.absolute_z_axis_tolerance = args.orientation_tolerance
    constraint.weight = 1.0
    return constraint


def make_shoulder_lift_guard(args: argparse.Namespace) -> JointConstraint:
    constraint = JointConstraint()
    constraint.joint_name = "shoulder_lift_joint"
    constraint.position = (
        args.shoulder_lift_min + args.shoulder_lift_max
    ) * 0.5
    constraint.tolerance_below = constraint.position - args.shoulder_lift_min
    constraint.tolerance_above = args.shoulder_lift_max - constraint.position
    constraint.weight = 1.0
    return constraint


def make_floor_collision_object(args: argparse.Namespace) -> CollisionObject:
    floor = CollisionObject()
    floor.header.frame_id = args.frame
    floor.id = "moveit_floor_z_guard"
    floor.operation = CollisionObject.ADD

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [
        args.floor_size,
        args.floor_size,
        args.floor_thickness,
    ]

    pose = Pose()
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = args.floor_z - (args.floor_thickness * 0.5)
    pose.orientation.w = 1.0

    floor.primitives.append(box)
    floor.primitive_poses.append(pose)
    return floor


def parse_args(args: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a TCP pose target to MoveIt after converting it to a tool0 target."
    )
    parser.add_argument("--x", type=float, default=0.5, help="TCP target x in base_link.")
    parser.add_argument("--y", type=float, default=0.0, help="TCP target y in base_link.")
    parser.add_argument("--z", type=float, default=0.3, help="TCP target z in base_link.")
    parser.add_argument("--qx", type=float, default=0.0, help="TCP target quaternion x.")
    parser.add_argument("--qy", type=float, default=0.0, help="TCP target quaternion y.")
    parser.add_argument("--qz", type=float, default=0.0, help="TCP target quaternion z.")
    parser.add_argument("--qw", type=float, default=1.0, help="TCP target quaternion w.")
    parser.add_argument("--frame", default="base_link", help="Planning frame.")
    parser.add_argument("--group", default="ur_manipulator", help="MoveIt planning group.")
    parser.add_argument("--link", default="tool0", help="MoveIt goal link.")
    parser.add_argument("--planning-time", type=float, default=5.0)
    parser.add_argument("--planning-attempts", type=int, default=10)
    parser.add_argument("--velocity-scaling", type=float, default=0.05)
    parser.add_argument("--accel-scaling", type=float, default=0.03)
    parser.add_argument("--position-tolerance", type=float, default=0.01)
    parser.add_argument("--orientation-tolerance", type=float, default=0.3)
    parser.add_argument(
        "--no-floor-collision",
        dest="floor_collision",
        action="store_false",
        help="Do not add a floor collision object to the MoveIt planning scene.",
    )
    parser.set_defaults(floor_collision=True)
    parser.add_argument("--floor-z", type=float, default=0.0)
    parser.add_argument("--floor-size", type=float, default=4.0)
    parser.add_argument("--floor-thickness", type=float, default=0.10)
    parser.add_argument(
        "--workspace-min-z",
        type=float,
        default=0.0,
        help="Minimum z for the MoveIt planning workspace.",
    )
    parser.add_argument(
        "--no-shoulder-lift-guard",
        dest="shoulder_lift_guard",
        action="store_false",
        help="Allow shoulder_lift_joint outside the guarded range.",
    )
    parser.set_defaults(shoulder_lift_guard=True)
    parser.add_argument(
        "--shoulder-lift-min",
        type=float,
        default=-3.14,
        help="Lower shoulder_lift_joint limit used by the path guard.",
    )
    parser.add_argument(
        "--shoulder-lift-max",
        type=float,
        default=0.15,
        help="Upper shoulder_lift_joint limit used by the path guard.",
    )
    parser.add_argument("--action-timeout", type=float, default=10.0)
    parser.add_argument("--execute", action="store_true", help="Execute the plan in addition to planning.")
    parser.add_argument("--replan", action="store_true", help="Allow MoveIt to replan during execution.")
    parser.add_argument("--replan-attempts", type=int, default=3)
    return parser.parse_args(args)


def main(args: Optional[list[str]] = None) -> None:
    parsed_args = parse_args(args)
    rclpy.init(args=args)
    node = MoveItPoseGoalClient()
    try:
        exit_code = node.send_goal(parsed_args)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
