import argparse
import time
from types import SimpleNamespace
from typing import Optional

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node

from isaac_moveit_bridge.moveit_pose_goal import ERROR_CODE_NAMES, build_goal
from isaac_moveit_bridge.tcp_pose import tcp_target_to_tool0_target


TOP_DOWN_TCP_QUATERNION = (0.0, 0.70710678, 0.0, 0.70710678)


class MoveItPickPlaceDemo(Node):
    def __init__(self) -> None:
        super().__init__("moveit_pick_place_demo")
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            "/gripper_controller/gripper_cmd",
        )

    def run(self, args: argparse.Namespace) -> int:
        self.get_logger().info("Waiting for /move_action...")
        if not self.move_group_client.wait_for_server(timeout_sec=args.action_timeout):
            self.get_logger().error("Timed out waiting for /move_action")
            return 1

        if args.execute:
            self.get_logger().info("Waiting for /gripper_controller/gripper_cmd...")
            if not self.gripper_client.wait_for_server(timeout_sec=args.action_timeout):
                self.get_logger().error("Timed out waiting for /gripper_controller/gripper_cmd")
                return 1

        pick_pose = make_tcp_pose(
            args.pick_x,
            args.pick_y,
            args.pick_z,
            args.qx,
            args.qy,
            args.qz,
            args.qw,
        )
        place_pose = make_tcp_pose(
            args.place_x,
            args.place_y,
            args.place_z,
            args.qx,
            args.qy,
            args.qz,
            args.qw,
        )

        pick_approach = offset_pose_z(pick_pose, args.approach_height)
        place_approach = offset_pose_z(place_pose, args.approach_height)
        sequence = (
            [("pick_approach", pick_approach)]
            + make_vertical_waypoints(
                "pick_descent",
                "pick_grasp",
                pick_approach,
                pick_pose,
                args.descent_steps,
            )
            + [
                ("lift_after_grasp", offset_pose_z(pick_pose, args.lift_height)),
                ("place_approach", place_approach),
            ]
            + make_vertical_waypoints(
                "place_descent",
                "place",
                place_approach,
                place_pose,
                args.descent_steps,
            )
            + [("retreat_after_place", offset_pose_z(place_pose, args.lift_height))]
        )
        unsafe_waypoints = [
            (name, tcp_pose.position.z)
            for name, tcp_pose in sequence
            if tcp_pose.position.z < args.min_tcp_z
        ]
        if unsafe_waypoints and not args.allow_low_z:
            for name, z in unsafe_waypoints:
                self.get_logger().error(
                    f"{name}: TCP z={z:.3f} is below --min-tcp-z={args.min_tcp_z:.3f}"
                )
            self.get_logger().error(
                "Aborting before planning. Raise pick/place z, lower --min-tcp-z, "
                "or pass --allow-low-z only after the TCP orientation/gripper grasp is verified."
            )
            return 1

        self.get_logger().info(
            "Running pick-place demo "
            f"(execute={args.execute}, pick=({args.pick_x}, {args.pick_y}, {args.pick_z}), "
            f"place=({args.place_x}, {args.place_y}, {args.place_z}))"
        )

        if args.execute and not self.send_gripper_goal("open_start", args.open_position, args):
            return 1

        for name, tcp_pose in sequence:
            if name == "lift_after_grasp" and args.execute:
                if not self.send_gripper_goal("close_gripper", args.close_position, args):
                    return 1

            if name == "retreat_after_place" and args.execute:
                if not self.send_gripper_goal("open_gripper", args.open_position, args):
                    return 1

            if not self.send_arm_goal(name, tcp_pose, args):
                return 1

            time.sleep(args.step_pause)

        self.get_logger().info("Pick-place demo completed")
        return 0

    def send_arm_goal(self, name: str, tcp_target: Pose, args: argparse.Namespace) -> bool:
        tool0_target = tcp_target_to_tool0_target(tcp_target)
        self.log_pose(f"{name} TCP target", tcp_target)
        self.log_pose(f"{name} tool0 target", tool0_target)

        goal_msg = build_goal(make_move_group_args(args), tool0_target)
        send_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{name}: MoveGroup goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        error_code = result.error_code.val
        error_name = ERROR_CODE_NAMES.get(error_code, f"UNKNOWN_{error_code}")

        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(
                f"{name}: MoveGroup succeeded ({error_name}), "
                f"planning_time={result.planning_time:.3f}s"
            )
            return True

        self.get_logger().error(f"{name}: MoveGroup failed ({error_name}, {error_code})")
        return False

    def send_gripper_goal(self, name: str, position: float, args: argparse.Namespace) -> bool:
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = float(args.gripper_effort)

        self.get_logger().info(f"{name}: sending gripper position={position:.4f}")
        send_future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{name}: gripper goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f"{name}: gripper reached_goal={result.reached_goal}, "
            f"position={result.position:.4f}, effort={result.effort:.4f}"
        )
        return bool(result.reached_goal)

    def log_pose(self, label: str, pose: Pose) -> None:
        self.get_logger().info(
            "\n"
            f"{label}:\n"
            f"  position: x={pose.position.x:.6f}, y={pose.position.y:.6f}, z={pose.position.z:.6f}\n"
            "  orientation: "
            f"x={pose.orientation.x:.6f}, y={pose.orientation.y:.6f}, "
            f"z={pose.orientation.z:.6f}, w={pose.orientation.w:.6f}"
        )


def make_tcp_pose(x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def offset_pose_z(pose: Pose, offset: float) -> Pose:
    return make_tcp_pose(
        pose.position.x,
        pose.position.y,
        pose.position.z + offset,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )


def make_vertical_waypoints(
    prefix: str,
    final_name: str,
    top_pose: Pose,
    bottom_pose: Pose,
    steps: int,
) -> list[tuple[str, Pose]]:
    if steps < 1:
        return [(final_name, bottom_pose)]

    waypoints = []
    for index in range(1, steps + 1):
        ratio = index / steps
        z = top_pose.position.z + (bottom_pose.position.z - top_pose.position.z) * ratio
        name = final_name if index == steps else f"{prefix}_{index}"
        waypoints.append(
            (
                name,
                make_tcp_pose(
                    bottom_pose.position.x,
                    bottom_pose.position.y,
                    z,
                    bottom_pose.orientation.x,
                    bottom_pose.orientation.y,
                    bottom_pose.orientation.z,
                    bottom_pose.orientation.w,
                ),
            )
        )
    return waypoints


def make_move_group_args(args: argparse.Namespace) -> SimpleNamespace:
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
        execute=args.execute,
        replan=args.replan,
        replan_attempts=args.replan_attempts,
    )


def parse_args(args: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fixed-coordinate MoveIt pick-place demo using TCP-to-tool0 conversion."
    )
    parser.add_argument("--pick-x", type=float, default=-0.30)
    parser.add_argument("--pick-y", type=float, default=0.30)
    parser.add_argument("--pick-z", type=float, default=0.0275)
    parser.add_argument("--place-x", type=float, default=0.0)
    parser.add_argument("--place-y", type=float, default=0.45)
    parser.add_argument("--place-z", type=float, default=0.20)
    parser.add_argument("--approach-height", type=float, default=0.15)
    parser.add_argument("--lift-height", type=float, default=0.20)
    parser.add_argument(
        "--descent-steps",
        type=int,
        default=4,
        help="Number of same-x/y waypoints used for vertical descend motions.",
    )
    parser.add_argument(
        "--min-tcp-z",
        type=float,
        default=0.02,
        help="Abort if any TCP waypoint is below this z height.",
    )
    parser.add_argument(
        "--allow-low-z",
        action="store_true",
        help="Allow waypoints below --min-tcp-z. Use only after TCP orientation is verified.",
    )
    parser.add_argument("--qx", type=float, default=TOP_DOWN_TCP_QUATERNION[0])
    parser.add_argument("--qy", type=float, default=TOP_DOWN_TCP_QUATERNION[1])
    parser.add_argument("--qz", type=float, default=TOP_DOWN_TCP_QUATERNION[2])
    parser.add_argument("--qw", type=float, default=TOP_DOWN_TCP_QUATERNION[3])
    parser.add_argument("--open-position", type=float, default=0.0)
    parser.add_argument("--close-position", type=float, default=0.7)
    parser.add_argument("--gripper-effort", type=float, default=20.0)
    parser.add_argument("--frame", default="base_link")
    parser.add_argument("--group", default="ur_manipulator")
    parser.add_argument("--link", default="tool0")
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
    parser.add_argument("--step-pause", type=float, default=0.2)
    parser.add_argument("--execute", action="store_true", help="Execute in Isaac. Omit for plan-only dry run.")
    parser.add_argument("--replan", action="store_true")
    parser.add_argument("--replan-attempts", type=int, default=3)
    return parser.parse_args(args)


def main(args: Optional[list[str]] = None) -> None:
    parsed_args = parse_args(args)
    rclpy.init(args=args)
    node = MoveItPickPlaceDemo()
    try:
        exit_code = node.run(parsed_args)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
