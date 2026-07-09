import argparse
import time
from dataclasses import dataclass, field
from enum import Enum
from types import SimpleNamespace
from typing import Optional

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node

from isaac_moveit_bridge.moveit_pose_goal import (
    ERROR_CODE_NAMES,
    build_goal,
    make_attached_box_collision_object,
    make_box_collision_object,
)
from isaac_moveit_bridge.tcp_pose import (
    TOOL0_TO_ROBOTIQ_GRASPING_FRAME,
    tcp_target_to_tool0_target,
)


TOP_DOWN_TCP_QUATERNION = (0.0, 0.70710678, 0.0, 0.70710678)
ROBOTIQ_TOUCH_LINKS = [
    "robotiq_base_link",
    "robotiq_left_inner_finger",
    "robotiq_left_inner_knuckle",
    "robotiq_left_inner_finger_pad",
    "robotiq_left_outer_finger",
    "robotiq_left_outer_knuckle",
    "robotiq_right_inner_finger",
    "robotiq_right_inner_knuckle",
    "robotiq_right_inner_finger_pad",
    "robotiq_right_outer_finger",
    "robotiq_right_outer_knuckle",
]


class StageKind(Enum):
    ARM = "arm"
    GRIPPER = "gripper"
    SCENE = "scene"


@dataclass(frozen=True)
class PickPlaceStage:
    name: str
    kind: StageKind
    tcp_pose: Optional[Pose] = None
    gripper_position: Optional[float] = None
    object_in_world: bool = True
    object_attached: bool = False
    object_touch_allowed: bool = False
    notes: tuple[str, ...] = field(default_factory=tuple)


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

        task = build_pick_place_task(args)
        unsafe_waypoints = [
            (stage.name, stage.tcp_pose.position.z)
            for stage in task
            if stage.tcp_pose is not None and stage.tcp_pose.position.z < args.min_tcp_z
        ]
        if unsafe_waypoints and not args.allow_low_z:
            for name, z in unsafe_waypoints:
                self.get_logger().error(
                    f"{name}: TCP z={z:.3f} is below --min-tcp-z={args.min_tcp_z:.3f}"
                )
            self.get_logger().error(
                "Aborting before planning. Raise --pick-z/--place-z, lower --min-tcp-z, "
                "or pass --allow-low-z only after the TCP frame is verified."
            )
            return 1

        self.get_logger().info(
            "Running MTC-style pick-place "
            f"(execute={args.execute}, pick=({args.pick_x:.3f}, {args.pick_y:.3f}, {args.pick_z:.3f}), "
            f"place=({args.place_x:.3f}, {args.place_y:.3f}, {args.place_z:.3f}))"
        )
        if not args.execute:
            self.get_logger().warning(
                "Plan-only mode does not propagate stage output states. "
                "For the full pick-place sequence, use --execute so each stage starts from the previous stage result."
            )
        self.get_logger().info(
            "Task stages: " + " -> ".join(stage.name for stage in task)
        )

        pick_object_pose = make_object_pose(args, args.pick_x, args.pick_y)
        place_object_pose = make_object_pose(args, args.place_x, args.place_y)
        object_dimensions = (args.object_size_x, args.object_size_y, args.object_size_z)

        for stage in task:
            if stage.notes:
                self.get_logger().info(f"{stage.name}: {'; '.join(stage.notes)}")

            if stage.kind == StageKind.GRIPPER:
                if args.execute and not self.send_gripper_goal(stage.name, stage.gripper_position, args):
                    return 1
                if not args.execute:
                    self.get_logger().info(
                        f"{stage.name}: plan-only, skipping gripper position={stage.gripper_position:.4f}"
                    )

            elif stage.kind == StageKind.SCENE:
                self.log_scene_stage(stage, args)

            elif stage.kind == StageKind.ARM:
                if not self.send_arm_goal(
                    stage.name,
                    stage.tcp_pose,
                    args,
                    collision_objects=make_collision_objects(
                        args,
                        stage,
                        pick_object_pose,
                        place_object_pose,
                        object_dimensions,
                    ),
                    attached_collision_objects=make_attached_objects(
                        args,
                        stage,
                        object_dimensions,
                    ),
                    allowed_collision_pairs=make_allowed_collision_pairs(args, stage),
                ):
                    return 1

            time.sleep(args.step_pause)

        self.get_logger().info("Pick-place task completed")
        return 0

    def send_arm_goal(
        self,
        name: str,
        tcp_target: Pose,
        args: argparse.Namespace,
        collision_objects: Optional[list[CollisionObject]] = None,
        attached_collision_objects: Optional[list] = None,
        allowed_collision_pairs: Optional[list[tuple[str, str]]] = None,
    ) -> bool:
        tool0_target = tcp_target_to_tool0_target(tcp_target)
        self.log_pose(f"{name} TCP target", tcp_target)
        self.log_pose(f"{name} tool0 target", tool0_target)

        if self.try_arm_goal(
            name,
            tool0_target,
            args,
            collision_objects or [],
            attached_collision_objects or [],
            allowed_collision_pairs or [],
            attempt_label="primary",
        ):
            return True

        if not args.retry_relaxed_constraints:
            return False

        relaxed_args = make_relaxed_retry_args(args)
        self.get_logger().warning(
            f"{name}: retrying with relaxed planning constraints "
            f"(planning_time={relaxed_args.planning_time:.1f}, attempts={relaxed_args.planning_attempts}, "
            f"position_tolerance={relaxed_args.position_tolerance:.3f}, "
            f"orientation_tolerance={relaxed_args.orientation_tolerance:.3f}, "
            f"shoulder_lift_guard={relaxed_args.shoulder_lift_guard})"
        )
        return self.try_arm_goal(
            name,
            tool0_target,
            relaxed_args,
            collision_objects or [],
            attached_collision_objects or [],
            allowed_collision_pairs or [],
            attempt_label="relaxed",
        )

    def try_arm_goal(
        self,
        name: str,
        tool0_target: Pose,
        args: argparse.Namespace,
        collision_objects: list[CollisionObject],
        attached_collision_objects: list,
        allowed_collision_pairs: list[tuple[str, str]],
        attempt_label: str,
    ) -> bool:
        goal_args = make_move_group_args(args)
        goal_args.collision_objects = collision_objects
        goal_args.attached_collision_objects = attached_collision_objects
        goal_args.allowed_collision_pairs = allowed_collision_pairs
        goal_msg = build_goal(goal_args, tool0_target)

        send_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{name}: MoveGroup goal rejected ({attempt_label})")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        error_code = result.error_code.val
        error_name = ERROR_CODE_NAMES.get(error_code, f"UNKNOWN_{error_code}")
        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(
                f"{name}: MoveGroup succeeded ({error_name}, {attempt_label}), "
                f"planning_time={result.planning_time:.3f}s"
            )
            return True

        self.get_logger().error(
            f"{name}: MoveGroup failed ({error_name}, {error_code}, {attempt_label})"
        )
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

    def log_scene_stage(self, stage: PickPlaceStage, args: argparse.Namespace) -> None:
        if not args.object_collision:
            self.get_logger().info(f"{stage.name}: planning-scene object disabled")
            return
        state = "attached" if stage.object_attached else "world" if stage.object_in_world else "removed"
        touch_state = "touch allowed" if stage.object_touch_allowed else "touch forbidden"
        self.get_logger().info(f"{stage.name}: planning-scene object state={state}, {touch_state}")

    def log_pose(self, label: str, pose: Pose) -> None:
        self.get_logger().info(
            "\n"
            f"{label}:\n"
            f"  position: x={pose.position.x:.6f}, y={pose.position.y:.6f}, z={pose.position.z:.6f}\n"
            "  orientation: "
            f"x={pose.orientation.x:.6f}, y={pose.orientation.y:.6f}, "
            f"z={pose.orientation.z:.6f}, w={pose.orientation.w:.6f}"
        )


def build_pick_place_task(args: argparse.Namespace) -> list[PickPlaceStage]:
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
    lift_pose = offset_pose_z(pick_pose, args.lift_height)
    retreat_pose = offset_pose_z(place_pose, args.retreat_height)

    return [
        PickPlaceStage(
            "current",
            StageKind.SCENE,
            notes=("captures the current robot state like MTC CurrentState",),
        ),
        PickPlaceStage("open hand", StageKind.GRIPPER, gripper_position=args.open_position),
        PickPlaceStage(
            "move to pick",
            StageKind.ARM,
            tcp_pose=pick_approach,
            notes=("Connect stage equivalent",),
        ),
        PickPlaceStage(
            "allow collision (hand,object)",
            StageKind.SCENE,
            object_in_world=True,
            object_touch_allowed=True,
            notes=("object touch links are allowed for the following grasp approach goals",),
        ),
        *make_vertical_motion(
            "approach object",
            pick_approach,
            pick_pose,
            args,
            final_name="grasp pose IK",
            object_touch_allowed=True,
            notes=("world -Z Cartesian approach",),
        ),
        PickPlaceStage("close hand", StageKind.GRIPPER, gripper_position=args.close_position),
        PickPlaceStage(
            "attach object",
            StageKind.SCENE,
            object_in_world=False,
            object_attached=True,
        ),
        PickPlaceStage(
            "lift object",
            StageKind.ARM,
            tcp_pose=lift_pose,
            object_in_world=False,
            object_attached=True,
            notes=("world +Z Cartesian lift",),
        ),
        PickPlaceStage(
            "move to place",
            StageKind.ARM,
            tcp_pose=place_approach,
            object_in_world=False,
            object_attached=True,
            notes=("Connect stage equivalent",),
        ),
        *make_vertical_motion(
            "generate place pose",
            place_approach,
            place_pose,
            args,
            final_name="place pose IK",
            object_in_world=False,
            object_attached=True,
            notes=("world -Z Cartesian place approach",),
        ),
        PickPlaceStage("open hand", StageKind.GRIPPER, gripper_position=args.open_position),
        PickPlaceStage(
            "forbid collision (hand,object)",
            StageKind.SCENE,
            object_in_world=True,
        ),
        PickPlaceStage("detach object", StageKind.SCENE, object_in_world=True),
        PickPlaceStage(
            "retreat",
            StageKind.ARM,
            tcp_pose=retreat_pose,
            object_in_world=True,
            notes=("world +Z retreat",),
        ),
        PickPlaceStage(
            "return home",
            StageKind.ARM,
            tcp_pose=pick_approach,
            object_in_world=True,
        ),
    ]


def make_vertical_motion(
    prefix: str,
    start_pose: Pose,
    end_pose: Pose,
    args: argparse.Namespace,
    final_name: str,
    object_in_world: bool = True,
    object_attached: bool = False,
    object_touch_allowed: bool = False,
    notes: tuple[str, ...] = (),
) -> list[PickPlaceStage]:
    distance = abs(start_pose.position.z - end_pose.position.z)
    if args.vertical_step_size > 0.0:
        steps = max(args.vertical_steps, int(distance / args.vertical_step_size) + 1)
    else:
        steps = max(1, args.vertical_steps)

    stages = []
    for index in range(1, steps + 1):
        ratio = index / steps
        z = start_pose.position.z + (end_pose.position.z - start_pose.position.z) * ratio
        name = final_name if index == steps else f"{prefix}_{index}"
        stages.append(
            PickPlaceStage(
                name,
                StageKind.ARM,
                tcp_pose=make_tcp_pose(
                    end_pose.position.x,
                    end_pose.position.y,
                    z,
                    end_pose.orientation.x,
                    end_pose.orientation.y,
                    end_pose.orientation.z,
                    end_pose.orientation.w,
                ),
                object_in_world=object_in_world,
                object_attached=object_attached,
                object_touch_allowed=object_touch_allowed,
                notes=notes if index == 1 else (),
            )
        )
    return stages


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


def make_object_pose(args: argparse.Namespace, x: float, y: float) -> Pose:
    object_z = args.object_z
    if object_z is None:
        object_z = args.pick_z - args.grasp_clearance - (args.object_size_z * 0.5)
    return make_tcp_pose(x, y, object_z, 0.0, 0.0, 0.0, 1.0)


def make_collision_objects(
    args: argparse.Namespace,
    stage: PickPlaceStage,
    pick_object_pose: Pose,
    place_object_pose: Pose,
    object_dimensions: tuple[float, float, float],
) -> list[CollisionObject]:
    if not args.object_collision:
        return []

    if stage.object_attached:
        return [
            make_box_collision_object(
                args.frame,
                args.object_id,
                pick_object_pose,
                object_dimensions,
                operation=CollisionObject.REMOVE,
            )
        ]

    object_pose = place_object_pose if is_after_detach(stage.name) else pick_object_pose
    return [
        make_box_collision_object(
            args.frame,
            args.object_id,
            object_pose,
            object_dimensions,
        )
    ]


def make_attached_objects(
    args: argparse.Namespace,
    stage: PickPlaceStage,
    object_dimensions: tuple[float, float, float],
) -> list:
    if not args.object_collision or not stage.object_attached:
        return []

    attached_object_pose = make_tcp_pose(
        0.0,
        0.0,
        args.attached_object_z_offset,
        0.0,
        0.0,
        0.0,
        1.0,
    )
    return [
        make_attached_box_collision_object(
            args.link,
            args.object_id,
            args.link,
            attached_object_pose,
            object_dimensions,
            ROBOTIQ_TOUCH_LINKS,
        )
    ]


def make_allowed_collision_pairs(
    args: argparse.Namespace,
    stage: PickPlaceStage,
) -> list[tuple[str, str]]:
    if not args.object_collision or not stage.object_touch_allowed:
        return []
    return [(args.object_id, link_name) for link_name in ROBOTIQ_TOUCH_LINKS]


def is_after_detach(stage_name: str) -> bool:
    return stage_name in {
        "forbid collision (hand,object)",
        "detach object",
        "retreat",
        "return home",
    }


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


def make_relaxed_retry_args(args: argparse.Namespace) -> argparse.Namespace:
    relaxed = argparse.Namespace(**vars(args))
    relaxed.planning_time = max(args.planning_time * 2.0, 8.0)
    relaxed.planning_attempts = max(args.planning_attempts * 2, 20)
    relaxed.position_tolerance = max(args.position_tolerance, 0.03)
    relaxed.orientation_tolerance = max(args.orientation_tolerance, 0.8)
    relaxed.shoulder_lift_guard = False
    relaxed.replan = True
    relaxed.replan_attempts = max(args.replan_attempts, 5)
    return relaxed


def parse_args(args: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="MTC-style pick and place sequence executed through MoveGroup and the Isaac bridge."
    )
    parser.add_argument("--pick-x", type=float, default=-0.30)
    parser.add_argument("--pick-y", type=float, default=0.30)
    parser.add_argument("--pick-z", type=float, default=0.065)
    parser.add_argument("--place-x", type=float, default=0.0)
    parser.add_argument("--place-y", type=float, default=0.45)
    parser.add_argument("--place-z", type=float, default=0.20)
    parser.add_argument("--approach-height", type=float, default=0.15)
    parser.add_argument("--lift-height", type=float, default=0.20)
    parser.add_argument("--retreat-height", type=float, default=0.20)
    parser.add_argument("--vertical-steps", type=int, default=6)
    parser.add_argument("--vertical-step-size", type=float, default=0.025)
    parser.add_argument("--min-tcp-z", type=float, default=0.0001)
    parser.add_argument("--allow-low-z", action="store_true")
    parser.add_argument("--qx", type=float, default=TOP_DOWN_TCP_QUATERNION[0])
    parser.add_argument("--qy", type=float, default=TOP_DOWN_TCP_QUATERNION[1])
    parser.add_argument("--qz", type=float, default=TOP_DOWN_TCP_QUATERNION[2])
    parser.add_argument("--qw", type=float, default=TOP_DOWN_TCP_QUATERNION[3])

    parser.add_argument("--object-id", default="object")
    parser.add_argument("--object-collision", action="store_true")
    parser.add_argument("--object-z", type=float, default=None)
    parser.add_argument("--grasp-clearance", type=float, default=0.010)
    parser.add_argument("--object-size-x", type=float, default=0.055)
    parser.add_argument("--object-size-y", type=float, default=0.055)
    parser.add_argument("--object-size-z", type=float, default=0.055)
    parser.add_argument(
        "--attached-object-z-offset",
        type=float,
        default=TOOL0_TO_ROBOTIQ_GRASPING_FRAME.translation[2],
    )

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
    parser.add_argument("--position-tolerance", type=float, default=0.015)
    parser.add_argument("--orientation-tolerance", type=float, default=0.35)
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
    parser.add_argument("--action-timeout", type=float, default=10.0)
    parser.add_argument("--step-pause", type=float, default=0.2)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--replan", action="store_true")
    parser.add_argument("--replan-attempts", type=int, default=3)
    parser.add_argument(
        "--no-relaxed-retry",
        dest="retry_relaxed_constraints",
        action="store_false",
        help="Disable the fallback retry that relaxes planning constraints after a MoveGroup failure.",
    )
    parser.set_defaults(retry_relaxed_constraints=True)
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
