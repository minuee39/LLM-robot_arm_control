import argparse
import copy
import json
import time
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    CollisionObject,
    ObjectColor,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


DEFAULT_SCENE_STATE_FILE = "/tmp/ur10e_isaac_scene_objects.json"

ROBOTIQ_TOUCH_LINKS = [
    "robotiq_base_link",
    "robotiq_left_inner_finger",
    "robotiq_left_inner_finger_pad",
    "robotiq_left_inner_knuckle",
    "robotiq_left_outer_finger",
    "robotiq_left_outer_knuckle",
    "robotiq_right_inner_finger",
    "robotiq_right_inner_finger_pad",
    "robotiq_right_inner_knuckle",
    "robotiq_right_outer_finger",
    "robotiq_right_outer_knuckle",
]

DEFAULT_OBJECTS = [
    {
        "name": "red_block",
        "position": [-0.30, 0.30, 0.02575],
        "size": [0.10, 0.0515, 0.10],
        "color": [1.0, 0.0, 0.0, 1.0],
    },
    {
        "name": "blue_block",
        "position": [0.30, 0.30, 0.02575],
        "size": [0.10, 0.0515, 0.10],
        "color": [0.0, 0.0, 1.0, 1.0],
    },
    {
        "name": "green_block",
        "position": [0.00, 0.45, 0.02575],
        "size": [0.10, 0.0515, 0.10],
        "color": [0.0, 1.0, 0.0, 1.0],
    },
]


def make_pose(position: list[float]) -> Pose:
    pose = Pose()
    pose.position.x = float(position[0])
    pose.position.y = float(position[1])
    pose.position.z = float(position[2])
    pose.orientation.w = 1.0
    return pose


def make_box_collision_object(frame_id: str, item: dict[str, Any]) -> CollisionObject:
    collision_object = CollisionObject()
    collision_object.header.frame_id = frame_id
    collision_object.id = str(item["name"])
    collision_object.operation = CollisionObject.ADD

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [float(value) for value in item["size"]]
    collision_object.primitives.append(box)
    collision_object.primitive_poses.append(make_pose(item["position"]))
    return collision_object


def make_object_color(item: dict[str, Any]) -> ObjectColor:
    color_values = item.get("color", [0.8, 0.8, 0.8, 1.0])
    object_color = ObjectColor()
    object_color.id = str(item["name"])
    object_color.color.r = float(color_values[0])
    object_color.color.g = float(color_values[1])
    object_color.color.b = float(color_values[2])
    object_color.color.a = float(color_values[3]) if len(color_values) > 3 else 1.0
    return object_color


def _make_acm_entry() -> AllowedCollisionEntry:
    return AllowedCollisionEntry()


def read_scene_state(path: Path) -> list[dict[str, Any]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    objects = data.get("objects", data)
    if not isinstance(objects, list):
        raise ValueError("scene state JSON must contain an object list")
    return objects


class SceneCollisionPublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("isaac_scene_collision_publisher")
        self.args = args
        self.scene_state_file = Path(args.scene_state_file)
        self.last_mtime_ns: int | None = None
        self.pending_future = None
        self.pending_object_names: list[str] = []
        self.attached_object_ids: set[str] = set()
        self.latest_allowed_collision_matrix = None
        self.touch_acm_published = False
        self.client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.get_scene_client = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.create_subscription(PlanningScene, "/monitored_planning_scene", self.on_planning_scene, 10)

        self.get_logger().info("Waiting for /apply_planning_scene ...")
        self.client.wait_for_service()
        self.get_logger().info("Connected to /apply_planning_scene")
        self.get_logger().info("Waiting for /get_planning_scene ...")
        self.get_scene_client.wait_for_service()
        self.get_logger().info("Connected to /get_planning_scene")
        self.refresh_allowed_collision_matrix()

        if args.once:
            self.publish_current_scene(force=True)
        else:
            self.timer = self.create_timer(args.period, self.publish_current_scene)

    def on_planning_scene(self, planning_scene: PlanningScene) -> None:
        if planning_scene.allowed_collision_matrix.entry_names:
            had_allowed_collision_matrix = self.latest_allowed_collision_matrix is not None
            self.latest_allowed_collision_matrix = copy.deepcopy(planning_scene.allowed_collision_matrix)
            if not had_allowed_collision_matrix and not self.touch_acm_published:
                self.last_mtime_ns = None
                self.get_logger().info("Received full ACM; scheduling touch-collision scene refresh")

        if not planning_scene.is_diff or not planning_scene.robot_state.is_diff:
            self.attached_object_ids = {
                attached_object.object.id
                for attached_object in planning_scene.robot_state.attached_collision_objects
                if attached_object.object.id
            }
            return

        for attached_object in planning_scene.robot_state.attached_collision_objects:
            object_id = attached_object.object.id
            if not object_id:
                continue
            if attached_object.object.operation == CollisionObject.REMOVE:
                self.attached_object_ids.discard(object_id)
            else:
                self.attached_object_ids.add(object_id)

    def refresh_allowed_collision_matrix(self) -> bool:
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self.get_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.timeout)
        if not future.done():
            self.get_logger().warn("Timed out fetching full ACM from /get_planning_scene")
            return False

        result = future.result()
        if result is None:
            self.get_logger().warn("GetPlanningScene returned no result while fetching ACM")
            return False

        matrix = result.scene.allowed_collision_matrix
        if not matrix.entry_names:
            self.get_logger().warn("GetPlanningScene returned an empty ACM")
            return False

        self.latest_allowed_collision_matrix = copy.deepcopy(matrix)
        self.get_logger().info(f"Fetched full ACM with {len(matrix.entry_names)} entries")
        return True

    def allow_object_touch_collisions(self, planning_scene: PlanningScene, object_names: list[str]) -> None:
        if self.latest_allowed_collision_matrix is None:
            self.get_logger().warn("Cannot publish touch ACM because no full ACM is available")
            return

        matrix = copy.deepcopy(self.latest_allowed_collision_matrix)
        for name in object_names + ROBOTIQ_TOUCH_LINKS:
            if name not in matrix.entry_names:
                matrix.entry_names.append(name)

        matrix.entry_values = list(matrix.entry_values)
        while len(matrix.entry_values) < len(matrix.entry_names):
            matrix.entry_values.append(type(matrix.entry_values[0])() if matrix.entry_values else _make_acm_entry())

        for entry in matrix.entry_values:
            entry.enabled = list(entry.enabled)
            if len(entry.enabled) < len(matrix.entry_names):
                entry.enabled.extend([False] * (len(matrix.entry_names) - len(entry.enabled)))

        name_to_index = {name: index for index, name in enumerate(matrix.entry_names)}
        for object_name in object_names:
            object_index = name_to_index[object_name]
            for link_name in ROBOTIQ_TOUCH_LINKS:
                link_index = name_to_index[link_name]
                matrix.entry_values[object_index].enabled[link_index] = True
                matrix.entry_values[link_index].enabled[object_index] = True

        planning_scene.allowed_collision_matrix = matrix
        self.touch_acm_published = True

    def publish_current_scene(self, force: bool = False) -> None:
        try:
            if self.pending_future is not None:
                self.check_pending_result()
                if self.pending_future is not None:
                    return

            if self.scene_state_file.exists():
                stat = self.scene_state_file.stat()
                if not force and self.last_mtime_ns == stat.st_mtime_ns:
                    return
                self.last_mtime_ns = stat.st_mtime_ns
                objects = read_scene_state(self.scene_state_file)
            elif self.args.use_defaults:
                objects = DEFAULT_OBJECTS
            else:
                self.get_logger().warn(f"Scene state file not found: {self.scene_state_file}")
                return

            self.apply_scene(objects)
        except Exception as error:
            self.get_logger().error(f"Failed to publish scene collision objects: {error}")

    def check_pending_result(self) -> None:
        if self.pending_future is None or not self.pending_future.done():
            return

        object_names = self.pending_object_names
        try:
            result = self.pending_future.result()
            if result is None:
                self.get_logger().error("ApplyPlanningScene returned no result")
            elif not result.success:
                self.get_logger().error("ApplyPlanningScene returned success=false")
            else:
                acm_status = "with touch ACM" if self.touch_acm_published else "without touch ACM"
                self.get_logger().info(
                    f"Applied {len(object_names)} collision object(s) {acm_status}: {', '.join(object_names)}"
                )
        except Exception as error:
            self.get_logger().error(f"ApplyPlanningScene call failed: {error}")
        finally:
            self.pending_future = None
            self.pending_object_names = []

    def apply_scene(self, objects: list[dict[str, Any]]) -> None:
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.is_diff = True

        object_names = [str(item["name"]) for item in objects]
        attached_names = set(object_names) & self.attached_object_ids
        if attached_names:
            self.get_logger().info(
                "Skipping attached collision object(s): " + ", ".join(sorted(attached_names))
            )

        publishable_objects = [item for item in objects if str(item["name"]) not in self.attached_object_ids]
        object_names = [str(item["name"]) for item in publishable_objects]
        for object_name in object_names:
            remove_object = CollisionObject()
            remove_object.header.frame_id = self.args.frame
            remove_object.id = object_name
            remove_object.operation = CollisionObject.REMOVE
            planning_scene.world.collision_objects.append(remove_object)

        for item in publishable_objects:
            planning_scene.world.collision_objects.append(make_box_collision_object(self.args.frame, item))
            planning_scene.object_colors.append(make_object_color(item))

        self.allow_object_touch_collisions(planning_scene, object_names)

        request = ApplyPlanningScene.Request()
        request.scene = planning_scene
        self.pending_future = self.client.call_async(request)
        self.pending_object_names = object_names


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish Isaac scene objects as MoveIt collision objects.")
    parser.add_argument("--scene-state-file", default=DEFAULT_SCENE_STATE_FILE)
    parser.add_argument("--frame", default="world")
    parser.add_argument("--period", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--no-defaults", dest="use_defaults", action="store_false")
    parser.set_defaults(use_defaults=True)
    return parser.parse_args()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = SceneCollisionPublisher(parse_args())
        if node.args.once:
            deadline = time.monotonic() + node.args.timeout
            while rclpy.ok() and node.pending_future is not None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                node.check_pending_result()
            if node.pending_future is not None:
                node.get_logger().error("Timed out waiting for ApplyPlanningScene")
            return
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
