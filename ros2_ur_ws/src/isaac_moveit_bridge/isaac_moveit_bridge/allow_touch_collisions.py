import argparse
import copy

import rclpy
from moveit_msgs.msg import AllowedCollisionEntry, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node


DEFAULT_OBJECT_NAMES = ["object", "red_block", "blue_block", "green_block"]

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


def make_entry() -> AllowedCollisionEntry:
    return AllowedCollisionEntry()


class TouchCollisionApplier(Node):
    def __init__(self, timeout: float) -> None:
        super().__init__("allow_touch_collisions")
        self.timeout = timeout
        self.get_scene_client = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.apply_scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def wait_for_services(self) -> bool:
        if not self.get_scene_client.wait_for_service(timeout_sec=self.timeout):
            self.get_logger().error("Timed out waiting for /get_planning_scene")
            return False
        if not self.apply_scene_client.wait_for_service(timeout_sec=self.timeout):
            self.get_logger().error("Timed out waiting for /apply_planning_scene")
            return False
        return True

    def get_allowed_collision_matrix(self):
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self.get_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError("failed to fetch full ACM from /get_planning_scene")

        matrix = future.result().scene.allowed_collision_matrix
        if not matrix.entry_names:
            raise RuntimeError("/get_planning_scene returned an empty ACM")
        return matrix

    def apply(self, object_names: list[str], link_names: list[str]) -> None:
        matrix = copy.deepcopy(self.get_allowed_collision_matrix())
        for name in object_names + link_names:
            if name not in matrix.entry_names:
                matrix.entry_names.append(name)

        matrix.entry_values = list(matrix.entry_values)
        while len(matrix.entry_values) < len(matrix.entry_names):
            matrix.entry_values.append(make_entry())

        for entry in matrix.entry_values:
            entry.enabled = list(entry.enabled)
            if len(entry.enabled) < len(matrix.entry_names):
                entry.enabled.extend([False] * (len(matrix.entry_names) - len(entry.enabled)))

        name_to_index = {name: index for index, name in enumerate(matrix.entry_names)}
        for object_name in object_names:
            object_index = name_to_index[object_name]
            for link_name in link_names:
                link_index = name_to_index[link_name]
                matrix.entry_values[object_index].enabled[link_index] = True
                matrix.entry_values[link_index].enabled[object_index] = True

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.allowed_collision_matrix = matrix

        request = ApplyPlanningScene.Request()
        request.scene = planning_scene
        future = self.apply_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError("failed to apply touch ACM to /apply_planning_scene")
        if not future.result().success:
            raise RuntimeError("/apply_planning_scene returned success=false while applying touch ACM")

        self.get_logger().info(
            f"Allowed touch collisions for {len(object_names)} object(s) and {len(link_names)} gripper link(s)"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Allow object-to-gripper touch collisions in MoveIt ACM.")
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--objects", nargs="+", default=DEFAULT_OBJECT_NAMES)
    parser.add_argument("--links", nargs="+", default=ROBOTIQ_TOUCH_LINKS)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = TouchCollisionApplier(args.timeout)
    try:
        if not node.wait_for_services():
            return 1
        node.apply(args.objects, args.links)
        return 0
    except Exception as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
