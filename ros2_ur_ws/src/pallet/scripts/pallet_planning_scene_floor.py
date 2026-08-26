#!/usr/bin/env python3
"""Add the Isaac floor to the MoveIt planning scene."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


FLOOR_TOP_Z = -0.005
FLOOR_THICKNESS = 0.10


def main() -> None:
    rclpy.init()
    node = Node("pallet_planning_scene_floor")
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    try:
        while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info("Waiting for /apply_planning_scene")

        floor = CollisionObject()
        floor.header.frame_id = "base_footprint"
        floor.id = "pallet_floor"
        floor.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [2.0, 2.0, FLOOR_THICKNESS]
        pose = Pose()
        pose.position.z = FLOOR_TOP_Z - FLOOR_THICKNESS / 2.0
        pose.orientation.w = 1.0
        floor.primitives = [box]
        floor.primitive_poses = [pose]

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [floor]
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("MoveIt rejected the Pallet floor planning scene")
        node.get_logger().info(
            f"Added MoveIt floor at z={FLOOR_TOP_Z:.3f} m in base_footprint"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
