#!/usr/bin/env python3
"""Register the Isaac table and cup in the MoveIt planning scene."""

import argparse
import sys
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


def pose(x: float, y: float, z: float) -> Pose:
    result = Pose()
    result.position.x = x
    result.position.y = y
    result.position.z = z
    result.orientation.w = 1.0
    return result


def box(x: float, y: float, z: float) -> SolidPrimitive:
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [x, y, z]
    return primitive


def build_scene(include_cup: bool = True) -> PlanningScene:
    config = Path(get_package_share_directory('pallet_gripper_moveit_config'))
    workcell = yaml.safe_load((config / 'config/workcell.yaml').read_text())
    table_spec = workcell['table']
    cup_spec = workcell['cup']
    table = CollisionObject()
    table.header.frame_id = "world"
    table.id = "workcell_table"
    table.operation = CollisionObject.ADD

    center_x, center_y = map(float, table_spec['center_xy_world'])
    size_x, size_y, size_z = map(float, table_spec['size'])
    top_thickness = float(table_spec['top_thickness'])
    leg_size = float(table_spec['leg_size'])
    top_z = size_z
    leg_height = size_z - top_thickness
    table.primitives.append(box(size_x, size_y, top_thickness))
    table.primitive_poses.append(pose(center_x, center_y, top_z - top_thickness / 2.0))
    for sx, sy in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
        table.primitives.append(box(leg_size, leg_size, leg_height))
        table.primitive_poses.append(pose(
            center_x + sx * (size_x / 2.0 - leg_size / 2.0),
            center_y + sy * (size_y / 2.0 - leg_size / 2.0),
            leg_height / 2.0,
        ))

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [table]
    colors = [(table.id, (0.42, 0.24, 0.10))]
    if include_cup:
        cup = CollisionObject()
        cup.header.frame_id = "world"
        cup.id = "target_cup"
        cup.operation = CollisionObject.ADD
        cup_shape = SolidPrimitive()
        cup_shape.type = SolidPrimitive.CYLINDER
        cup_shape.dimensions = [float(cup_spec['height']), float(cup_spec['radius'])]
        cup.primitives.append(cup_shape)
        cup_x, cup_y = map(float, cup_spec['center_xy_world'])
        cup.primitive_poses.append(pose(cup_x, cup_y, top_z + cup_shape.dimensions[0] / 2.0))
        scene.world.collision_objects.append(cup)
        colors.append((cup.id, (0.12, 0.45, 0.85)))
    for object_id, rgb in colors:
        color = ObjectColor()
        color.id = object_id
        color.color.r, color.color.g, color.color.b, color.color.a = (*rgb, 1.0)
        scene.object_colors.append(color)
    return scene


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-cup', action='store_true')
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = Node("pallet_workcell_scene")
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not client.wait_for_service(timeout_sec=120.0):
        node.get_logger().error("MoveIt apply_planning_scene service did not become available")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    request = ApplyPlanningScene.Request()
    request.scene = build_scene(include_cup=not args.no_cup)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    response = future.result()
    if response is None or not response.success:
        node.get_logger().error("MoveIt rejected the table/cup planning scene")
        result = 1
    else:
        node.get_logger().info("Registered workcell table%s in MoveIt" % (
            " and target cup" if not args.no_cup else "",
        ))
        result = 0
    node.destroy_node()
    rclpy.shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
