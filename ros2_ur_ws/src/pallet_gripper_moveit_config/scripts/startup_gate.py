#!/usr/bin/env python3
"""Exit successfully only after an integration startup milestone is usable."""
import argparse
import math
import time

import rclpy
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


REQUIRED_JOINTS = {f'J{i}' for i in range(1, 7)} | {'J8'}


def wait_for_isaac(node, timeout):
    ready = False

    def feedback(msg):
        nonlocal ready
        values = dict(zip(msg.name, msg.position))
        ready = REQUIRED_JOINTS.issubset(values) and all(
            math.isfinite(values[name]) for name in REQUIRED_JOINTS
        )

    subscription = node.create_subscription(
        JointState, '/isaac_joint_states', feedback, qos_profile_sensor_data
    )
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not ready and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(subscription)
    if not ready:
        raise RuntimeError(f'No valid /isaac_joint_states within {timeout:g} seconds')
    node.get_logger().info('Isaac joint feedback is ready')


def wait_for_move_group(node, timeout):
    client = node.create_client(GetPlanningScene, '/get_planning_scene')
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=0.2):
            break
    else:
        raise RuntimeError(f'/get_planning_scene unavailable after {timeout:g} seconds')

    request = GetPlanningScene.Request()
    request.components.components = (
        PlanningSceneComponents.SCENE_SETTINGS | PlanningSceneComponents.ROBOT_STATE
    )
    future = client.call_async(request)
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not future.done() or future.result() is None:
        raise RuntimeError('Planning Scene service did not return a response')
    if future.result().scene.robot_model_name != 'pallet_gripper':
        raise RuntimeError(
            f"Unexpected robot model: {future.result().scene.robot_model_name!r}"
        )
    node.get_logger().info('MoveGroup Planning Scene is ready')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('stage', choices=('isaac', 'move_group'))
    parser.add_argument('--timeout', type=float, default=180.0)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = Node(f'pallet_gripper_{args.stage}_startup_gate')
    exit_code = 0
    try:
        if args.stage == 'isaac':
            wait_for_isaac(node, args.timeout)
        else:
            wait_for_move_group(node, args.timeout)
    except Exception as exc:
        node.get_logger().error(str(exc))
        exit_code = 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
