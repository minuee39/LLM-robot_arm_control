"""Integration check against a running Isaac endpoint, on an isolated ROS domain."""
import threading
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from isaac_action_bridge import Bridge, ARM

rclpy.init()
bridge = Bridge()
client = Node('pallet_isaac_integration_test')
executor = MultiThreadedExecutor(num_threads=6)
executor.add_node(bridge)
executor.add_node(client)
thread = threading.Thread(target=executor.spin)
thread.start()

def wait(future):
    deadline = time.monotonic() + 30
    while not future.done() and time.monotonic() < deadline:
        time.sleep(.02)
    assert future.done(), 'Action timeout'
    return future.result()

try:
    deadline = time.monotonic() + 90
    while not bridge.positions and time.monotonic() < deadline:
        time.sleep(.1)
    assert bridge.positions, 'No Isaac feedback'
    arm = ActionClient(client, FollowJointTrajectory, '/pallet_arm_controller/follow_joint_trajectory')
    assert arm.wait_for_server(timeout_sec=5)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ARM
    point = JointTrajectoryPoint()
    point.positions = bridge.current(ARM)
    point.positions[0] += .03
    point.time_from_start.sec = 3
    goal.trajectory.points = [point]
    handle = wait(arm.send_goal_async(goal))
    assert handle.accepted
    result = wait(handle.get_result_async())
    assert result.status == 4, str(result)
    print('PASS Isaac arm measured target', bridge.current(ARM), flush=True)
    gripper = ActionClient(client, GripperCommand, '/gripper_controller/gripper_cmd')
    assert gripper.wait_for_server(timeout_sec=5)
    goal = GripperCommand.Goal()
    goal.command.position = .05
    handle = wait(gripper.send_goal_async(goal))
    assert handle.accepted
    result = wait(handle.get_result_async())
    assert result.status == 4 and result.result.reached_goal, str(result)
    print('PASS Isaac gripper measured target', bridge.current(['J8']), flush=True)
finally:
    executor.shutdown()
    thread.join()
    bridge.destroy_node()
    client.destroy_node()
    rclpy.shutdown()
