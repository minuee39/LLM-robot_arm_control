"""Small plan-and-execute test; requires isaac_demo and Isaac endpoint running."""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

rclpy.init()
node = Node('pallet_moveit_execution_test')
state = {}
node.create_subscription(JointState, '/joint_states', lambda m: state.update(zip(m.name, m.position)), 10)
client = ActionClient(node, MoveGroup, '/move_action')
try:
    assert client.wait_for_server(timeout_sec=30)
    deadline = time.monotonic() + 90
    while not state and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    assert state, 'No current state'
    goal = MoveGroup.Goal()
    req = goal.request
    req.group_name = 'pallet_arm'
    req.allowed_planning_time = 5.
    req.num_planning_attempts = 3
    req.max_velocity_scaling_factor = .1
    req.max_acceleration_scaling_factor = .1
    req.start_state.is_diff = True
    constraints = Constraints()
    for name in ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']:
        joint = JointConstraint()
        joint.joint_name = name
        joint.position = state[name] + (.03 if name == 'J1' else 0.)
        joint.tolerance_above = joint.tolerance_below = .005
        joint.weight = 1.
        constraints.joint_constraints.append(joint)
    req.goal_constraints = [constraints]
    goal.planning_options.plan_only = False
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15)
    handle = future.result()
    assert handle and handle.accepted
    future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, future, timeout_sec=30)
    result = future.result()
    assert result and result.result.error_code.val == 1, str(result)
    print('PASS MoveIt OMPL + time parameterization + Isaac execution', flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
