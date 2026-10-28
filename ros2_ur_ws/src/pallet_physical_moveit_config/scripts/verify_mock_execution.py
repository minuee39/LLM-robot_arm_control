#!/usr/bin/env python3
"""Launch an isolated GenericSystem and check planning, execution and cancellation.
Never connects to CAN/serial. Uses a private ROS domain to avoid live controllers.
"""
import json
import os
import re
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    # Set before importing DDS/rclpy. Every child uses this isolated domain.
    os.environ['ROS_DOMAIN_ID'] = '87'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    folder = Path(tempfile.mkdtemp(prefix='pallet_mock_verify_'))
    os.environ['ROS_HOME'] = str(folder / 'ros')
    os.environ['ROS_LOG_DIR'] = str(folder / 'logs')
    import rclpy
    from rclpy.action import ActionClient
    from action_msgs.msg import GoalStatus
    from control_msgs.action import FollowJointTrajectory
    from moveit_msgs.action import ExecuteTrajectory
    from moveit_msgs.msg import Constraints, JointConstraint
    from moveit_msgs.srv import GetMotionPlan
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration

    report = {'log_directory': str(folder), 'hardware': 'GenericSystem', 'domain': 87}
    log = (folder / 'launch.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', 'pallet_physical_moveit_config',
                                'demo.launch.py', 'use_rviz:=false'],
                               stdout=log, stderr=log, start_new_session=True)
    node = None
    try:
        rclpy.init()
        node = rclpy.create_node('verify_mock_execution')
        positions = {}
        def receive(msg):
            positions.update(zip(msg.name, msg.position))
        subscription = node.create_subscription(JointState, '/joint_states', receive, 10)
        def wait(future, timeout=15):
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            if not future.done():
                raise TimeoutError('ROS operation timed out')
            return future.result()
        def feedback(target, tolerance=0.002):
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if all(n in positions and abs(positions[n] - v) < tolerance for n, v in target.items()):
                    return
            raise AssertionError(f'Feedback did not reach {target}; observed {positions}')
        planner = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        if not planner.wait_for_service(timeout_sec=35):
            raise TimeoutError('MoveIt planning service unavailable')
        launch_text = (folder / 'launch.log').read_text()
        match = re.search(r'\[move_group-\d+\]: process started with pid \[(\d+)\]', launch_text)
        assert match, 'move_group PID unavailable'
        report['shutdown_fix_loaded'] = 'libpallet_moveit_shutdown_fix.so' in Path(f'/proc/{match[1]}/maps').read_text()
        assert report['shutdown_fix_loaded'], 'Shutdown fix is not mapped in move_group'
        request = GetMotionPlan.Request()
        m = request.motion_plan_request
        m.group_name = 'pallet_arm'
        m.pipeline_id = 'ompl'
        m.allowed_planning_time = 3.
        m.start_state.is_diff = True
        m.max_velocity_scaling_factor = 0.1
        m.max_acceleration_scaling_factor = 0.1
        constraints = Constraints()
        for i in range(1, 7):
            constraints.joint_constraints.append(JointConstraint(
                joint_name=f'J{i}', position=0.02 if i == 1 else 0.,
                tolerance_above=0.0001, tolerance_below=0.0001, weight=1.))
        m.goal_constraints = [constraints]
        plan = wait(planner.call_async(request)).motion_plan_response
        assert plan.error_code.val == 1, f'Planning failed: {plan.error_code.val}'
        report['planned_points'] = len(plan.trajectory.joint_trajectory.points)
        execute = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
        assert execute.wait_for_server(timeout_sec=10)
        goal = ExecuteTrajectory.Goal(trajectory=plan.trajectory)
        handle = wait(execute.send_goal_async(goal))
        assert handle.accepted
        result = wait(handle.get_result_async(), 20)
        assert result.status == GoalStatus.STATUS_SUCCEEDED
        assert result.result.error_code.val == 1
        feedback({f'J{i}': 0.02 if i == 1 else 0. for i in range(1, 7)})
        report['arm_execution'] = 'passed'
        gripper = ActionClient(node, FollowJointTrajectory, '/pallet_gripper_controller/follow_joint_trajectory')
        assert gripper.wait_for_server(timeout_sec=5)
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = ['g_r']
        g.trajectory.points = [JointTrajectoryPoint(positions=[-0.02], time_from_start=Duration(sec=2))]
        h = wait(gripper.send_goal_async(g)); assert h.accepted
        r = wait(h.get_result_async()); assert r.status == GoalStatus.STATUS_SUCCEEDED and r.result.error_code == 0
        # Mimic is applied by robot_state_publisher, not independently measured.
        feedback({'g_r': -0.02})
        report['gripper_execution'] = 'passed'
        arm = ActionClient(node, FollowJointTrajectory, '/pallet_arm_controller/follow_joint_trajectory')
        assert arm.wait_for_server(timeout_sec=5)
        g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = [f'J{i}' for i in range(1, 7)]
        g.trajectory.points = [JointTrajectoryPoint(positions=[0.1, 0., 0., 0., 0., 0.], time_from_start=Duration(sec=10))]
        h = wait(arm.send_goal_async(g)); assert h.accepted
        # Wait for actual progress before requesting cancellation.
        deadline = time.monotonic() + 3
        while positions.get('J1', 0.) < 0.022 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert positions.get('J1', 0.) >= 0.022
        cancel = wait(h.cancel_goal_async()); assert cancel.goals_canceling
        r = wait(h.get_result_async()); assert r.status == GoalStatus.STATUS_CANCELED
        for _ in range(5): rclpy.spin_once(node, timeout_sec=0.05)
        stopped = positions['J1']
        until = time.monotonic() + 0.5
        while time.monotonic() < until: rclpy.spin_once(node, timeout_sec=0.05)
        assert abs(positions['J1'] - stopped) < 0.002
        assert positions['J1'] < 0.1
        report['controller_cancel_and_hold'] = 'passed'
    except Exception as error:
        report['failure'] = str(error)
        raise
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        # Signal the launcher once; it forwards SIGINT to its children.
        process.send_signal(signal.SIGINT)
        try: process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            report['shutdown_timeout'] = True
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        log.close()
        text = (folder / 'launch.log').read_text()
        report['move_group_clean_exit'] = bool(re.search(r'\[move_group-\d+\]: process has finished cleanly', text))
        report['shutdown_segfault'] = 'Segmentation fault' in text or 'exit code -11' in text
        (folder / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)

    if report.get('shutdown_segfault') or report.get('shutdown_timeout') or not report.get('move_group_clean_exit'):
        raise RuntimeError('Execution checks passed but shutdown failed; see result.json')


if __name__ == '__main__':
    main()
