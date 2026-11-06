#!/usr/bin/env python3
"""Dry-only MoveIt/bridge action smoke test; writes logs under /tmp."""
import os,subprocess,signal,time,yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory as F
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus
rclpy.init();n=Node('adaptive_integration_test');states=[]
n.create_subscription(JointState,'/joint_states',lambda m:states.append(dict(zip(m.name,m.position))),10)
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
config=yaml.safe_load((Path(get_package_share_directory('pallet'))/'config/serial_trajectory_dry_run.yaml').read_text());config['final_hold_s']=1.;config['hold_after_success']=False
open('/tmp/adaptive-test.yaml','w').write(yaml.safe_dump(config))
log=open('/tmp/adaptive-moveit-test.log','w')
p=subprocess.Popen(['ros2','launch','pallet_physical_moveit_config','adaptive_serial_bridge.launch.py','use_rviz:=false','dry_run:=true','config_file:=/tmp/adaptive-test.yaml'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
def spin(t):
 end=time.monotonic()+t
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
def send(group,names,target,cancel=False):
 client=ActionClient(n,F,'/pallet_'+group+'_controller/follow_joint_trajectory');assert client.wait_for_server(timeout_sec=20)
 spin(1)
 goal=F.Goal();goal.trajectory.joint_names=names
 goal.trajectory.points=[JointTrajectoryPoint(positions=[states[-1][j] for j in names]),JointTrajectoryPoint(positions=target)]
 goal.trajectory.points[1].time_from_start.sec=3
 future=client.send_goal_async(goal);rclpy.spin_until_future_complete(n,future,timeout_sec=12);h=future.result();assert h and h.accepted
 result=h.get_result_async()
 if cancel:
  spin(.4);c=h.cancel_goal_async();rclpy.spin_until_future_complete(n,c,timeout_sec=4)
 rclpy.spin_until_future_complete(n,result,timeout_sec=12);r=result.result();assert r is not None
 assert r.status==(GoalStatus.STATUS_CANCELED if cancel else GoalStatus.STATUS_SUCCEEDED),(r.status,r.result.error_string)
 client.destroy()
try:
 spin(5);assert states and len(states[-1])==22
 send('gripper',['gripper_motor'],[.2]);spin(.3)
 assert abs(states[-1]['gripper_motor']-.2)<1e-9
 assert abs(states[-1]['g2']-.0016957322966666665)<1e-5
 send('arm',[f'J{i}' for i in range(1,7)],[.01]*6)
 send('gripper',['gripper_motor'],[.4],cancel=True)
 from moveit_msgs.action import ExecuteTrajectory
 client=ActionClient(n,ExecuteTrajectory,'/execute_trajectory');assert client.wait_for_server(timeout_sec=10)
 goal=ExecuteTrajectory.Goal();goal.trajectory.joint_trajectory.joint_names=['gripper_motor']
 spin(1);start=states[-1]['gripper_motor']
 goal.trajectory.joint_trajectory.points=[JointTrajectoryPoint(positions=[start]),JointTrajectoryPoint(positions=[.3])]
 goal.trajectory.joint_trajectory.points[-1].time_from_start.sec=3
 f=client.send_goal_async(goal);rclpy.spin_until_future_complete(n,f,timeout_sec=10);h=f.result();assert h.accepted
 f=h.get_result_async();rclpy.spin_until_future_complete(n,f,timeout_sec=15)
 assert f.result() is not None and f.result().result.error_code.val==1,f.result()
 client.destroy()
 print('PASS: MoveIt ExecuteTrajectory -> gripper controller -> shared bridge')

 print('PASS: 22 joint state values, gripper action, arm action, derived nut motion, cancellation')
finally:
 os.killpg(p.pid,signal.SIGINT)
 try:p.wait(timeout=12)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 log.close();n.destroy_node();rclpy.shutdown()
