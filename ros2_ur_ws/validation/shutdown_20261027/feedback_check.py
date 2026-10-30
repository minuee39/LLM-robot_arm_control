import os,re,signal,subprocess,time,json
from pathlib import Path
os.environ.update(ROS_DOMAIN_ID='88',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_HOME='/tmp/pallet_feedback_ros',ROS_LOG_DIR='/tmp/pallet_feedback_logs')
import rclpy
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from std_msgs.msg import Float64MultiArray
root=Path('/home/minwoo/Development/LLM/ros2_ur_ws/validation/shutdown_20261027')
log=(root/'feedback.log').open('w')
p=subprocess.Popen(['ros2','launch','pallet_physical_moveit_config','feedback.launch.py','use_rviz:=false'],stdout=log,stderr=log,start_new_session=True)
result={}
try:
 rclpy.init(); n=rclpy.create_node('feedback_shutdown_check')
 c=n.create_client(GetPlanningScene,'/get_planning_scene')
 assert c.wait_for_service(timeout_sec=35)
 pub=n.create_publisher(Float64MultiArray,'/pallet_motor_positions',10)
 raw=[.01,.02,.03,.04,.05,.06];target=[-.01,-.02,.03,.04,-.05,-.06]
 for _ in range(20):pub.publish(Float64MultiArray(data=raw));rclpy.spin_once(n,timeout_sec=.05)
 request=GetPlanningScene.Request();request.components.components=PlanningSceneComponents.ROBOT_STATE
 f=c.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=5);assert f.done()
 js=f.result().scene.robot_state.joint_state;actual=dict(zip(js.name,js.position))
 assert all(abs(actual.get(f'J{i+1}',999)-v)<1e-6 for i,v in enumerate(target)),actual
 result['moveit_feedback_matches']=True
 n.destroy_node();rclpy.try_shutdown()
finally:
 p.send_signal(signal.SIGINT)
 try:p.wait(timeout=10)
 except subprocess.TimeoutExpired:
  result['shutdown_timeout']=True;p.send_signal(signal.SIGTERM);p.wait(timeout=5)
 log.close()
 text=(root/'feedback.log').read_text()
 result['move_group_clean_exit']=bool(re.search(r'\[move_group-\d+\]: process has finished cleanly',text))
 result['feedback_node_clean_exit']=bool(re.search(r'\[pallet_motor_joint_state.py-\d+\]: process has finished cleanly',text))
 result['segfault']='Segmentation fault' in text or 'exit code -11' in text
 (root/'feedback_result.json').write_text(json.dumps(result,indent=2))
 print(json.dumps(result,indent=2))
 assert result.get('move_group_clean_exit') and result.get('feedback_node_clean_exit') and not result.get('segfault') and not result.get('shutdown_timeout')
