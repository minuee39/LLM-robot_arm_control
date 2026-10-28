"""Read-only motor/manual input -> joint_states -> robot_state_publisher -> RViz."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    root = Path(get_package_share_directory('pallet'))
    urdf = root/'urdf/pallet_with_gripper.urdf'
    return LaunchDescription([
        DeclareLaunchArgument('gravity_preview',default_value='false',description='Read-only gravity diagnostic from same joint_states'),
        DeclareLaunchArgument('mode',default_value='manual',description='manual motor angles, can read-only, or joint_preview'),
        DeclareLaunchArgument('transport',default_value='socketcan'),
        DeclareLaunchArgument('serial_port',default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baud',default_value='921600'),
        DeclareLaunchArgument('include_gripper',default_value='false'),
        DeclareLaunchArgument('config_file',default_value=str(root/'config/hand_guiding.yaml')),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':urdf.read_text()}],output='screen'),
        Node(package='pallet',executable='pallet_motor_joint_state.py',parameters=[{'transport':LaunchConfiguration('transport'),'serial_port':LaunchConfiguration('serial_port'),'serial_baud':ParameterValue(LaunchConfiguration('serial_baud'),value_type=int),'mode':LaunchConfiguration('mode'),'include_gripper':ParameterValue(LaunchConfiguration('include_gripper'),value_type=bool),'config_file':LaunchConfiguration('config_file'),'urdf_file':str(urdf)}],output='screen'),
        Node(package='pallet',executable='pallet_gravity_preview.py',arguments=['--from-joint-states','--config',LaunchConfiguration('config_file'),'--urdf',str(urdf)],condition=IfCondition(LaunchConfiguration('gravity_preview')),output='screen'),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(root/'config/display.rviz'),'-f','base_footprint'],output='screen'),
    ])
