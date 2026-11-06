"""Isolated gripper preview; does not start any hardware nodes."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = Path(get_package_share_directory('pallet'))
    return LaunchDescription([
        DeclareLaunchArgument('motor_rad', default_value='0'),
        Node(package='pallet', executable='adaptive_gripper_preview.py',
             arguments=['--gui', '--motor-rad', LaunchConfiguration('motor_rad')], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': (package/'urdf/adaptive_gripper_preview.urdf').read_text(),
                          'frame_prefix': 'adaptive_preview/'}],
             remappings=[('joint_states', '/adaptive_gripper_preview/joint_states')]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(package/'config/adaptive_gripper/preview.rviz')]),
    ])
