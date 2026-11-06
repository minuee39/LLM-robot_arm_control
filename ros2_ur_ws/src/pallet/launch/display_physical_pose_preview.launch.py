"""Photo-based approximate pose preview. No hardware access."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    root=Path(get_package_share_directory('pallet'))
    description=(root/'urdf/pallet_with_gripper.urdf').read_text()
    zeros=yaml.safe_load((root/'config/physical_pose_preview.yaml').read_text())['joint_positions_rad']
    return LaunchDescription([
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':description}],output='screen'),
        Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui',parameters=[{'robot_description':description,'zeros':zeros}],output='screen'),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(root/'config/display.rviz'),'-f','base_footprint'],output='screen'),
    ])
