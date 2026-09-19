"""MoveIt connected to Isaac joint feedback (no mock hardware)."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(get_package_share_directory("pallet_gripper_moveit_config"))
    def include(name):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root / "launch" / name)))
    return LaunchDescription([
        include("static_virtual_joint_tfs.launch.py"),
        include("rsp.launch.py"),
        Node(package="pallet_gripper_moveit_config", executable="isaac_action_bridge.py",
             output="screen"),
        include("move_group.launch.py"),
        include("moveit_rviz.launch.py"),
    ])
