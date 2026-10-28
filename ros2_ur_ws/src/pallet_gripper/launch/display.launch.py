"""Display the normalized Z-up robot from its base_footprint frame."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    package = Path(get_package_share_directory("pallet_gripper"))
    description = (package / "urdf/pallet_gripper.urdf").read_text(encoding="utf-8")
    actions = [
        LogInfo(msg=(
            "Inertial values are the verified Z-up transform of the archived "
            "SolidWorks export; this does not certify the CAD mass properties."
        )),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}], output="screen"),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             parameters=[{"robot_description": description}], output="screen"),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", str(package / "config/display.rviz")], output="screen"),
    ]
    return LaunchDescription(actions)
