"""Launch MoveIt2/RViz against the torque-limited Isaac Sim Pallet endpoint."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def include_moveit_launch(filename: str) -> IncludeLaunchDescription:
    launch_path = (
        Path(get_package_share_directory("pallet_moveit_config")) / "launch" / filename
    )
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_path)))


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            include_moveit_launch("static_virtual_joint_tfs.launch.py"),
            include_moveit_launch("rsp.launch.py"),
            Node(
                package="pallet",
                executable="pallet_follow_joint_trajectory_bridge.py",
                name="pallet_follow_joint_trajectory_bridge",
                output="screen",
                parameters=[
                    {
                        "goal_tolerance": 0.08,
                        "settle_timeout": 2.0,
                    }
                ],
            ),
            include_moveit_launch("move_group.launch.py"),
            include_moveit_launch("moveit_rviz.launch.py"),
        ]
    )
