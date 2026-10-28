"""Launch MoveIt/RViz with the RobStride FollowJointTrajectory endpoint."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def include_moveit_launch(filename: str) -> IncludeLaunchDescription:
    path = Path(get_package_share_directory("pallet_moveit_config")) / "launch" / filename
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)))


def generate_launch_description() -> LaunchDescription:
    default_config = Path(get_package_share_directory("pallet")) / "config" / "robstride_hardware.yaml"
    return LaunchDescription([
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="Keep CAN disabled and simulate measured joint positions",
        ),
        DeclareLaunchArgument(
            "can_interface", default_value="can0", description="SocketCAN interface"
        ),
        DeclareLaunchArgument(
            "allow_live_direct_goal", default_value="false",
            description="Explicitly allow final-target-only PP motion on live motors",
        ),
        DeclareLaunchArgument(
            "hardware_config",
            default_value=str(default_config),
            description="Commissioned RobStride joint mapping and safety limits",
        ),
        include_moveit_launch("static_virtual_joint_tfs.launch.py"),
        include_moveit_launch("rsp.launch.py"),
        Node(
            package="pallet",
            executable="pallet_robstride_trajectory_bridge.py",
            output="screen",
            parameters=[{
                "dry_run": ParameterValue(
                    LaunchConfiguration("dry_run"), value_type=bool
                ),
                "can_interface": LaunchConfiguration("can_interface"),
                "allow_live_direct_goal": ParameterValue(
                    LaunchConfiguration("allow_live_direct_goal"), value_type=bool
                ),
                "config_file": LaunchConfiguration("hardware_config"),
            }],
        ),
        include_moveit_launch("move_group.launch.py"),
        Node(package="pallet", executable="pallet_planning_scene_floor.py", output="screen"),
        include_moveit_launch("moveit_rviz.launch.py"),
    ])
