import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "ur10e_robotiq_2f140",
        package_name="ur10e_robotiq_2f140_moveit_config",
    ).to_moveit_configs()

    moveit_config_pkg = get_package_share_directory("ur10e_robotiq_2f140_moveit_config")
    launch_dir = os.path.join(moveit_config_pkg, "launch")

    return LaunchDescription(
        [
            DeclareBooleanLaunchArg(
                "use_rviz",
                default_value=True,
                description="Start RViz with the UR10e MoveIt config",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "static_virtual_joint_tfs.launch.py")
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "rsp.launch.py")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "move_group.launch.py")),
                launch_arguments={
                    "capabilities": "move_group/ExecuteTaskSolutionCapability",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "moveit_rviz.launch.py")),
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    moveit_config.robot_description,
                    os.path.join(moveit_config_pkg, "config", "ros2_controllers.yaml"),
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "spawn_controllers.launch.py")
                ),
            ),
        ]
    )
