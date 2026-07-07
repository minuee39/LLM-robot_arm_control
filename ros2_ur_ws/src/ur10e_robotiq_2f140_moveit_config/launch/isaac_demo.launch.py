from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include_launch_file(file_name):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur10e_robotiq_2f140_moveit_config"),
                    "launch",
                    file_name,
                ]
            )
        )
    )


def generate_launch_description():
    return LaunchDescription(
        [
            include_launch_file("static_virtual_joint_tfs.launch.py"),
            include_launch_file("rsp.launch.py"),
            include_launch_file("move_group.launch.py"),
            include_launch_file("moveit_rviz.launch.py"),
        ]
    )
