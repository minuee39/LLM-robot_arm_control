import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def include_moveit_config_launch(file_name):
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
    moveit_config = (
        MoveItConfigsBuilder(
            "ur10e_robotiq_2f140",
            package_name="ur10e_robotiq_2f140_moveit_config",
        )
        .robot_description(file_path="config/ur10e_robotiq_2f140.urdf.xacro")
        .robot_description_semantic(file_path="config/ur10e_robotiq_2f140.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
        )
        .to_moveit_configs()
    )

    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability",
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("mtc_tutorial"),
        "launch",
        "mtc.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription(
        [
            include_moveit_config_launch("static_virtual_joint_tfs.launch.py"),
            include_moveit_config_launch("rsp.launch.py"),
            move_group_node,
            rviz_node,
        ]
    )
