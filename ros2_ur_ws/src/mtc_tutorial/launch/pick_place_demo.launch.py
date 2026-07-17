from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def float_launch_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


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
        .to_dict()
    )

    pick_place_demo = Node(
        package="mtc_tutorial",
        executable="mtc_tutorial",
        output="screen",
        parameters=[
            moveit_config,
            {
                "object_x": float_launch_param("object_x"),
                "object_y": float_launch_param("object_y"),
                "object_z": float_launch_param("object_z"),
                "object_height": float_launch_param("object_height"),
                "object_radius": float_launch_param("object_radius"),
                "place_x": float_launch_param("place_x"),
                "place_y": float_launch_param("place_y"),
                "place_z": float_launch_param("place_z"),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("object_x", default_value="0.70"),
            DeclareLaunchArgument("object_y", default_value="0.40"),
            DeclareLaunchArgument("object_z", default_value="0.05"),
            DeclareLaunchArgument("object_height", default_value="0.10"),
            DeclareLaunchArgument("object_radius", default_value="0.02"),
            DeclareLaunchArgument("place_x", default_value="0.30"),
            DeclareLaunchArgument("place_y", default_value="-0.30"),
            DeclareLaunchArgument("place_z", default_value="0.05"),
            pick_place_demo,
        ]
    )
