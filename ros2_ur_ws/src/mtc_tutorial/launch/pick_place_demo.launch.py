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
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
        )
        .to_dict()
    )

    pick_place_demo = Node(
        package="mtc_tutorial",
        executable="mtc_tutorial",
        output="screen",
        parameters=[
            moveit_config,
            {
                "object_id": LaunchConfiguration("object_id"),
                "planner_id": LaunchConfiguration("planner_id"),
                "object_x": float_launch_param("object_x"),
                "object_y": float_launch_param("object_y"),
                "object_z": float_launch_param("object_z"),
                "object_size_x": float_launch_param("object_size_x"),
                "object_size_y": float_launch_param("object_size_y"),
                "object_size_z": float_launch_param("object_size_z"),
                "place_x": float_launch_param("place_x"),
                "place_y": float_launch_param("place_y"),
                "place_z": float_launch_param("place_z"),
                "max_solutions": float_launch_param("max_solutions"),
                "move_to_pick_timeout": float_launch_param("move_to_pick_timeout"),
                "move_to_pick_max_path_length": float_launch_param("move_to_pick_max_path_length"),
                "move_to_place_timeout": float_launch_param("move_to_place_timeout"),
                "return_home_timeout": float_launch_param("return_home_timeout"),
                "gripper_close_min": float_launch_param("gripper_close_min"),
                "gripper_close_max": float_launch_param("gripper_close_max"),
                "gripper_close_step": float_launch_param("gripper_close_step"),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("object_id", default_value="red_block"),
            DeclareLaunchArgument(
                "planner_id",
                default_value="RRTConnectkConfigDefault",
                description="OMPL planner configuration used for sampled arm motions.",
            ),
            DeclareLaunchArgument("object_x", default_value="-0.30"),
            DeclareLaunchArgument("object_y", default_value="0.30"),
            DeclareLaunchArgument("object_z", default_value="0.05"),
            DeclareLaunchArgument("object_size_x", default_value="0.10"),
            DeclareLaunchArgument("object_size_y", default_value="0.0515"),
            DeclareLaunchArgument("object_size_z", default_value="0.10"),
            DeclareLaunchArgument("place_x", default_value="0.30"),
            DeclareLaunchArgument("place_y", default_value="-0.30"),
            DeclareLaunchArgument("place_z", default_value="0.05"),
            DeclareLaunchArgument(
                "max_solutions",
                default_value="3",
                description="Number of task solutions required before execution.",
            ),
            DeclareLaunchArgument("move_to_pick_timeout", default_value="5.0"),
            DeclareLaunchArgument("move_to_pick_max_path_length", default_value="8.0"),
            DeclareLaunchArgument("move_to_place_timeout", default_value="6.0"),
            DeclareLaunchArgument("return_home_timeout", default_value="1.0"),
            DeclareLaunchArgument("gripper_close_min", default_value="0.02"),
            DeclareLaunchArgument("gripper_close_max", default_value="0.45"),
            DeclareLaunchArgument("gripper_close_step", default_value="0.08"),
            pick_place_demo,
        ]
    )
