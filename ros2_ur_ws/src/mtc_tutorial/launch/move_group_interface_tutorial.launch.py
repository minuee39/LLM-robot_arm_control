from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "ur10e_robotiq_2f140",
            package_name="ur10e_robotiq_2f140_moveit_config",
        )
        .robot_description(file_path="config/ur10e_robotiq_2f140.urdf.xacro")
        .robot_description_semantic(file_path="config/ur10e_robotiq_2f140.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # MTC demo executable
    mtc_demo = Node(
        name="mtc_tutorial",
        package="mtc_tutorial",
        executable="mtc_tutorial",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([mtc_demo])
