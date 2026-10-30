"""Run Pallet MoveIt Task Constructor with mock ros2_control hardware."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            'pallet_gripper',
            package_name='pallet_gripper_moveit_config',
        )
        .robot_description(file_path='config/pallet_gripper.urdf.xacro')
        .robot_description_semantic(file_path='config/pallet_gripper.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
        .to_moveit_configs()
    )
    config_share = Path(get_package_share_directory('pallet_gripper_moveit_config'))

    def include(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(config_share / 'launch' / name))
        )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'capabilities': 'move_group/ExecuteTaskSolutionCapability'},
        ],
    )
    ros2_control = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            str(config_share / 'config/ros2_controllers.yaml'),
        ],
    )
    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager'],
            output='screen',
        )
        for name in (
            'joint_state_broadcaster',
            'pallet_arm_controller',
            'gripper_controller',
        )
    ]
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=[
            '-d',
            str(Path(get_package_share_directory('mtc_tutorial')) / 'launch/mtc.rviz'),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_rviz', default_value='true'),
            include('static_virtual_joint_tfs.launch.py'),
            include('rsp.launch.py'),
            move_group,
            ros2_control,
            *controller_spawners,
            rviz,
        ]
    )
