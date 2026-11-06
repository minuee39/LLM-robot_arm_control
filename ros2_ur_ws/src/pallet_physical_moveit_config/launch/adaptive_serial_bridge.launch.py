"""MoveIt arm + ID7 adaptive gripper, one serial owner; dry-run by default."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,GroupAction,SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    root=Path(get_package_share_directory('pallet_physical_moveit_config'))
    pallet=Path(get_package_share_directory('pallet'))
    config=(MoveItConfigsBuilder('pallet',package_name='pallet_physical_moveit_config')
        .robot_description(file_path='config/pallet_adaptive_control.urdf')
        .robot_description_semantic(file_path='config/pallet_adaptive.srdf')
        .joint_limits(file_path='config/adaptive_joint_limits.yaml')
        .trajectory_execution(file_path='config/adaptive_moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl']).to_moveit_configs())
    library=Path(get_package_prefix('pallet_physical_moveit_config'))/'lib/pallet_physical_moveit_config/libpallet_moveit_shutdown_fix.so'
    if not library.is_file():raise RuntimeError('Build pallet_physical_moveit_config shutdown fix first')
    preload=str(library)+((':'+os.environ['LD_PRELOAD']) if os.environ.get('LD_PRELOAD') else '')
    return LaunchDescription([
        DeclareLaunchArgument('dry_run',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('observe_only',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('use_rviz',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('config_file',default_value=str(pallet/'config/serial_trajectory_dry_run.yaml')),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[config.robot_description]),
        Node(package='tf2_ros',executable='static_transform_publisher',arguments=['--frame-id','world','--child-frame-id','base_footprint']),
        Node(package='pallet',executable='pallet_serial_trajectory_bridge.py',output='screen',parameters=[{
            'adaptive_gripper':True,
            'dry_run':ParameterValue(LaunchConfiguration('dry_run'),value_type=bool),
            'observe_only':ParameterValue(LaunchConfiguration('observe_only'),value_type=bool),
            'config_file':LaunchConfiguration('config_file')}]),
        GroupAction([SetEnvironmentVariable('LD_PRELOAD',preload),Node(package='moveit_ros_move_group',executable='move_group',output='screen',parameters=[config.to_dict()])]),
        Node(package='pallet',executable='pallet_planning_scene_floor.py',output='screen'),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(root/'config/adaptive_moveit.rviz')],parameters=[config.to_dict()],condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
