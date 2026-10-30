"""Latest CAD + read-only motor feedback + planning-only MoveIt; no controllers."""
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pallet = Path(get_package_share_directory('pallet'))
    package = Path(get_package_share_directory('pallet_physical_moveit_config'))
    config = (MoveItConfigsBuilder('pallet', package_name='pallet_physical_moveit_config')
              .planning_pipelines(pipelines=['ompl']).to_moveit_configs())
    for dependency in ('moveit_ros_planning', 'moveit_ros_move_group', 'moveit_simple_controller_manager'):
        installed = Path(get_package_share_directory(dependency)) / 'package.xml'
        if ET.parse(installed).getroot().findtext('version') != '2.12.4':
            raise RuntimeError(f'Shutdown fix requires {dependency} 2.12.4; review and rebuild before use')
    library = Path(get_package_prefix('pallet_physical_moveit_config')) / 'lib/pallet_physical_moveit_config/libpallet_moveit_shutdown_fix.so'
    if not library.is_file():
        raise RuntimeError(f'Missing shutdown fix library: {library}; rebuild the package')
    preload = str(library)
    if os.environ.get('LD_PRELOAD'):
        preload += ':' + os.environ['LD_PRELOAD']
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='manual', choices=['manual', 'can'],
                              description='manual: inject raw motor angles; can: read hardware via selected transport'),
        DeclareLaunchArgument('transport', default_value='serial', choices=['serial', 'socketcan']),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baud', default_value='921600'),
        DeclareLaunchArgument('config_file', default_value=str(pallet / 'config/hand_guiding.yaml')),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[config.robot_description], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'world', '--child-frame-id', 'base_footprint']),
        Node(package='pallet', executable='pallet_motor_joint_state.py', parameters=[{
            'mode': LaunchConfiguration('mode'),
            'transport': LaunchConfiguration('transport'),
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baud': ParameterValue(LaunchConfiguration('serial_baud'), value_type=int),
            'config_file': LaunchConfiguration('config_file'),
            'urdf_file': str(pallet / 'urdf/pallet_with_gripper.urdf'),
            # ID7 calibration is incomplete. Open pose is a display placeholder.
            'include_gripper': False,
        }], output='screen'),
        Node(package='moveit_ros_move_group', executable='move_group',
             additional_env={'LD_PRELOAD': preload}, parameters=[
            config.to_dict(),
            {'allow_trajectory_execution': False,
             'disable_capabilities': 'move_group/MoveGroupExecuteTrajectoryAction',
             'publish_robot_description_semantic': True,
             'publish_planning_scene': True,
             'publish_geometry_updates': True,
             'publish_state_updates': True,
             'publish_transforms_updates': True},
        ], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(package / 'config/moveit.rviz')],
             parameters=[config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics, config.joint_limits],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
