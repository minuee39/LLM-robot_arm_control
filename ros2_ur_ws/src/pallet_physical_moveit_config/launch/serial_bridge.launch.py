"""MoveIt -> six-axis serial bridge; DRY RUN by default, no fake arm controller."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    root=Path(get_package_share_directory('pallet_physical_moveit_config'))
    pallet=Path(get_package_share_directory('pallet'))
    def include(name):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'launch'/name)))
    return LaunchDescription([
        DeclareLaunchArgument('dry_run',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('observe_only',default_value='false',choices=['true','false']),
        # Virtual execution has its own envelope. LIVE continues to require an
        # explicitly supplied commissioned hardware configuration.
        DeclareLaunchArgument('config_file',default_value=str(pallet/'config/serial_trajectory_dry_run.yaml')),
        DeclareLaunchArgument('use_rviz',default_value='true',choices=['true','false']),
        include('rsp.launch.py'),include('static_virtual_joint_tfs.launch.py'),
        Node(package='pallet',executable='pallet_serial_trajectory_bridge.py',output='screen',parameters=[{
            'dry_run':ParameterValue(LaunchConfiguration('dry_run'),value_type=bool),
            'observe_only':ParameterValue(LaunchConfiguration('observe_only'),value_type=bool),
            'config_file':LaunchConfiguration('config_file')}]),
        include('move_group.launch.py'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'launch/moveit_rviz.launch.py')),
                                 condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
