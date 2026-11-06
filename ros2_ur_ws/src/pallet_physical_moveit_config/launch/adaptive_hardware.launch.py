"""Physical arm + ID7 entry point; observe first, execute after commissioning."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    config = Path(get_package_share_directory('pallet_physical_moveit_config'))
    pallet = Path(get_package_share_directory('pallet'))
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='observe', choices=['observe', 'execute']),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(config / 'launch/adaptive_serial_bridge.launch.py')),
            launch_arguments={
                'dry_run': 'false',
                'observe_only': PythonExpression(["'true' if '", LaunchConfiguration('mode'), "' == 'observe' else 'false'"]),
                'config_file': str(pallet / 'config/serial_trajectory_live_tested.yaml'),
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items()),
    ])
