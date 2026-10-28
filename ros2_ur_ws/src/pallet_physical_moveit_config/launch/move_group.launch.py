import os
import xml.etree.ElementTree as ET
from pathlib import Path
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, SetEnvironmentVariable
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("pallet", package_name="pallet_physical_moveit_config").planning_pipelines(pipelines=["ompl"]).to_moveit_configs()
    for dependency in ('moveit_ros_planning', 'moveit_ros_move_group', 'moveit_simple_controller_manager'):
        installed = Path(get_package_share_directory(dependency)) / 'package.xml'
        if ET.parse(installed).getroot().findtext('version') != '2.12.4':
            raise RuntimeError(f'Shutdown fix requires {dependency} 2.12.4; review and rebuild before use')
    library = Path(get_package_prefix("pallet_physical_moveit_config")) / "lib/pallet_physical_moveit_config/libpallet_moveit_shutdown_fix.so"
    if not library.is_file():
        raise RuntimeError(f"Missing shutdown fix library: {library}; rebuild the package")
    preload = str(library)
    if os.environ.get("LD_PRELOAD"):
        preload += ":" + os.environ["LD_PRELOAD"]
    # Scope the plugin lifetime guard to move_group, never to controllers/RViz.
    return LaunchDescription([GroupAction(actions=[
        SetEnvironmentVariable("LD_PRELOAD", preload),
        *generate_move_group_launch(moveit_config).entities,
    ])])
