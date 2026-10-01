from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('pallet_gripper_moveit_config'))
    workcell = yaml.safe_load((config / 'config/workcell.yaml').read_text())
    table = workcell['table']
    center_x, center_y = map(float, table['center_xy_world'])
    table_height = float(table['size'][2])
    # Keep MoveIt's world frame aligned with the Isaac table-mounted base.
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_pallet_table_mount",
            arguments=[
                "--x", str(center_x), "--y", str(center_y), "--z", str(table_height),
                "--qx", "0.0", "--qy", "0.0", "--qz", "0.0", "--qw", "1.0",
                "--frame-id", "world", "--child-frame-id", "base_footprint",
            ],
            output="screen",
        )
    ])
