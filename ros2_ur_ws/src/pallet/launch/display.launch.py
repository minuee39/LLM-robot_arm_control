"""Display the mass-revised model; not a hardware or dynamics launch."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    share = Path(get_package_share_directory('pallet'))
    model = share / 'urdf' / 'pallet.urdf'
    description = model.read_text(encoding='utf-8')
    robot = ET.fromstring(description)
    srdf = ET.parse(LaunchConfiguration('srdf').perform(context)).getroot()
    home = srdf.find("group_state[@name='home'][@group='pallet_arm']")
    if home is None:
        raise ValueError('SRDF must define pallet_arm/home')
    joints = {j.get('name'): j for j in robot.findall('joint')}
    zeros = {}
    for entry in home.findall('joint'):
        name, value = entry.get('name'), float(entry.get('value'))
        if name not in joints:
            raise ValueError(f'SRDF home joint missing in model: {name}')
        limit = joints[name].find('limit')
        if limit is None or not float(limit.get('lower')) <= value <= float(limit.get('upper')):
            raise ValueError(f'SRDF home outside joint limits: {name}={value}')
        zeros[name] = value
    parameters = {'robot_description': description, 'zeros': zeros}
    return [
        LogInfo(msg='MASS-REVISED PREVIEW ONLY: base inertial frame is provisional; no.2/no.3 source tensors fail physical inertia checks. Not certified for dynamics/hardware.'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             parameters=[parameters], condition=IfCondition(LaunchConfiguration('gui')), output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[parameters], condition=UnlessCondition(LaunchConfiguration('gui')), output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'config' / 'display.rviz'), '-f', 'base_footprint'],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('pallet'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('srdf', default_value=str(share / 'config' / 'pallet.srdf')),
        OpaqueFunction(function=_nodes),
    ])
