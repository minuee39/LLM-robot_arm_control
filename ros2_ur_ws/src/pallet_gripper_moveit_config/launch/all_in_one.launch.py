"""Start Isaac, wait for feedback, then MoveGroup, and finally RViz."""
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE = 'pallet_gripper_moveit_config'


def generate_launch_description():
    share = Path(get_package_share_directory(PACKAGE))
    executable_dir = Path(get_package_prefix(PACKAGE)) / 'lib' / PACKAGE
    headless = LaunchConfiguration('headless')
    startup_timeout = LaunchConfiguration('startup_timeout')

    def include(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / name))
        )

    isaac_gui = ExecuteProcess(
        cmd=[str(executable_dir / 'run_isaac.sh')],
        output='screen',
        condition=UnlessCondition(headless),
    )
    isaac_headless = ExecuteProcess(
        cmd=[str(executable_dir / 'run_isaac.sh'), '--headless'],
        output='screen',
        condition=IfCondition(headless),
    )
    isaac_gate = Node(
        package=PACKAGE,
        executable='startup_gate.py',
        arguments=['isaac', '--timeout', startup_timeout],
        output='screen',
    )
    move_group_gate = Node(
        package=PACKAGE,
        executable='startup_gate.py',
        arguments=['move_group', '--timeout', startup_timeout],
        output='screen',
    )

    moveit_core = [
        LogInfo(msg='Isaac feedback ready; starting MoveIt core.'),
        include('static_virtual_joint_tfs.launch.py'),
        include('rsp.launch.py'),
        Node(
            package=PACKAGE,
            executable='isaac_action_bridge.py',
            output='screen',
        ),
        include('move_group.launch.py'),
        Node(
            package=PACKAGE,
            executable='workcell_scene.py',
            output='screen',
        ),
        move_group_gate,
    ]
    rviz = [
        LogInfo(msg='MoveGroup Planning Scene ready; starting RViz.'),
        include('moveit_rviz.launch.py'),
    ]

    def continue_or_stop(actions, stage):
        def callback(event, _context):
            if event.returncode == 0:
                return actions
            return [
                LogInfo(msg=f'{stage} readiness check failed; shutting down.'),
                EmitEvent(event=Shutdown(reason=f'{stage} startup failed')),
            ]
        return callback

    def stop_when_isaac_exits(event, _context):
        return [EmitEvent(event=Shutdown(reason=f'Isaac exited with code {event.returncode}'))]

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Isaac Sim without its GUI; RViz is still shown.',
        ),
        DeclareLaunchArgument(
            'startup_timeout', default_value='180',
            description='Maximum seconds for each readiness stage.',
        ),
        RegisterEventHandler(OnProcessExit(
            target_action=isaac_gate,
            on_exit=continue_or_stop(moveit_core, 'Isaac'),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=move_group_gate,
            on_exit=continue_or_stop(rviz, 'MoveGroup'),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=isaac_gui,
            on_exit=stop_when_isaac_exits,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=isaac_headless,
            on_exit=stop_when_isaac_exits,
        )),
        LogInfo(msg='Starting Isaac Sim and waiting for measured joint feedback.'),
        isaac_gui,
        isaac_headless,
        isaac_gate,
    ])
