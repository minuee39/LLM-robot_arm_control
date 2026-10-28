"""Start the Pallet Isaac workcell and MoveGroup for direct-pose MTC runs."""

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
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


PACKAGE = 'pallet_gripper_moveit_config'


def generate_launch_description():
    config_share = Path(get_package_share_directory(PACKAGE))
    executable_dir = Path(get_package_prefix(PACKAGE)) / 'lib' / PACKAGE
    headless = LaunchConfiguration('headless')
    startup_timeout = LaunchConfiguration('startup_timeout')
    safety_cancel_enabled = LaunchConfiguration('safety_cancel_enabled')
    fault_injection_enabled = LaunchConfiguration('fault_injection_enabled')
    block_args = [
        '--block-center-xy', LaunchConfiguration('block_x'), LaunchConfiguration('block_y'),
        '--block-size', LaunchConfiguration('block_size_x'), LaunchConfiguration('block_size_y'),
        LaunchConfiguration('block_size_z'), '--block-mass-kg', LaunchConfiguration('block_mass_kg'),
        '--fault-injection-enabled', fault_injection_enabled,
    ]

    def include(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(config_share / 'launch' / name))
        )

    moveit_config = (
        MoveItConfigsBuilder('pallet_gripper', package_name=PACKAGE)
        .robot_description(file_path='config/pallet_gripper.urdf.xacro')
        .robot_description_semantic(file_path='config/pallet_gripper.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
        .to_moveit_configs()
    )

    isaac_gui = ExecuteProcess(
        cmd=[str(executable_dir / 'run_isaac.sh'), '--calibration-block', '--no-cup', '--rgbd-camera', *block_args],
        output='screen',
        condition=UnlessCondition(headless),
    )
    isaac_headless = ExecuteProcess(
        cmd=[
            str(executable_dir / 'run_isaac.sh'), '--headless',
            '--calibration-block', '--no-cup', '--rgbd-camera',
            *block_args,
        ],
        output='screen',
        condition=IfCondition(headless),
    )
    isaac_gate = Node(
        package=PACKAGE,
        executable='startup_gate.py',
        arguments=['isaac', '--timeout', startup_timeout],
        output='screen',
    )
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'capabilities': 'move_group/ExecuteTaskSolutionCapability'},
            {
                # Isaac's measured-feedback bridge needs extra settling time
                # after short Cartesian segments. Scope this to the simulator.
                'trajectory_execution.allowed_execution_duration_scaling': 1.5,
                # The simulated gripper deliberately opens/closes at 5 mm/s and
                # confirms that both jaws have settled before succeeding.  A
                # full-stroke command therefore needs about 7 s, longer than
                # the 2.4 s MTC trajectory duration used for interpolation.
                'trajectory_execution.allowed_goal_duration_margin': 5.0,
            },
        ],
    )
    move_group_gate = Node(
        package=PACKAGE,
        executable='startup_gate.py',
        arguments=['move_group', '--timeout', startup_timeout],
        output='screen',
    )
    workcell_scene = Node(
        package=PACKAGE,
        executable='workcell_scene.py',
        arguments=['--no-cup'],
        output='screen',
    )
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

    def continue_or_stop(actions, stage):
        def callback(event, _context):
            if event.returncode == 0:
                return actions
            return [
                LogInfo(msg=f'{stage} failed; shutting down.'),
                EmitEvent(event=Shutdown(reason=f'{stage} startup failed')),
            ]
        return callback

    def stop_when_isaac_exits(event, _context):
        return [EmitEvent(event=Shutdown(reason=f'Isaac exited with code {event.returncode}'))]

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('startup_timeout', default_value='180'),
        DeclareLaunchArgument(
            'safety_cancel_enabled', default_value='false',
            description='Abort active controller goals on validated safety events.'),
        DeclareLaunchArgument(
            'fault_injection_enabled', default_value='false',
            description='Enable the test-only Isaac physical fault endpoint.'),
        DeclareLaunchArgument('block_x', default_value='-0.30'),
        DeclareLaunchArgument('block_y', default_value='0.30'),
        DeclareLaunchArgument('block_size_x', default_value='0.030'),
        DeclareLaunchArgument('block_size_y', default_value='0.030'),
        DeclareLaunchArgument('block_size_z', default_value='0.050'),
        DeclareLaunchArgument('block_mass_kg', default_value='0.05'),
        RegisterEventHandler(OnProcessExit(
            target_action=isaac_gate,
            on_exit=continue_or_stop([
                LogInfo(msg='Isaac feedback ready; starting Pallet MoveGroup.'),
                include('static_virtual_joint_tfs.launch.py'),
                include('rsp.launch.py'),
                Node(
                    package=PACKAGE,
                    executable='isaac_action_bridge.py',
                    output='screen',
                    parameters=[{
                        'safety_cancel_enabled': ParameterValue(
                            safety_cancel_enabled, value_type=bool),
                    }],
                ),
                move_group,
                move_group_gate,
            ], 'Isaac readiness check'),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=move_group_gate,
            on_exit=continue_or_stop([workcell_scene], 'MoveGroup readiness check'),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=workcell_scene,
            on_exit=continue_or_stop([
                LogInfo(msg='Pallet workcell ready for direct-pose MTC execution.'),
                rviz,
            ], 'Workcell scene registration'),
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=isaac_gui, on_exit=stop_when_isaac_exits,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=isaac_headless, on_exit=stop_when_isaac_exits,
        )),
        LogInfo(msg='Starting Pallet Isaac Sim and waiting for measured joint feedback.'),
        isaac_gui,
        isaac_headless,
        isaac_gate,
    ])
