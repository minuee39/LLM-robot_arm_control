from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


DEFAULT_ISAAC_BRIDGE_SCRIPT = "/home/minwoo/Desktop/LLM/ur10e/scripts/run_isaac_moveit_bridge.sh"
DEFAULT_SCENE_STATE_FILE = "/tmp/ur10e_isaac_scene_objects.json"


def generate_launch_description():
    isaac_bridge_script = LaunchConfiguration("isaac_bridge_script")
    scene_state_file = LaunchConfiguration("scene_state_file")
    collision_period = LaunchConfiguration("collision_period")
    collision_timeout = LaunchConfiguration("collision_timeout")

    isaac_process = ExecuteProcess(
        cmd=[isaac_bridge_script],
        name="isaac_moveit_bridge_app",
        output="screen",
        sigterm_timeout="10",
        sigkill_timeout="5",
    )

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mtc_tutorial"), "launch", "mtc_isaac_demo.launch.py"]
            )
        )
    )

    action_bridge = Node(
        package="isaac_moveit_bridge",
        executable="action_bridge",
        name="isaac_moveit_action_bridge",
        output="screen",
    )

    collision_publisher = Node(
        package="isaac_moveit_bridge",
        executable="scene_collision_publisher",
        name="isaac_scene_collision_publisher",
        output="screen",
        arguments=[
            "--scene-state-file",
            scene_state_file,
            "--period",
            collision_period,
            "--timeout",
            collision_timeout,
        ],
    )

    shutdown_if_isaac_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=isaac_process,
            on_exit=[EmitEvent(event=Shutdown(reason="Isaac Sim bridge exited"))],
        )
    )
    shutdown_if_action_bridge_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=action_bridge,
            on_exit=[EmitEvent(event=Shutdown(reason="Isaac MoveIt action bridge exited"))],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "isaac_bridge_script",
                default_value=DEFAULT_ISAAC_BRIDGE_SCRIPT,
                description="Executable script that starts the Isaac Sim MoveIt bridge app.",
            ),
            DeclareLaunchArgument(
                "scene_state_file",
                default_value=DEFAULT_SCENE_STATE_FILE,
                description="Isaac scene-state JSON consumed by the collision publisher.",
            ),
            DeclareLaunchArgument(
                "collision_period",
                default_value="0.5",
                description="Collision scene update period in seconds.",
            ),
            DeclareLaunchArgument(
                "collision_timeout",
                default_value="10.0",
                description="MoveIt planning-scene service timeout in seconds.",
            ),
            isaac_process,
            action_bridge,
            moveit_rviz,
            collision_publisher,
            shutdown_if_isaac_exits,
            shutdown_if_action_bridge_exits,
        ]
    )
