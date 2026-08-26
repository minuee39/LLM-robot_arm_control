from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


DEFAULT_REPOSITORY_ROOT = "/home/minwoo/Desktop/LLM"


def generate_launch_description() -> LaunchDescription:
    repository_root = LaunchConfiguration("repository_root")
    python_executable = LaunchConfiguration("python_executable")
    model = LaunchConfiguration("model")
    confidence = LaunchConfiguration("confidence")
    rviz_config = LaunchConfiguration("rviz_config")
    start_camera = LaunchConfiguration("start_camera")

    astra_launch = Path(get_package_share_directory("astra_camera")) / "launch" / "astra.launch.xml"
    default_rviz_config = PathJoinSubstitution(
        [get_package_share_directory("ur10e_vision_bringup"), "config", "astra_yolo.rviz"]
    )

    camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(str(astra_launch)),
        condition=IfCondition(start_camera),
        launch_arguments={
            "depth_registration": "true",
            "color_depth_synchronization": "true",
            "enable_ir": "false",
            "enable_point_cloud": "true",
            "color_width": "640",
            "color_height": "480",
            "color_fps": "30",
            "depth_width": "640",
            "depth_height": "480",
            "depth_fps": "30",
        }.items(),
    )

    yolo = ExecuteProcess(
        cmd=[
            python_executable,
            PathJoinSubstitution([repository_root, "ur10e", "apps", "yolo_camera_node.py"]),
            "--model",
            model,
            "--conf",
            confidence,
            "--label-mode",
            "model",
            "--classes",
            "",
            "--rgb-topic",
            "/camera/color/image_raw",
            "--depth-topic",
            "/camera/depth/image_raw",
            "--camera-info-topic",
            "/camera/color/camera_info",
            "--coordinate-mode",
            "camera",
            "--require-segmentation",
            "--sync-slop",
            "0.05",
        ],
        additional_env={
            "MPLCONFIGDIR": "/tmp/matplotlib",
            "ROS_LOG_DIR": "/tmp/ros_logs",
        },
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="astra_yolo_rviz",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("repository_root", default_value=DEFAULT_REPOSITORY_ROOT),
            DeclareLaunchArgument(
                "python_executable",
                default_value=PathJoinSubstitution([repository_root, ".venv", "bin", "python"]),
            ),
            DeclareLaunchArgument(
                "model",
                default_value=PathJoinSubstitution([repository_root, "yolo26n-seg.pt"]),
            ),
            DeclareLaunchArgument("confidence", default_value="0.4"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            camera,
            yolo,
            rviz,
        ]
    )
