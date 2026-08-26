from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


DEFAULT_REPOSITORY_ROOT = "/home/minwoo/Desktop/LLM"
DEFAULT_REMIND_ROOT = "/home/minwoo/Desktop/yolo_remind/remind-reid-tracker"
DEFAULT_REMIND_PYTHON = "/home/minwoo/anaconda3/envs/remind/bin/python"


def generate_launch_description() -> LaunchDescription:
    repository_root = LaunchConfiguration("repository_root")
    python_executable = LaunchConfiguration("python_executable")
    model = LaunchConfiguration("model")
    confidence = LaunchConfiguration("confidence")
    rviz_config = LaunchConfiguration("rviz_config")
    start_camera = LaunchConfiguration("start_camera")
    start_remind = LaunchConfiguration("start_remind")
    remind_root = LaunchConfiguration("remind_root")
    remind_python = LaunchConfiguration("remind_python")
    remind_model = LaunchConfiguration("remind_model")
    remind_confidence = LaunchConfiguration("remind_confidence")
    remind_stride = LaunchConfiguration("remind_stride")
    remind_device = LaunchConfiguration("remind_device")
    remind_output_dir = LaunchConfiguration("remind_output_dir")

    astra_launch = Path(get_package_share_directory("astra_camera")) / "launch" / "astra.launch.xml"
    default_rviz_config = PathJoinSubstitution(
        [get_package_share_directory("ur10e_vision_bringup"), "config", "astra_yolo_remind.rviz"]
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
            "--model", model,
            "--conf", confidence,
            "--label-mode", "model",
            "--classes", "",
            "--rgb-topic", "/camera/color/image_raw",
            "--depth-topic", "/camera/depth/image_raw",
            "--camera-info-topic", "/camera/color/camera_info",
            "--coordinate-mode", "camera",
            "--require-segmentation",
            "--sync-slop", "0.05",
        ],
        additional_env={"MPLCONFIGDIR": "/tmp/matplotlib", "ROS_LOG_DIR": "/tmp/ros_logs"},
        output="screen",
    )

    remind = ExecuteProcess(
        cmd=[
            remind_python,
            PathJoinSubstitution([repository_root, "ur10e", "apps", "remind_ros_camera_node.py"]),
            "--remind-root", remind_root,
            "--model", remind_model,
            "--rgb-topic", "/camera/color/image_raw",
            "--output-topic", "/remind/annotated",
            "--detections-topic", "/remind/detections",
            "--confidence", remind_confidence,
            "--stride", remind_stride,
            "--device", remind_device,
            "--output-dir", remind_output_dir,
        ],
        condition=IfCondition(start_remind),
        additional_env={
            "HF_HUB_OFFLINE": "1",
            "MPLCONFIGDIR": "/tmp/matplotlib-remind",
            "ROS_LOG_DIR": "/tmp/ros_logs",
            "TRANSFORMERS_OFFLINE": "1",
            "PYTHONNOUSERSITE": "1",
            "YOLO_CONFIG_DIR": "/tmp/Ultralytics",
        },
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="astra_yolo_remind_rviz",
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
                "model", default_value=PathJoinSubstitution([repository_root, "yolo26n-seg.pt"])
            ),
            DeclareLaunchArgument("confidence", default_value="0.4"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_remind", default_value="true"),
            DeclareLaunchArgument("remind_root", default_value=DEFAULT_REMIND_ROOT),
            DeclareLaunchArgument("remind_python", default_value=DEFAULT_REMIND_PYTHON),
            DeclareLaunchArgument(
                "remind_model",
                default_value=PathJoinSubstitution([remind_root, "yolo", "yolo11n-seg.pt"]),
            ),
            DeclareLaunchArgument("remind_confidence", default_value="0.25"),
            DeclareLaunchArgument("remind_stride", default_value="3"),
            DeclareLaunchArgument("remind_device", default_value="auto"),
            DeclareLaunchArgument("remind_output_dir", default_value="/tmp/remind_ros"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            camera,
            yolo,
            remind,
            rviz,
        ]
    )
