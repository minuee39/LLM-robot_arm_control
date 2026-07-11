import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # planning_context
    moveit_config = MoveItConfigsBuilder(
        "ur10e_robotiq_2f140",
        package_name="ur10e_robotiq_2f140_moveit_config",
    ).robot_description(
        file_path="config/ur10e_robotiq_2f140.urdf.xacro",
    ).robot_description_semantic(
        file_path="config/ur10e_robotiq_2f140.srdf",
    ).to_moveit_configs()

    moveit_config_pkg = get_package_share_directory("ur10e_robotiq_2f140_moveit_config")

    # Load ExecuteTaskSolutionCapability so we can execute found solutions in simulation.
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability",
    }

    # Start the actual move_group node/action server.
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("mtc_tutorial"),
        "launch",
        "mtc.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # ros2_control using mock hardware from the generated URDF.
    ros2_controllers_path = os.path.join(
        moveit_config_pkg,
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.to_dict(), ros2_controllers_path],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "ur_manipulator_controller",
        "gripper_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
        ]
        + load_controllers
    )
