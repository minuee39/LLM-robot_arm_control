from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


FLOAT_DEFAULTS = {
    'object_x': -0.30,
    'object_y': 0.30,
    'object_z': 0.05,
    'object_size_x': 0.10,
    'object_size_y': 0.0515,
    'object_size_z': 0.10,
    'table_size_x': 0.0,
    'table_size_y': 0.0,
    'table_height': 0.0,
    'table_center_x': 0.0,
    'table_center_y': 0.0,
    'table_top_z': 0.0,
    'place_x': 0.30,
    'place_y': -0.30,
    'place_z': 0.05,
    'max_solutions': 1.0,
    'max_solution_cost': 60.0,
    'task_timeout': -1.0,
    'place_max_ik_solutions': 1.0,
    'place_ik_min_solution_distance': 1.0,
    'move_to_pick_timeout': 5.0,
    'move_to_pick_max_path_length': -1.0,
    'move_to_place_timeout': 4.0,
    'move_to_place_max_path_length': -1.0,
    'transfer_min_clearance': -1.0,
    'transfer_clearance_check_step': 0.02,
    'return_home_timeout': 1.0,
    'return_home_max_path_length': -1.0,
    'arm_velocity_scaling': 0.5,
    'arm_acceleration_scaling': 0.5,
    'cartesian_velocity_scaling': 0.25,
    'cartesian_acceleration_scaling': 0.25,
    'gripper_velocity_scaling': 0.5,
    'gripper_acceleration_scaling': 0.5,
    'gripper_close_min': 0.02,
    'gripper_close_max': 0.50,
    'gripper_close_step': 0.08,
    'recovery_retreat_distance': 0.15,
    'recovery_retreat_timeout': 3.0,
    'recovery_home_timeout': 5.0,
    'recovery_direction_x': 0.0,
    'recovery_direction_y': 0.0,
    'recovery_direction_z': 1.0,
}


def _profiles():
    path = Path(get_package_share_directory('mtc_tutorial')) / 'config/robot_profiles.yaml'
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    profiles = data.get('profiles', {}) if isinstance(data, dict) else {}
    if not profiles:
        raise RuntimeError(f'No robot profiles found in {path}')
    return profiles


def _as_bool(value):
    normalized = value.strip().lower()
    if normalized in {'1', 'true', 'yes', 'on'}:
        return True
    if normalized in {'0', 'false', 'no', 'off'}:
        return False
    raise ValueError(f'Expected a boolean value, got {value!r}')


def _launch_setup(context):
    profile_name = LaunchConfiguration('robot_profile').perform(context)
    profiles = _profiles()
    if profile_name not in profiles:
        available_profiles = ', '.join(sorted(profiles))
        raise RuntimeError(
            f'Unknown robot_profile {profile_name!r}; available: {available_profiles}'
        )
    profile = profiles[profile_name]

    moveit_config = (
        MoveItConfigsBuilder(
            profile['robot_name'],
            package_name=profile['moveit_config_package'],
        )
        .robot_description(file_path=profile['robot_description_file'])
        .robot_description_semantic(file_path=profile['semantic_file'])
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
        .to_dict()
    )

    parameters = dict(profile.get('mtc_parameters', {}))
    for name, shared_default in FLOAT_DEFAULTS.items():
        raw = LaunchConfiguration(name).perform(context)
        if raw == 'profile':
            parameters[name] = float(parameters.get(name, shared_default))
        else:
            parameters[name] = float(raw)

    parameters.update(
        {
            'object_id': LaunchConfiguration('object_id').perform(context),
            'planner_id': LaunchConfiguration('planner_id').perform(context),
            'execution_mode': LaunchConfiguration('execution_mode').perform(context),
            'recovery_only': _as_bool(
                LaunchConfiguration('recovery_only').perform(context)
            ),
            'recovery_candidate_id': LaunchConfiguration(
                'recovery_candidate_id').perform(context),
            'planning_only': _as_bool(
                LaunchConfiguration('planning_only').perform(context)
            ),
            'robot_profile': profile_name,
        }
    )

    return [
        Node(
            package='mtc_tutorial',
            executable='mtc_tutorial',
            output='screen',
            parameters=[moveit_config, parameters],
        )
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'robot_profile',
            default_value='ur10e_robotiq',
            description='Robot profile from config/robot_profiles.yaml.',
        ),
        DeclareLaunchArgument('object_id', default_value='red_block'),
        DeclareLaunchArgument(
            'execution_mode',
            default_value='standard',
            description="Use 'standard' MTC execution or experimental 'blended' execution.",
        ),
        DeclareLaunchArgument(
            'planner_id',
            default_value='RRTConnectkConfigDefault',
            description='OMPL planner configuration used for sampled arm motions.',
        ),
        DeclareLaunchArgument('recovery_only', default_value='false'),
        DeclareLaunchArgument('recovery_candidate_id', default_value='vertical'),
        DeclareLaunchArgument(
            'planning_only',
            default_value='false',
            description='Plan and report the task without executing a trajectory.',
        ),
    ]
    arguments.extend(
        DeclareLaunchArgument(
            name,
            default_value='profile',
            description=(
                "Use the selected robot profile value when set to 'profile'; "
                'otherwise provide a numeric override.'
            ),
        )
        for name in FLOAT_DEFAULTS
    )
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_setup)])
