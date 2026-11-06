"""Measured ID7 raw motor position -> visual joints; never sends hardware I/O.

The virtual wrist geometry is a visualization assumption, not contact feedback.
"""
import math
from adaptive_gripper_core import AdaptiveGripper

MOTOR_JOINT = 'gripper_motor'


class AdaptiveMotorMapping:
    def __init__(self, geometry_path):
        self.model = AdaptiveGripper.load(geometry_path)

    def measured_state(self, arm_names, arm_positions, motor_raw_rad):
        if len(arm_names) != len(arm_positions) or len(set(arm_names)) != len(arm_names):
            raise ValueError('Invalid arm state')
        if not all(type(q) in (int, float) and math.isfinite(q) for q in arm_positions):
            raise ValueError('Invalid measured arm positions')
        pose = self.model.pose(motor_raw_rad)
        joints = dict(zip(arm_names, arm_positions))
        if set(joints) & (set(pose['joints']) | {MOTOR_JOINT}):
            raise ValueError('Arm and gripper joint names overlap')
        joints[MOTOR_JOINT] = motor_raw_rad
        joints.update(pose['joints'])
        return joints


def load_motor_profile(path):
    """Load the confirmed profile without opening or enabling a motor."""
    import yaml
    with open(path) as stream:
        config = yaml.safe_load(stream)
    if config['motor_id'] != 7 or config['motor_model'] != 'RS05':
        raise ValueError('Expected confirmed ID7 RS05')
    for key in ('velocity_rad_s', 'acceleration_rad_s2'):
        value = config[key]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'Invalid {key}')
    if config['current_limit_policy'] != 'preserve_existing':
        raise ValueError('Only preserve-existing current limit is supported')
    if (config['motor_lower_rad'], config['motor_upper_rad']) != (-12., 12.):
        raise ValueError('Bounds must match the measured calibration range')
    return config
