"""Validate per-run Isaac block overrides without changing the shared workcell."""

import math


def diagnostic_block_poses(entries, workcell):
    """Explicit simulation fixtures in base metres/radians; never estimator input."""
    result = {}
    if entries is None:
        return result
    sx, sy, _ = workcell['block']['size']
    tx, ty, _ = workcell['table']['size']
    for entry in entries:
        if len(entry) != 4:
            raise ValueError('diagnostic block pose requires OBJECT X Y YAW')
        name, *raw = entry
        if name not in {'red_block', 'green_block', 'blue_block'} or name in result:
            raise ValueError('diagnostic block pose requires unique known object')
        if any(isinstance(value, bool) for value in raw):
            raise ValueError('diagnostic block pose must be finite metres/radians')
        try:
            x, y, yaw = map(float, raw)
        except (TypeError, ValueError) as error:
            raise ValueError('diagnostic block pose must be finite metres/radians') from error
        if not all(math.isfinite(v) for v in (x, y, yaw)) or not -math.pi <= yaw <= math.pi:
            raise ValueError('diagnostic block pose must be finite with yaw within [-pi, pi]')
        ex = (abs(math.cos(yaw))*sx + abs(math.sin(yaw))*sy)/2
        ey = (abs(math.sin(yaw))*sx + abs(math.cos(yaw))*sy)/2
        if abs(x)+ex >= tx/2 or abs(y)+ey >= ty/2:
            raise ValueError('diagnostic block pose must fit within tabletop')
        result[name] = (x, y, yaw)
    return result


def rgbd_camera_resolution(mode='640x480'):
    modes = {'640x480': (640, 480), '1280x960': (1280, 960)}
    if mode not in modes:
        raise ValueError('unsupported RGB-D resolution')
    return modes[mode]


def rgbd_camera_offsets(view='source'):
    """Fixed optical view offsets in base metres; source preserves legacy pose."""
    if view == 'source':
        return (-0.55, 0.65, 0.85), (-0.32, 0.18, 0.025)
    if view == 'workcell':
        return (-0.15, 0.45, 1.10), (-0.02, -0.09, 0.025)
    raise ValueError('camera view must be source or workcell')


def joint_state_snapshot(position, velocity, effort, expected_count):
    """Return a complete articulation snapshot, or None when feedback is unavailable."""
    readings = (position, velocity, effort)
    if any(reading is None for reading in readings):
        return None
    try:
        if any(len(reading) != expected_count for reading in readings):
            return None
    except TypeError:
        return None
    return readings


def gripper_effort_limit(physics):
    """Return the Isaac-only force ceiling for each prismatic jaw."""
    value = physics.get('gripper_effort_limit_n_per_finger')
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('gripper effort limit must be a number in N/finger')
    if not math.isfinite(value) or value <= 0:
        raise ValueError('gripper effort limit must be finite and positive')
    return float(value)


def gripper_symmetry_config(physics):
    """Validate Isaac-only bilateral-jaw centering parameters."""
    gain = physics.get('gripper_symmetry_gain')
    max_correction = physics.get('gripper_symmetry_max_correction_m')
    values = (gain, max_correction)
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           for value in values):
        raise ValueError('gripper symmetry parameters must be numeric')
    if not all(math.isfinite(value) and value > 0 for value in values):
        raise ValueError('gripper symmetry parameters must be finite and positive')
    if gain > 1:
        raise ValueError('gripper symmetry gain must not exceed 1')
    return float(gain), float(max_correction)


def symmetric_jaw_targets(command, observed_j8, observed_j9, lower, upper,
                          gain, max_correction):
    """Return differential targets that recenter an independently driven jaw pair."""
    values = (command, observed_j8, observed_j9, lower, upper, gain, max_correction)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('jaw centering values must be finite')
    if lower > command or command > upper or lower >= upper:
        raise ValueError('jaw command must be within valid limits')
    if not 0 < gain <= 1 or max_correction <= 0:
        raise ValueError('jaw centering gain and correction must be positive')
    correction = max(-max_correction, min(
        max_correction, gain * (observed_j8 - observed_j9) / 2))
    return (
        max(lower, min(upper, command - correction)),
        max(lower, min(upper, command + correction)),
    )


def block_scenario(workcell, center_xy=None, size=None, mass_kg=None):
    block = workcell['block']
    center = tuple(block['center_xy_base'] if center_xy is None else center_xy)
    dimensions = tuple(block['size'] if size is None else size)
    mass = block['mass_kg'] if mass_kg is None else mass_kg
    if len(center) != 2 or len(dimensions) != 3:
        raise ValueError('block center needs 2 values and size needs 3 values')
    if not all(math.isfinite(value) for value in (*center, *dimensions, mass)):
        raise ValueError('block scenario values must be finite')
    if any(value <= 0 for value in dimensions) or mass <= 0:
        raise ValueError('block dimensions and mass must be positive')
    if max(dimensions[:2]) >= 0.067:
        raise ValueError('block XY footprint must fit within the measured 67 mm gripper opening')
    table_x, table_y = workcell['table']['size'][:2]
    if abs(center[0]) + dimensions[0] / 2 >= table_x / 2 or (
            abs(center[1]) + dimensions[1] / 2 >= table_y / 2):
        raise ValueError('block must fit inside the tabletop')
    return {'center_xy_base': list(center), 'size': list(dimensions), 'mass_kg': mass}
