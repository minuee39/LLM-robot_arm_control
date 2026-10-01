"""Validate per-run Isaac block overrides without changing the shared workcell."""

import math


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
