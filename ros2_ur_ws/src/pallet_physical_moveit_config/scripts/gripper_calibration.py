#!/usr/bin/env python3
"""Offline conversion only: never opens serial/CAN or sends a motor command."""
import argparse
import json
import math
from pathlib import Path


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Expected a finite number')
    return float(value)


class Calibration:
    def __init__(self, config):
        if config.get('angle_unit') != 'rad' or config.get('opening_unit') != 'mm':
            raise ValueError('Calibration units must be rad and mm')
        if config.get('extrapolation') != 'reject':
            raise ValueError('Extrapolation must be disabled')
        points = config['samples']
        if len(points) < 2:
            raise ValueError('At least two calibration samples required')
        self.angles = [finite(p['motor_angle_rad']) for p in points]
        self.widths = [finite(p['opening_mm']) for p in points]
        if any(w < 0 for w in self.widths):
            raise ValueError('Negative opening')
        if any(a >= b for values in (self.angles, self.widths) for a,b in zip(values,values[1:])):
            raise ValueError('Calibration angles and widths must be strictly increasing')

    @staticmethod
    def interpolate(value, source, target):
        value = finite(value)
        if not source[0] <= value <= source[-1]:
            raise ValueError(f'{value} outside measured range [{source[0]}, {source[-1]}]')
        for i in range(1, len(source)):
            if value <= source[i]:
                ratio = (value-source[i-1])/(source[i]-source[i-1])
                return target[i-1]+ratio*(target[i]-target[i-1])
        raise ValueError('No interpolation interval')

    def motor_to_opening(self, angle_rad):
        return self.interpolate(angle_rad, self.angles, self.widths)

    def opening_to_motor(self, opening_mm):
        return self.interpolate(opening_mm, self.widths, self.angles)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    values = parser.add_mutually_exclusive_group(required=True)
    values.add_argument('--opening-mm', type=float, help='Convert requested free-space opening to a motor target; no actuation')
    values.add_argument('--motor-rad', type=float, help='Estimate free-space opening from raw Motor app angle')
    args = parser.parse_args()
    if args.config is None:
        from ament_index_python.packages import get_package_share_directory
        args.config = Path(get_package_share_directory('pallet_physical_moveit_config'))/'config/gripper_calibration.json'
    try:
        config = json.loads(args.config.read_text())
        calibration = Calibration(config)
        if args.opening_mm is not None:
            result = {'requested_opening_mm': args.opening_mm, 'motor_target_rad': calibration.opening_to_motor(args.opening_mm)}
        else:
            result = {'raw_motor_rad': args.motor_rad, 'estimated_free_space_opening_mm': calibration.motor_to_opening(args.motor_rad)}
        print(json.dumps({'motor_id': config['motor_id'], **result, 'actuation': False}, allow_nan=False))
    except (ValueError, KeyError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
