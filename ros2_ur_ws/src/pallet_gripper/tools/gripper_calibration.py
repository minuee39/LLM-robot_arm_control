#!/usr/bin/env python3
"""Map one-jaw J8 travel (metres) to total fingertip clearance."""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]
URDF = PACKAGE / 'urdf/pallet_gripper.urdf'
# Mesh vertices and joint axes are decimal approximations. Allow one micrometre
# of endpoint roundoff without accepting physically out-of-range jaw openings.
CLEARANCE_TOLERANCE_M = 1e-6


def vector(text):
    return np.fromstring(text, sep=' ')


def rpy_matrix(text):
    roll, pitch, yaw = vector(text)
    cr, cp, cy = np.cos((roll, pitch, yaw))
    sr, sp, sy = np.sin((roll, pitch, yaw))
    return np.array(
        (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )
    )


def obj_vertices(path):
    vertices = []
    for line in path.read_text(encoding='utf-8').splitlines():
        if line.startswith('v '):
            vertices.append(tuple(float(value) for value in line.split()[1:4]))
    if not vertices:
        raise ValueError(f'No vertices found in {path}')
    return np.asarray(vertices)


class GripperGeometry:
    def __init__(self):
        self.robot = ET.parse(URDF).getroot()
        limit = self.robot.find("joint[@name='J8']/limit")
        self.lower = float(limit.get('lower'))
        self.upper = float(limit.get('upper'))

    def jaw_vertices(self, name, position):
        joint = self.robot.find(f"joint[@name='{name}']")
        origin = joint.find('origin')
        rotation = rpy_matrix(origin.get('rpy'))
        translation = vector(origin.get('xyz')) + rotation @ (
            vector(joint.find('axis').get('xyz')) * position
        )
        mesh_uri = self.robot.find(
            f"link[@name='{name}']/collision/geometry/mesh"
        ).get('filename')
        mesh_path = PACKAGE / mesh_uri.removeprefix('package://pallet_gripper/')
        return obj_vertices(mesh_path) @ rotation.T + translation

    def clearance(self, position, tip_depth):
        jaws = {
            name: self.jaw_vertices(name, position) for name in ('J8', 'J9')
        }
        tip_z = max(points[:, 2].max() for points in jaws.values())
        tips = {
            name: points[points[:, 2] >= tip_z - tip_depth]
            for name, points in jaws.items()
        }
        negative = min(tips.values(), key=lambda points: points[:, 0].mean())
        positive = max(tips.values(), key=lambda points: points[:, 0].mean())
        return positive[:, 0].min() - negative[:, 0].max()

    def position_for_clearance(self, target, tip_depth):
        closed_clearance = self.clearance(self.lower, tip_depth)
        open_clearance = self.clearance(self.upper, tip_depth)
        if (not np.isfinite(target) or
                not closed_clearance - CLEARANCE_TOLERANCE_M <= target <=
                open_clearance + CLEARANCE_TOLERANCE_M):
            raise ValueError(
                f'target {target * 1000:.3f} mm is outside the model range '
                f'{closed_clearance * 1000:.3f}..{open_clearance * 1000:.3f} mm'
            )
        if abs(target - closed_clearance) <= CLEARANCE_TOLERANCE_M:
            return self.lower
        if abs(target - open_clearance) <= CLEARANCE_TOLERANCE_M:
            return self.upper
        low, high = self.lower, self.upper
        for _ in range(60):
            middle = 0.5 * (low + high)
            if self.clearance(middle, tip_depth) >= target:
                high = middle
            else:
                low = middle
        return 0.5 * (low + high)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--object-width-mm', type=float)
    parser.add_argument('--tip-depth-mm', type=float, default=10.0)
    args = parser.parse_args()
    geometry = GripperGeometry()
    tip_depth = args.tip_depth_mm / 1000.0

    for position in np.linspace(geometry.lower, geometry.upper, 7):
        clearance = geometry.clearance(float(position), tip_depth)
        print(f'J8={position:.6f} m  clearance={clearance * 1000:.3f} mm')

    if args.object_width_mm is not None:
        target = args.object_width_mm / 1000.0
        try:
            position = geometry.position_for_clearance(target, tip_depth)
        except ValueError as error:
            parser.error(str(error))
        print(
            f'TARGET object_width={args.object_width_mm:.3f} mm '
            f'J8={position:.6f} m'
        )


if __name__ == '__main__':
    main()
