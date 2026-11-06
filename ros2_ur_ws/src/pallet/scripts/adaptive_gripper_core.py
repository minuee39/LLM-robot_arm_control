"""Measured free-space reconstruction of the CAD linkage; no motor I/O.

Lengths use mm internally; published URDF prismatic positions use m. Both nut
travel and distal contact opening are calibration inputs, not independent
predictions. Closed-loop cuts are checked before any pose is published.
"""
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Expected finite numeric input')
    return float(value)


def rotation(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s], [s, c]])


def angle_between(before, after):
    return math.atan2((before[0]*after[1]-before[1]*after[0]), np.dot(before, after))


def interpolate(value, source, target):
    value = finite(value)
    if not source[0] <= value <= source[-1]:
        raise ValueError(f'{value} outside measured range [{source[0]}, {source[-1]}]')
    return float(np.interp(value, source, target))


class Side:
    def __init__(self, data):
        self.p = {name: np.array(value['move_xy_mm'], dtype=float)
                  for name, value in data['pins'].items()}
        self.tip = np.array(data['contact_datum_xy_mm'], dtype=float)
        self.sign = float(np.sign(self.tip[0]))
        self.parallel_pad_angle = finite(data['parallel_pad_angle_rad'])

    def solve(self, displacement_mm, opening_mm, constraint):
        p = self.p
        g, a, b, n = [p[k] for k in ('rocker_ground', 'rocker_nut', 'rocker_outer', 'nut_pivot')]
        d, f, e, c = [p[k] for k in ('inner_ground', 'inner_elbow', 'pad_inner', 'pad_outer')]
        n1 = n + [0.0, finite(displacement_mm)]
        # Intersect rocker and helper circles, retaining the CAD assembly branch.
        delta = n1 - g
        distance = np.linalg.norm(delta)
        r1, r2 = np.linalg.norm(a - g), np.linalg.norm(a - n)
        if distance <= 1e-9 or not abs(r1-r2) <= distance <= r1+r2:
            raise ValueError('Nut/rocker loop cannot close at requested displacement')
        along = (r1*r1-r2*r2+distance*distance)/(2*distance)
        height = math.sqrt(max(0.0, r1*r1-along*along))
        unit = delta/distance
        foot = g+along*unit
        perpendicular = np.array([-unit[1], unit[0]])
        choices = [foot+height*perpendicular, foot-height*perpendicular]
        a1 = min(choices, key=lambda point: abs(angle_between(a-g, point-g)))
        rocker = angle_between(a-g, a1-g)
        helper = angle_between(a-n, a1-n1)
        b1 = g+rotation(rocker)@(b-g)
        width = finite(opening_mm)

        # Align the actual pin-to-pin axes, rather than assuming a zero
        # relative URDF angle means the exported CAD links are collinear.
        elbow = angle_between(e-f, f-d)

        def points(angles):
            if constraint in ('straight_inner', 'straight_parallel'):
                outer, pad, lower = angles
                relative_elbow = elbow
            elif constraint == 'parallel_pads':
                outer, lower, relative_elbow = angles
                pad = self.parallel_pad_angle
            else:
                raise ValueError('Unknown passive linkage constraint')
            c1 = b1+rotation(outer)@(c-b)
            f1 = d+rotation(lower)@(f-d)
            inner_e = f1+rotation(lower+relative_elbow)@(e-f)
            pad_e = c1+rotation(pad)@(e-c)
            if constraint == 'straight_parallel':
                tip = pad_e+rotation(self.parallel_pad_angle)@(self.tip-e)
            else:
                tip = c1+rotation(pad)@(self.tip-c)
            return c1, f1, inner_e, pad_e, tip

        def residual(angles):
            _, _, inner_e, pad_e, tip = points(angles)
            return np.r_[pad_e-inner_e, tip[0]-self.sign*width/2]

        if constraint == 'parallel_pads':
            pad = self.parallel_pad_angle
            target_cx = self.sign*width/2-(rotation(pad)@(self.tip-c))[0]
            dx = target_cx-b1[0]
            outer_length = np.linalg.norm(c-b)
            if abs(dx) > outer_length:
                raise ValueError('Requested opening exceeds outer link reach')
            c1 = np.array([target_cx, b1[1]+math.sqrt(max(0.,outer_length**2-dx**2))])
            outer = angle_between(c-b,c1-b1)
            e1 = c1+rotation(pad)@(e-c)
            delta = e1-d
            distance = np.linalg.norm(delta)
            rlower, rupper = np.linalg.norm(f-d), np.linalg.norm(e-f)
            if distance <= 1e-9 or not abs(rlower-rupper) <= distance <= rlower+rupper:
                raise ValueError('Requested opening exceeds inner linkage reach')
            along = (rlower**2-rupper**2+distance**2)/(2*distance)
            height = math.sqrt(max(0.,rlower**2-along**2))
            unit = delta/distance
            f1 = d+along*unit+self.sign*height*np.array([-unit[1],unit[0]])
            lower = angle_between(f-d,f1-d)
            upper = angle_between(e-f,e1-f1)
            angles = np.array([outer,lower,upper-lower])
        elif constraint in ('straight_inner', 'straight_parallel'):
            solution = least_squares(residual, [0., 0., 0.], bounds=(-1.2, 1.2),
                                     max_nfev=200, ftol=1e-12, xtol=1e-12, gtol=1e-12)
            if not solution.success:
                raise ValueError('Straight linkage solve did not converge')
            angles = solution.x
        else:
            raise ValueError('Unknown passive linkage constraint')
        if np.max(np.abs(residual(angles))) > 1e-6:
            raise ValueError('Selected linkage constraint cannot close at requested opening/nut depth')
        if constraint in ('straight_inner', 'straight_parallel'):
            outer, pad, lower = angles
        else:
            outer, lower, elbow = angles
            pad = self.parallel_pad_angle
        c1, f1, inner_e, pad_e, tip = points(angles)
        # URDF is a tree: check both omitted pin constraints explicitly.
        helper_a = n1+rotation(helper)@(a-n)
        errors = [np.linalg.norm(helper_a-a1), np.linalg.norm(pad_e-inner_e)]
        if max(errors) > 1e-6:
            raise ValueError('CAD pin closure tolerance exceeded')
        return {
            'angles': {'rocker': rocker, 'helper': helper,
                       'outer': outer-rocker, 'pad': pad-outer,
                       'inner_lower': lower, 'inner_upper': elbow,
                       'wrist': self.parallel_pad_angle-pad if constraint == 'straight_parallel' else 0.0},
            'absolute_angles': {'rocker': rocker, 'helper': helper, 'outer': outer,
                                'pad': pad, 'inner_lower': lower, 'inner_upper': lower+elbow},
            'anchors_mm': {'rocker': g, 'helper': n1, 'outer': b1,
                           'pad': c1, 'inner_lower': d, 'inner_upper': f1, 'wrist': pad_e},
            'contact_mm': tip, 'closure_error_mm': float(max(errors)),
        }


class AdaptiveGripper:
    def __init__(self, geometry, calibration):
        if geometry.get('schema_version') != 1:
            raise ValueError('Unsupported geometry version')
        if calibration.get('angle_unit') != 'rad' or calibration.get('opening_unit') != 'mm' or calibration.get('nut_depth_unit') != 'mm':
            raise ValueError('Expected rad/mm calibration')
        if calibration.get('extrapolation') != 'reject':
            raise ValueError('Extrapolation must be disabled')
        samples = calibration['samples']
        self.angles = [finite(row['motor_angle_rad']) for row in samples]
        self.widths = [finite(row['opening_mm']) for row in samples]
        self.depths = [finite(row['nut_depth_mm']) for row in samples]
        if len(samples) < 2 or any(a >= b for values in (self.angles, self.widths) for a, b in zip(values, values[1:])):
            raise ValueError('Angles and openings must be strictly increasing')
        if any(a <= b for a, b in zip(self.depths, self.depths[1:])):
            raise ValueError('Nut depths must be strictly decreasing')
        if any(value < 0 for value in self.widths+self.depths):
            raise ValueError('Negative opening/depth')
        self.geometry = geometry
        self.sides = {name: Side(data) for name, data in geometry['sides'].items()}
        self.reference_depth = finite(geometry['reference']['cad_nut_upper_face_depth_mm'])

    @classmethod
    def load(cls, geometry_path, calibration_path=None):
        geometry = json.loads(Path(geometry_path).read_text())
        calibration = json.loads(Path(calibration_path).read_text()) if calibration_path else geometry['calibration']
        return cls(geometry, calibration)

    def pose(self, motor_rad, constraint='straight_parallel'):
        angle = finite(motor_rad)
        width = interpolate(angle, self.angles, self.widths)
        depth = interpolate(angle, self.angles, self.depths)
        displacement = self.reference_depth-depth
        sides = {name: side.solve(displacement, width, constraint)
                 for name, side in self.sides.items()}
        joints = {'g2': displacement/1000.0}
        for side, result in sides.items():
            joints.update({f'{side}_{name}': float(value) for name, value in result['angles'].items()})
        if not all(math.isfinite(q) and abs(q) <= 2.4 for name, q in joints.items() if name != 'g2'):
            raise ValueError('Preview rotation exceeds URDF range')
        actual = sides['l']['contact_mm'][0]-sides['r']['contact_mm'][0]
        return {'motor_rad': angle, 'opening_mm': width, 'nut_depth_mm': depth,
                'reconstructed_opening_mm': float(actual),
                'nut_displacement_mm': displacement, 'joints': joints, 'sides': sides,
                'closure_error_mm': max(s['closure_error_mm'] for s in sides.values()),
                'actuation': False}
