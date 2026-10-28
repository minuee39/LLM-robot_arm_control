#!/usr/bin/env python3
"""Export URDF root-frame axes or fit motor/joint calibration from measured poses."""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from hand_guiding_core import origin, rotation


def axes_at_zero(path):
    robot = ET.parse(path).getroot()
    joints = robot.findall('joint')
    children = {j.find('child').get('link') for j in joints}
    root = (set(l.get('name') for l in robot.findall('link'))-children).pop()
    frames = {root: np.eye(4)}
    rows = []
    pending = list(joints)
    while pending:
        progressed = False
        for j in pending[:]:
            parent, child = j.find('parent').get('link'), j.find('child').get('link')
            if parent not in frames:
                continue
            t = frames[parent] @ origin(j.find('origin'))
            frames[child] = t
            if j.get('type') != 'fixed':
                a = np.fromstring(j.find('axis').get('xyz'), sep=' ')
                a = a/np.linalg.norm(a)
                rows.append({'joint':j.get('name'), 'type':j.get('type'), 'origin_root_m':t[:3,3].tolist(), 'positive_axis_root':(t[:3,:3]@a).tolist(), 'mimic':j.find('mimic') is not None})
            pending.remove(j)
            progressed = True
        if not progressed:
            raise ValueError('Invalid URDF tree')
    return {'frame':root, 'pose':'all URDF coordinates zero, not photograph pose', 'joints':rows}


def fit(samples, tolerance):
    """motor = slope*URDF_joint + zero. Samples must be continuously unwrapped."""
    a = np.asarray(samples, dtype=float)
    if a.ndim != 2 or a.shape[1] != 2 or len(a) < 3 or not np.isfinite(a).all():
        raise ValueError('Need >=3 finite [URDF_joint_rad, continuous_motor_rad] samples')
    q, motor = a.T
    if np.ptp(q) < 0.01:
        raise ValueError('Reference joint travel too small')
    slope, zero = np.linalg.lstsq(np.column_stack([q,np.ones(len(q))]), motor, rcond=None)[0]
    error = float(np.max(np.abs(motor-(slope*q+zero))))
    if not np.isfinite(tolerance) or tolerance <= 0 or abs(slope) < 1e-6 or error > tolerance:
        raise ValueError(f'Calibration inconsistent or nonlinear; max motor residual {error:.6g} rad')
    return {'direction':1 if slope > 0 else -1, 'ratio':abs(float(slope)), 'zero_rad':float(zero), 'max_residual_motor_rad':error}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--samples', help='JSON mapping joint name to >=3 [URDF rad, unwrapped motor rad] pairs')
    p.add_argument('--config', default=str(ROOT/'config/hand_guiding.yaml'))
    p.add_argument('--tolerance', type=float, default=0.01)
    p.add_argument('--apply', action='store_true')
    args = p.parse_args()
    result = axes_at_zero(ROOT/'urdf/pallet_with_gripper.urdf')
    if args.samples:
        data = json.loads(Path(args.samples).read_text())
        cfg = yaml.safe_load(Path(args.config).read_text())
        if not data or set(data)-set(cfg['joints']):
            raise ValueError('Unknown or empty joint measurements')
        fitted = {name:fit(samples,args.tolerance) for name,samples in data.items()}
        result['calibration'] = fitted
        if args.apply:
            for name, values in fitted.items():
                cfg['joints'][name].update({k:values[k] for k in ('direction','ratio','zero_rad')})
            cfg['commissioned'] = False
            cfg['gripper_homed'] = False
            Path(args.config).write_text(yaml.safe_dump(cfg,sort_keys=False))
    elif args.apply:
        p.error('--apply requires measured --samples')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
