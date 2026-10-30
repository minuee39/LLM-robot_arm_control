"""Compare measured loaded motion, not planned trajectory or physical contact forces."""
import argparse
import json
import math
from pathlib import Path

LOADED = {'bilateral_loaded', 'bilateral_stationary_loaded'}


def normalized(q):
    if len(q) != 4 or not all(map(math.isfinite, q)):
        raise ValueError('Invalid xyzw quaternion')
    n = math.sqrt(sum(x*x for x in q))
    if n < 1e-12:
        raise ValueError('Zero quaternion')
    return [x/n for x in q]


def conjugate(q):
    return [-q[0], -q[1], -q[2], q[3]]


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def rotation_vector(q):
    q = normalized(q)
    if q[3] < 0:
        q = [-x for x in q]
    n = math.sqrt(sum(x*x for x in q[:3]))
    if n < 1e-12:
        return [0.0, 0.0, 0.0]
    angle = 2*math.atan2(n, q[3])
    return [x*angle/n for x in q[:3]]


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def measured_kinematics(samples, window=0.5, max_gap=0.25):
    """Windowed world-frame velocity; reject gaps instead of bridging missing data."""
    velocities = []
    start = 0
    for end, current in enumerate(samples):
        if end and (current['seconds']-samples[end-1]['seconds'] > max_gap or
                    current['seconds'] <= samples[end-1]['seconds']):
            start = end
        while start < end and current['seconds']-samples[start+1]['seconds'] >= window:
            start += 1
        previous = samples[start]
        dt = current['seconds']-previous['seconds']
        if dt < window:
            continue
        p = previous['tcp_measurement']
        c = current['tcp_measurement']
        linear = [(x-y)/dt for x, y in zip(c['position_m'], p['position_m'])]
        delta = multiply(normalized(c['orientation_xyzw']),
                         conjugate(normalized(p['orientation_xyzw'])))
        angular = [x/dt for x in rotation_vector(delta)]
        velocities.append({'seconds': (current['seconds']+previous['seconds'])/2,
                           'linear_m_s': linear, 'angular_rad_s': angular})
    accelerations = []
    for p, c in zip(velocities, velocities[1:]):
        dt = c['seconds']-p['seconds']
        if not 0 < dt <= max_gap:
            continue
        accelerations.append({
            'seconds': (p['seconds']+c['seconds'])/2,
            'linear_m_s2': [(x-y)/dt for x, y in zip(c['linear_m_s'], p['linear_m_s'])],
            'angular_rad_s2': [(x-y)/dt for x, y in zip(c['angular_rad_s'], p['angular_rad_s'])],
        })
    return velocities, accelerations


def analyze(data, window=0.5):
    samples = [s for s in data['samples']
               if s.get('relative_displacement_norm_m') is not None
               and s['joint_measurement']['contact_state'] in LOADED
               and s['tcp_measurement']['status'] == 'fresh']
    if not samples:
        raise ValueError('No fresh loaded samples with contact reference')
    reference = samples[0]
    ref_relative = multiply(conjugate(normalized(reference['tcp_measurement']['orientation_xyzw'])),
                            normalized(reference['orientation_xyzw']))
    velocities, accelerations = measured_kinematics(samples, window)
    result = {
        'window_seconds': window, 'max_gap_seconds': 0.25,
        'analysis_interval_start_seconds': reference['seconds'],
        'reported_contact_reference_seconds': data['summary'].get('telemetry', {}).get(
            'contact_reference_seconds'),
        'initial_tcp_orientation_xyzw': reference['tcp_measurement']['orientation_xyzw'],
        'loaded_sample_count': len(samples),
        'arm_joint_record_available': bool(reference['joint_measurement'].get('arm')),
        'initial_arm': reference['joint_measurement'].get('arm', {}),
        'slip_detected': data['summary']['slip_detected'],
        'placement_pass': data['summary']['placement_pass'],
        'caution': 'Measured finite differences, smoothed over window; not instantaneous dynamics or planned path. Loaded interval includes lift/lower.',
    }
    for label, cutoff in [('initial_5s', reference['seconds']+5), ('all_loaded', math.inf)]:
        selected = [s for s in samples if s['seconds'] <= cutoff]
        vmax = [s for s in velocities if s['seconds'] <= cutoff]
        amax = [s for s in accelerations if s['seconds'] <= cutoff]
        rotations = []
        for s in selected:
            relative = multiply(conjugate(normalized(s['tcp_measurement']['orientation_xyzw'])),
                                normalized(s['orientation_xyzw']))
            rotations.append(norm(rotation_vector(multiply(conjugate(ref_relative), relative))))
        result[label] = {
            'max_relative_displacement_mm': max(s['relative_displacement_norm_m'] for s in selected)*1000,
            'max_relative_rotation_deg': math.degrees(max(rotations)),
            'max_linear_velocity_m_s': max((norm(s['linear_m_s']) for s in vmax), default=None),
            'max_angular_velocity_deg_s': max((math.degrees(norm(s['angular_rad_s'])) for s in vmax), default=None),
            'max_linear_acceleration_m_s2': max((norm(s['linear_m_s2']) for s in amax), default=None),
            'max_angular_acceleration_deg_s2': max((math.degrees(norm(s['angular_rad_s2'])) for s in amax), default=None),
            'velocity_estimate_count': len(vmax), 'acceleration_estimate_count': len(amax),
        }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('motion', nargs='+', type=Path)
    parser.add_argument('--window', type=float, default=0.5)
    args = parser.parse_args()
    if not math.isfinite(args.window) or not 0.25 <= args.window <= 2:
        parser.error('--window must be within [0.25, 2] seconds')
    results = {str(p.resolve()): analyze(json.loads(p.read_text()), args.window) for p in args.motion}
    print(json.dumps(results, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
