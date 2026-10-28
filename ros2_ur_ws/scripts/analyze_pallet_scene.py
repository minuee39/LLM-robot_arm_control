"""Offline whole-scene audit; does not replace live safety monitors or command motion."""
import argparse
import json
import math
from pathlib import Path

BLOCKS = {'red_block', 'green_block', 'blue_block'}
# Observation quality only: allow one missing 1 Hz sample plus scheduling margin.
# These bounds do not relax the placement or live motion safety thresholds.
MAX_OBSERVATION_GAP_SECONDS = 2.5


def pose_metrics(block, target):
    position = block['position_m']
    quaternion = block['orientation_wxyz']
    if (len(position) != 3 or len(quaternion) != 4 or len(target) != 3 or
            any(type(x) not in (int, float) or not math.isfinite(x)
                for x in [*position, *quaternion, *target])):
        raise ValueError('Scene pose/target requires finite numerical components')
    length = math.sqrt(sum(x*x for x in quaternion))
    if abs(length - 1) > 0.01:
        raise ValueError('Invalid scene quaternion norm')
    w, x, y, z = [v/length for v in quaternion]
    cosine = min(1.0, abs(1 - 2*(x*x+y*y)))
    return {'xy_error_m': math.hypot(position[0]-target[0], position[1]-target[1]),
            'z_error_m': abs(position[2]-target[2]),
            'tilt_deg': math.degrees(math.acos(cosine))}


def first_attempt_passed(run):
    attempts = run.get('attempts')
    if not isinstance(attempts, list) or len(attempts) != 1 or not isinstance(attempts[0], dict):
        return False
    attempt = attempts[0]
    return (run.get('passed') is True and type(run.get('returncode')) is int and run['returncode'] == 0
            and type(attempt.get('attempt')) is int and attempt['attempt'] == 1
            and type(attempt.get('returncode')) is int and attempt['returncode'] == 0)


def analyze(records, report):
    if len(records) < 2:
        raise ValueError('Whole-scene audit requires at least two geometry samples')
    for record in records:
        if record.get('frame') != 'base_footprint' or record.get('unit') != 'm':
            raise ValueError('Scene records require base_footprint/metres')
        if set(record.get('blocks', {})) != BLOCKS:
            raise ValueError('Whole-scene audit requires all three block poses')
        if type(record.get('unix_seconds')) not in (int, float) or not math.isfinite(record['unix_seconds']):
            raise ValueError('Invalid scene time')
        for name in BLOCKS:
            pose_metrics(record['blocks'][name], records[0]['blocks'][name]['position_m'])
    times = [r['unix_seconds'] for r in records]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('Scene wall timestamps must increase')
    ended = report.get('created_at')
    if type(ended) not in (int, float) or not math.isfinite(ended):
        raise ValueError('Report requires finite completion epoch')
    max_gap = max(b-a for a, b in zip(times, times[1:]))
    if max_gap > MAX_OBSERVATION_GAP_SECONDS:
        raise ValueError('Whole-scene observation contains a logging gap')
    if not times[0] <= ended <= times[-1]+MAX_OBSERVATION_GAP_SECONDS:
        raise ValueError('Whole-scene observation does not cover job completion')
    runs = {r['object_id']: r for r in report['runs']}
    if len(runs) != len(report['runs']) or not set(runs) <= BLOCKS:
        raise ValueError('Report requires unique known block runs')
    results = {}
    for name in sorted(BLOCKS):
        run = runs.get(name)
        if run is None:
            target = records[0]['blocks'][name]['position_m']
            selected = records
            mode = 'unhandled_initial_pose'
        elif run.get('passed') is True:
            summary = run['motion_summary']
            release = summary.get('release_seconds')
            started = summary.get('monitoring', {}).get('started_at_unix')
            if any(type(v) not in (int, float) or not math.isfinite(v) for v in (release, started)):
                raise ValueError('Completed block requires finite release time and monitor epoch')
            if summary.get('release_detected') is not True or release < 0 or started+release > ended:
                raise ValueError('Completed block requires detected release before job completion')
            target = run['target']
            selected = [r for r in records if r['unix_seconds'] >= started+release]
            mode = 'completed_post_release'
        else:
            results[name] = {'mode': 'failed_run', 'passed': False}
            continue
        metrics = [pose_metrics(r['blocks'][name], target) for r in selected]
        worst = {k: max((m[k] for m in metrics), default=None)
                 for k in ('xy_error_m', 'z_error_m', 'tilt_deg')}
        passed = len(metrics) >= 2 and all(
            m['xy_error_m'] <= 0.020 and m['z_error_m'] <= 0.005 and m['tilt_deg'] <= 15
            for m in metrics)
        results[name] = {'mode': mode, 'sample_count': len(metrics), 'passed': passed,
                         'worst': worst, 'final': metrics[-1] if metrics else None}
    scene_passed = report['passed'] is True and all(r['passed'] for r in results.values())
    first_attempt_ok = report['passed'] is True and bool(runs) and all(
        first_attempt_passed(run) for run in runs.values())
    return {'job_passed': report['passed'] is True,
            'scene_passed': scene_passed,
            'first_attempt_passed': first_attempt_ok,
            'regression_passed': scene_passed and first_attempt_ok,
            'sample_count': len(records), 'max_geometry_gap_seconds': max_gap,
            'completion_coverage_seconds': times[-1]-ended,
            'max_observation_gap_seconds': MAX_OBSERVATION_GAP_SECONDS,
            'thresholds': {'xy_m': 0.020, 'z_m': 0.005, 'tilt_deg': 15},
            'blocks': results,
            'caution': '1 Hz sampled observation, not continuous contact evidence. Release alignment uses shared wall-clock epochs.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result_dir', type=Path)
    args = parser.parse_args()
    records = []
    for line in (args.result_dir/'simulation.log').read_text(errors='replace').splitlines():
        if 'PALLET_CONTACT_GEOMETRY ' in line:
            records.append(json.loads(line.split('PALLET_CONTACT_GEOMETRY ', 1)[1]))
    report = json.loads((args.result_dir/'job/report.json').read_text())
    result = analyze(records, report)
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['regression_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
