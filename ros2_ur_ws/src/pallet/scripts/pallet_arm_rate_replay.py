#!/usr/bin/env python3
"""Evaluate arm target spacing and serial TX budget from an ARM_CYCLE log."""
import argparse
import json
import math
import statistics

ARM_AXES = 6
PHYSICAL_AXES = 7
MIN_TX_SPACING_S = 0.003
FEEDBACK_HZ = 10
GRIPPER_HOLD_HZ = 10
READS_PER_AXIS = 3


def load_latest_motion(stream):
    motion = []
    previous_elapsed = math.inf
    for line in stream:
        if 'ARM_CYCLE ' not in line:
            continue
        row = json.loads(line.split('ARM_CYCLE ', 1)[1])
        elapsed = row['elapsed_s']
        if elapsed < previous_elapsed:
            motion.clear()
        previous_elapsed = elapsed
        if row['phase'] == 'motion':
            motion.append(row)
    if len(motion) < 2:
        raise ValueError('The latest motion needs at least two ARM_CYCLE records')
    return motion


def estimate(motion, arm_hz):
    # Consecutive recorded goals give a piecewise-linear replay. This estimates
    # command spacing, not motor response or the original MoveIt spline.
    intervals = []
    speeds = {}
    for previous, current in zip(motion, motion[1:]):
        dt = current['command_start_s'] - previous['command_start_s']
        if dt <= 0:
            raise ValueError('Command timestamps must increase')
        intervals.append(dt)
        goals = {axis['joint']: axis['desired_rad'] for axis in previous['axes']}
        for axis in current['axes']:
            name = axis['joint']
            speed = abs(axis['desired_rad'] - goals[name]) / dt
            speeds[name] = max(speeds.get(name, 0), speed)

    frames_per_s = (ARM_AXES * arm_hz + GRIPPER_HOLD_HZ
                    + PHYSICAL_AXES * READS_PER_AXIS * FEEDBACK_HZ)
    occupancy = frames_per_s * MIN_TX_SPACING_S
    return {
        'arm_command_hz': arm_hz,
        'motion_samples': len(motion),
        'recorded_command_median_ms': statistics.median(intervals) * 1000,
        'replay_max_step_deg': {name: math.degrees(speed / arm_hz)
                                for name, speed in speeds.items()},
        'tx_frames_per_s': frames_per_s,
        'minimum_tx_occupancy_pct': occupancy * 100,
        'over_tx_budget': occupancy > 1,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=argparse.FileType('r'))
    args = parser.parse_args()
    with args.log as stream:
        motion = load_latest_motion(stream)
    print(json.dumps([estimate(motion, hz) for hz in (10, 15, 20, 50)], indent=2))


if __name__ == '__main__':
    main()
