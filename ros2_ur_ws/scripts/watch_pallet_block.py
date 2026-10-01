#!/usr/bin/env python3
"""Bounded, read-only Isaac block-motion check; never commands robot motion."""

import argparse
from collections import deque
import json
import math
import os
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


TELEMETRY_MAX_AGE_SECONDS = 0.5
TELEMETRY_SYNC_DELAY_SECONDS = 0.15
CONTACT_EFFORT_N = 1.0
CONTACT_MAX_JAW_SPEED_M_S = 0.001
LOADED_CONTACT_STATES = {'bilateral_loaded', 'bilateral_stationary_loaded'}


def rotate_vector_inverse(vector, orientation):
    """Rotate a world-frame vector into the quaternion's local frame."""
    x, y, z, w = orientation
    q_vector = (x, y, z)
    dot_q_v = sum(a * b for a, b in zip(q_vector, vector))
    dot_q_q = sum(a * a for a in q_vector)
    cross_q_v = (
        y * vector[2] - z * vector[1],
        z * vector[0] - x * vector[2],
        x * vector[1] - y * vector[0],
    )
    return [
        2 * dot_q_v * q_vector[index]
        + (w * w - dot_q_q) * vector[index]
        - 2 * w * cross_q_v[index]
        for index in range(3)
    ]


def tcp_relative_position(tcp_position, tcp_orientation, block_position):
    """Return the block centre expressed in the TCP coordinate frame."""
    world_delta = [block_position[index] - tcp_position[index] for index in range(3)]
    return rotate_vector_inverse(world_delta, tcp_orientation)


def jaw_contact_state(jaws):
    """Classify bilateral simulated contact from one fresh joint snapshot."""
    if set(jaws) != {'J8', 'J9'}:
        return 'unavailable'
    efforts = [jaws[name].get('effort') for name in ('J8', 'J9')]
    velocities = [jaws[name].get('velocity') for name in ('J8', 'J9')]
    if any(value is None for value in efforts):
        return 'unavailable'
    if any(abs(value) < CONTACT_EFFORT_N for value in efforts):
        return 'not_loaded'
    if (any(value is None for value in velocities) or
            any(abs(value) > CONTACT_MAX_JAW_SPEED_M_S for value in velocities)):
        return 'bilateral_loaded'
    return 'bilateral_stationary_loaded'


def closest_joint_feedback(history, stamp_seconds):
    """Return the joint snapshot closest to a block-pose timestamp."""
    if not history:
        return None, None
    feedback = min(history, key=lambda item: abs(item['stamp_seconds'] - stamp_seconds))
    return feedback, abs(feedback['stamp_seconds'] - stamp_seconds)


class SlipDetector:
    """One-shot detector for persistent displacement under bilateral load."""

    def __init__(self, threshold, confirm_samples=3):
        if not math.isfinite(threshold) or threshold <= 0:
            raise ValueError('threshold must be finite and positive')
        if confirm_samples <= 0:
            raise ValueError('confirm_samples must be positive')
        self.threshold = threshold
        self.confirm_samples = confirm_samples
        self.consecutive = []
        self.event = None

    def update(self, sample):
        if self.event is not None:
            return None
        displacement = sample.get('relative_displacement_norm_m')
        contact_state = sample.get('joint_measurement', {}).get('contact_state')
        if (contact_state in LOADED_CONTACT_STATES and displacement is not None and
                displacement > self.threshold):
            self.consecutive.append(sample)
        else:
            self.consecutive.clear()
        if len(self.consecutive) >= self.confirm_samples:
            first = self.consecutive[0]
            self.event = {
                'seconds': sample['seconds'],
                'first_exceeded_seconds': first['seconds'],
                'confirmed_seconds': sample['seconds'],
                'relative_displacement_m': displacement,
                'contact_state': contact_state,
                'block_position_m': sample['position_m'],
                'threshold_m': self.threshold,
                'confirm_samples': self.confirm_samples,
            }
            return self.event
        return None


def slip_event(samples, threshold, confirm_samples=3):
    """Return the first debounced slip event in a sequence of samples."""
    detector = SlipDetector(threshold, confirm_samples)
    for sample in samples:
        event = detector.update(sample)
        if event is not None:
            return event
    return None


class RuntimeFaultDetector:
    """Detect a dropped/tilted block or stale carry telemetry with debounce."""

    def __init__(self, target, block_size, table_top_z=0.0,
                 drop_clearance_threshold=0.005, tilt_threshold_deg=20.0,
                 confirm_samples=3, target_xy_tolerance=0.05,
                 lifted_clearance=0.020):
        if len(target) != 3 or len(block_size) != 3:
            raise ValueError('target and block_size must each contain 3 values')
        if not math.isfinite(drop_clearance_threshold) or drop_clearance_threshold < 0:
            raise ValueError('drop_clearance_threshold must be finite and non-negative')
        if not math.isfinite(tilt_threshold_deg) or not 0 < tilt_threshold_deg < 90:
            raise ValueError('tilt_threshold_deg must be within (0, 90)')
        if confirm_samples <= 0:
            raise ValueError('confirm_samples must be positive')
        self.target = list(target)
        self.block_size = list(block_size)
        self.table_top_z = table_top_z
        self.drop_clearance_threshold = drop_clearance_threshold
        self.tilt_threshold_deg = tilt_threshold_deg
        self.confirm_samples = confirm_samples
        self.target_xy_tolerance = target_xy_tolerance
        self.lifted_clearance = lifted_clearance
        self.armed = False
        self.lift_confirmed = False
        self.released = False
        self.last_loaded = False
        self.release_count = 0
        self.counts = {
            'BLOCK_DROP_DETECTED': 0,
            'TILT_DETECTED': 0,
            'TELEMETRY_STALE': 0,
        }
        self.first_samples = {}
        self.event = None

    def pose_stream_watchdog(self, seconds, age_seconds, timeout_seconds):
        """Return a stale event when an armed pose stream stops completely."""
        if self.event is not None or self.released or not self.armed:
            return None
        if age_seconds < timeout_seconds:
            return None
        self.event = {
            'code': 'TELEMETRY_STALE',
            'seconds': seconds,
            'first_exceeded_seconds': seconds - age_seconds + timeout_seconds,
            'confirmed_seconds': seconds,
            'confirm_samples': 0,
            'block_position_m': None,
            'stale_measurements': ['block_pose_stream'],
            'pose_stream_age_seconds': age_seconds,
            'timeout_seconds': timeout_seconds,
        }
        return self.event

    def _debounce(self, code, condition, sample, details):
        if condition:
            self.counts[code] += 1
            self.first_samples.setdefault(code, sample)
        else:
            self.counts[code] = 0
            self.first_samples.pop(code, None)
        if self.counts[code] < self.confirm_samples:
            return None
        first = self.first_samples[code]
        self.event = {
            'code': code,
            'seconds': sample['seconds'],
            'first_exceeded_seconds': first['seconds'],
            'confirmed_seconds': sample['seconds'],
            'confirm_samples': self.confirm_samples,
            'block_position_m': sample['position_m'],
            **details,
        }
        return self.event

    def update(self, sample):
        if self.event is not None or self.released:
            return None
        contact_state = sample.get('joint_measurement', {}).get('contact_state')
        loaded = contact_state in LOADED_CONTACT_STATES
        if loaded:
            self.armed = True
            self.last_loaded = True

        position = sample.get('position_m')
        orientation = sample.get('orientation_xyzw')
        near_target = (
            position is not None and
            math.dist(position[:2], self.target[:2]) <= self.target_xy_tolerance)
        if self.armed and near_target and contact_state == 'not_loaded':
            self.release_count += 1
            if self.release_count >= self.confirm_samples:
                self.released = True
                return None
        elif loaded or not near_target:
            self.release_count = 0

        if not self.armed or position is None or orientation is None:
            return None
        clearance = block_bottom_z(position, orientation, self.block_size) - self.table_top_z
        if clearance >= self.lifted_clearance:
            self.lift_confirmed = True

        joint_status = sample.get('joint_measurement', {}).get('status')
        tcp_status = sample.get('tcp_measurement', {}).get('status')
        stale_statuses = [name for name, status in (
            ('joint', joint_status), ('tcp', tcp_status)) if status != 'fresh']
        event = self._debounce(
            'TELEMETRY_STALE', self.last_loaded and bool(stale_statuses), sample,
            {'stale_measurements': stale_statuses})
        if event is not None:
            return event

        event = self._debounce(
            'BLOCK_DROP_DETECTED',
            self.lift_confirmed and not near_target and
            clearance < self.drop_clearance_threshold,
            sample, {
                'bottom_clearance_m': clearance,
                'threshold_m': self.drop_clearance_threshold,
            })
        if event is not None:
            return event

        vertical_cosine = abs(1 - 2 * (
            orientation[0] * orientation[0] + orientation[1] * orientation[1]))
        return self._debounce(
            'TILT_DETECTED', self.lift_confirmed and
            vertical_cosine < math.cos(math.radians(self.tilt_threshold_deg)),
            sample, {
                'long_axis_vertical_cosine': vertical_cosine,
                'threshold_deg': self.tilt_threshold_deg,
            })


def release_stability(samples, target, confirm_samples=3, target_xy_tolerance=0.03):
    """Summarize release and any later bilateral jaw re-contact near the target."""
    loaded_states = {'bilateral_loaded', 'bilateral_stationary_loaded'}
    loaded_near_target = False
    unladen = []
    release_index = None
    for index, sample in enumerate(samples):
        state = sample.get('joint_measurement', {}).get('contact_state')
        position = sample.get('position_m')
        near_target = (
            position is not None and
            math.dist(position[:2], target[:2]) <= target_xy_tolerance)
        if state in loaded_states and near_target:
            loaded_near_target = True
            unladen.clear()
        elif loaded_near_target and state == 'not_loaded':
            unladen.append(index)
            if len(unladen) >= confirm_samples:
                release_index = unladen[0]
                break
        elif loaded_near_target:
            unladen.clear()

    if release_index is None:
        return {
            'release_detected': False,
            'release_seconds': None,
            'post_release_recontact_detected': None,
            'post_release_recontact_seconds': None,
            'max_post_release_displacement_m': None,
        }

    release_sample = samples[release_index]
    release_position = release_sample['position_m']
    post_release = samples[release_index + confirm_samples:]
    recontacts = [
        sample for sample in post_release
        if sample.get('joint_measurement', {}).get('contact_state') in loaded_states
    ]
    displacements = [
        math.dist(sample['position_m'], release_position)
        for sample in samples[release_index:]
        if sample.get('position_m') is not None
    ]
    return {
        'release_detected': True,
        'release_seconds': release_sample['seconds'],
        'post_release_recontact_detected': bool(recontacts),
        'post_release_recontact_seconds': recontacts[0]['seconds'] if recontacts else None,
        'max_post_release_displacement_m': max(displacements, default=0.0),
    }


def block_bottom_z(position, orientation, size):
    """Lowest world-Z corner of a box at the measured centre and orientation."""
    x, y, z, w = orientation
    world_z_x = 2 * (x * z - w * y)
    world_z_y = 2 * (y * z + w * x)
    world_z_z = 1 - 2 * (x * x + y * y)
    half_span = sum(abs(axis) * dimension for axis, dimension in zip(
        (world_z_x, world_z_y, world_z_z), size)) / 2
    return position[2] - half_span


def transfer_clearance(samples, target, block_size, table_top_z):
    """Report the lowest box corner through the middle 80% of XY transport."""
    if not samples:
        return None, 0
    start = samples[0]['position_m']
    travel = [target[i] - start[i] for i in (0, 1)]
    distance_squared = sum(axis * axis for axis in travel)
    if distance_squared < 0.01:  # A final-only observation has no transfer segment.
        return None, 0
    bottoms = []
    for sample in samples:
        position = sample['position_m']
        progress = sum((position[i] - start[i]) * travel[i] for i in (0, 1)) / distance_squared
        if 0.1 <= progress <= 0.9:
            bottoms.append(block_bottom_z(
                position, sample['orientation_xyzw'], block_size) - table_top_z)
    return (min(bottoms) if bottoms else None), len(bottoms)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=180.0)
    parser.add_argument('--target', nargs=3, type=float, default=[0.30, -0.30, 0.025])
    parser.add_argument('--tolerance', type=float, default=0.02)
    parser.add_argument('--height-tolerance', type=float, default=0.005)
    parser.add_argument('--upright-tolerance-deg', type=float, default=15.0)
    parser.add_argument('--block-size', nargs=3, type=float, default=[0.03, 0.03, 0.05])
    parser.add_argument('--table-top-z', type=float, default=0.0)
    parser.add_argument('--slip-threshold', type=float, default=0.005)
    parser.add_argument('--slip-confirm-samples', type=int, default=3)
    parser.add_argument('--drop-clearance-threshold', type=float, default=0.005)
    parser.add_argument('--tilt-threshold-deg', type=float, default=20.0)
    parser.add_argument('--fault-confirm-samples', type=int, default=3)
    parser.add_argument('--pose-stream-timeout', type=float, default=0.75)
    parser.add_argument(
        '--inject-safety-code', default='SLIP_DETECTED', choices=(
            'SLIP_DETECTED', 'BLOCK_DROP_DETECTED', 'TILT_DETECTED',
            'TELEMETRY_STALE'))
    parser.add_argument(
        '--inject-safety-event-after-contact', type=float, default=-1.0,
        help='test-only: publish --inject-safety-code this many seconds after contact')
    parser.add_argument(
        '--inject-sim-fault-phase', default='none', choices=(
            'none', 'after_contact', 'after_lift', 'transfer', 'lower',
            'pre_release'),
        help='test-only: ask Isaac to create a physical fault at this motion phase')
    parser.add_argument(
        '--inject-sim-fault-code', default='BLOCK_DROP_DETECTED', choices=(
            'BLOCK_DROP_DETECTED', 'TILT_DETECTED', 'TELEMETRY_STALE'))
    parser.add_argument('--inject-sim-fault-delay', type=float, default=0.0)
    parser.add_argument('--event-output', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be finite and positive')
    if (not all(map(math.isfinite, args.target)) or not 0 < args.tolerance < 1 or
            not 0 < args.height_tolerance < 1 or
            not 0 < args.upright_tolerance_deg < 90 or
            not all(math.isfinite(size) and size > 0 for size in args.block_size) or
            not math.isfinite(args.table_top_z) or
            not math.isfinite(args.slip_threshold) or not 0 < args.slip_threshold < 1 or
            not math.isfinite(args.drop_clearance_threshold) or
            not 0 <= args.drop_clearance_threshold < 1 or
            not math.isfinite(args.tilt_threshold_deg) or
            not 0 < args.tilt_threshold_deg < 90 or
            not math.isfinite(args.pose_stream_timeout) or
            not 0.1 <= args.pose_stream_timeout <= 10.0):
        parser.error('target/tolerances must be finite and in their valid ranges')
    if args.slip_confirm_samples <= 0 or args.fault_confirm_samples <= 0:
        parser.error('confirmation sample counts must be positive')
    if (not math.isfinite(args.inject_safety_event_after_contact) or
            args.inject_safety_event_after_contact < -1.0):
        parser.error('--inject-safety-event-after-contact must be -1 (off) or non-negative')
    if not math.isfinite(args.inject_sim_fault_delay) or args.inject_sim_fault_delay < 0:
        parser.error('--inject-sim-fault-delay must be finite and non-negative')
    rclpy.init()
    node = rclpy.create_node('pallet_block_motion_check')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node, spin_thread=False)
    event_qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    safety_event_pub = node.create_publisher(String, '/pallet/safety_events', event_qos)
    sim_fault_pub = node.create_publisher(String, '/pallet/sim_fault_commands', event_qos)
    slip_detector = SlipDetector(args.slip_threshold, args.slip_confirm_samples)
    fault_detector = RuntimeFaultDetector(
        args.target, args.block_size, args.table_top_z,
        args.drop_clearance_threshold, args.tilt_threshold_deg,
        args.fault_confirm_samples)
    started = time.monotonic()
    samples = []
    joint_history = deque(maxlen=400)
    pending_poses = deque()
    contact_reference = None
    contact_reference_seconds = None
    published_safety_event = None
    published_sim_fault = None
    loaded_source_position = None
    phase_ready_since = None
    last_pose_received = None
    last_printed = None

    def publish_safety_event(code, event, injected=False):
        nonlocal published_safety_event
        if published_safety_event is not None:
            return
        payload = {
            'event_id': f'watch-pallet-block-{os.getpid()}-{time.monotonic_ns()}',
            'code': code,
            'severity': 'critical',
            'source': 'watch_pallet_block',
            'frame': 'base_footprint',
            'published_at_unix': time.time(),
            'injected': injected,
            **event,
        }
        published_safety_event = payload
        safety_event_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))
        if args.event_output:
            args.event_output.write_text(json.dumps(payload, indent=2) + '\n')
        node.get_logger().error(
            f'Published {"injected " if injected else ""}{code} safety event')

    def on_joints(msg):
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity)) if len(msg.velocity) == len(msg.name) else {}
        efforts = dict(zip(msg.name, msg.effort)) if len(msg.effort) == len(msg.name) else {}
        if not all(name in positions and math.isfinite(positions[name]) for name in ('J8', 'J9')):
            return
        jaws = {}
        for name in ('J8', 'J9'):
            velocity = velocities.get(name)
            effort = efforts.get(name)
            jaws[name] = {
                'position': positions[name],
                'velocity': velocity if velocity is not None and math.isfinite(velocity) else None,
                'effort': effort if effort is not None and math.isfinite(effort) else None,
            }
        stamp = Time.from_msg(msg.header.stamp)
        joint_history.append({
            'stamp_seconds': stamp.nanoseconds / 1e9,
            'received': time.monotonic(),
            'jaws': jaws,
        })

    def on_pose(msg):
        nonlocal last_pose_received
        p = msg.pose.position
        position = [p.x, p.y, p.z]
        if msg.header.frame_id != 'base_footprint' or not all(map(math.isfinite, position)):
            node.get_logger().error('Rejected invalid block pose or frame')
            return
        q = msg.pose.orientation
        orientation = [q.x, q.y, q.z, q.w]
        if not all(map(math.isfinite, orientation)) or abs(math.sqrt(sum(v*v for v in orientation)) - 1) > 0.01:
            node.get_logger().error('Rejected invalid block orientation')
            return
        stamp = Time.from_msg(msg.header.stamp)
        pending_poses.append({
            'received': time.monotonic(),
            'seconds': time.monotonic() - started,
            'stamp': stamp,
            'stamp_seconds': stamp.nanoseconds / 1e9,
            'position': position,
            'orientation': orientation,
        })
        last_pose_received = time.monotonic()

    def process_pose(pose):
        nonlocal contact_reference, contact_reference_seconds, last_printed
        nonlocal loaded_source_position, phase_ready_since, published_sim_fault
        position = pose['position']
        orientation = pose['orientation']
        feedback, feedback_alignment = closest_joint_feedback(
            joint_history, pose['stamp_seconds'])
        feedback_fresh = (
            feedback_alignment is not None and
            feedback_alignment <= TELEMETRY_MAX_AGE_SECONDS)
        jaws = feedback['jaws'] if feedback_fresh else {}
        contact_state = jaw_contact_state(jaws) if feedback_fresh else (
            'stale' if feedback else 'unavailable')
        joint_measurement = {
            'status': 'fresh' if feedback_fresh else ('stale' if feedback else 'unavailable'),
            'age_seconds': feedback_alignment,
            'alignment_error_seconds': feedback_alignment,
            'jaws': jaws,
            'contact_state': contact_state,
        }

        tcp_measurement = {'status': 'unavailable'}
        relative = None
        try:
            transform = tf_buffer.lookup_transform('base_footprint', 'tcp', pose['stamp'])
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            tcp_position = [translation.x, translation.y, translation.z]
            tcp_orientation = [rotation.x, rotation.y, rotation.z, rotation.w]
            stamp = Time.from_msg(transform.header.stamp)
            transform_alignment = abs(stamp.nanoseconds / 1e9 - pose['stamp_seconds'])
            transform_fresh = (
                stamp.nanoseconds > 0 and
                transform_alignment <= TELEMETRY_MAX_AGE_SECONDS)
            tcp_measurement = {
                'status': 'fresh' if transform_fresh else 'stale',
                'age_seconds': transform_alignment,
                'alignment_error_seconds': transform_alignment,
                'position_m': tcp_position,
                'orientation_xyzw': tcp_orientation,
            }
            if transform_fresh:
                relative = tcp_relative_position(tcp_position, tcp_orientation, position)
        except TransformException:
            pass

        if (contact_reference is None and relative is not None and
                contact_state == 'bilateral_stationary_loaded'):
            contact_reference = list(relative)
            contact_reference_seconds = pose['seconds']
            loaded_source_position = list(position)
        displacement = (
            [relative[index] - contact_reference[index] for index in range(3)]
            if relative is not None and contact_reference is not None else None)
        sample = {
            'seconds': pose['seconds'],
            'position_m': position,
            'orientation_xyzw': orientation,
            # Preserve the original compact field for existing report readers.
            'jaw_m': {name: data['position'] for name, data in jaws.items()},
            'joint_measurement': joint_measurement,
            'tcp_measurement': tcp_measurement,
            'tcp_to_block_m': relative,
            'relative_displacement_from_contact_m': displacement,
            'relative_displacement_norm_m': math.dist(displacement, [0.0, 0.0, 0.0])
            if displacement is not None else None,
        }
        samples.append(sample)
        fault_event = fault_detector.update(sample)
        if fault_event is not None:
            publish_safety_event(fault_event['code'], fault_event)
        event = slip_detector.update(sample)
        if event is not None:
            publish_safety_event('SLIP_DETECTED', event)

        bottom_clearance = block_bottom_z(position, orientation, args.block_size) - args.table_top_z
        near_target = math.dist(position[:2], args.target[:2]) <= 0.05
        transfer_progress = None
        if loaded_source_position is not None:
            travel = [args.target[i] - loaded_source_position[i] for i in (0, 1)]
            distance_squared = sum(axis * axis for axis in travel)
            if distance_squared > 1e-9:
                transfer_progress = sum(
                    (position[i] - loaded_source_position[i]) * travel[i]
                    for i in (0, 1)) / distance_squared
        phase_ready = {
            'none': False,
            'after_contact': contact_reference_seconds is not None,
            'after_lift': fault_detector.lift_confirmed,
            'transfer': (fault_detector.lift_confirmed and transfer_progress is not None and
                         0.25 <= transfer_progress <= 0.75),
            'lower': (near_target and contact_state in LOADED_CONTACT_STATES and
                      bottom_clearance > 0.010),
            'pre_release': (near_target and contact_state in LOADED_CONTACT_STATES and
                            bottom_clearance <= 0.010),
        }[args.inject_sim_fault_phase]
        if phase_ready:
            phase_ready_since = phase_ready_since or pose['seconds']
        else:
            phase_ready_since = None
        if (published_sim_fault is None and phase_ready_since is not None and
                pose['seconds'] - phase_ready_since >= args.inject_sim_fault_delay):
            published_sim_fault = {
                'fault_id': f'watch-pallet-block-{os.getpid()}-{time.monotonic_ns()}',
                'code': args.inject_sim_fault_code,
                'phase': args.inject_sim_fault_phase,
                'published_at_unix': time.time(),
            }
            sim_fault_pub.publish(String(data=json.dumps(
                published_sim_fault, separators=(',', ':'))))
            node.get_logger().warn(
                f'Published test-only Isaac fault command {args.inject_sim_fault_code} '
                f'at phase {args.inject_sim_fault_phase}')
        if (args.inject_safety_event_after_contact >= 0.0 and
                contact_reference_seconds is not None and
                pose['seconds'] - contact_reference_seconds >=
                args.inject_safety_event_after_contact):
            publish_safety_event(args.inject_safety_code, {
                'seconds': pose['seconds'],
                'first_exceeded_seconds': pose['seconds'],
                'confirmed_seconds': pose['seconds'],
                'relative_displacement_m': sample['relative_displacement_norm_m'],
                'contact_state': contact_state,
                'block_position_m': position,
                'threshold_m': args.slip_threshold,
                'confirm_samples': 0,
            }, injected=True)
        if last_printed is None or math.dist(position, last_printed) >= 0.005:
            print(json.dumps(sample), flush=True)
            last_printed = position

    def process_pending(force=False):
        now = time.monotonic()
        while pending_poses and (
                force or now - pending_poses[0]['received'] >= TELEMETRY_SYNC_DELAY_SECONDS):
            process_pose(pending_poses.popleft())

    node.create_subscription(PoseStamped, '/isaac_calibration_block_pose', on_pose, 10)
    node.create_subscription(JointState, '/isaac_joint_states', on_joints, qos_profile_sensor_data)
    try:
        while rclpy.ok() and time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
            process_pending()
            now = time.monotonic()
            if last_pose_received is not None:
                event = fault_detector.pose_stream_watchdog(
                    now - started, now - last_pose_received,
                    args.pose_stream_timeout)
                if event is not None:
                    publish_safety_event(event['code'], event)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        # rclpy can raise from take_message while the launch graph is tearing
        # down after a safety cancellation. Preserve the already captured
        # measurements and event instead of losing the experiment summary.
        if rclpy.ok() and 'Unable to convert call argument' not in str(error):
            raise
        node.get_logger().warn(f'Monitor stopped during ROS teardown: {error}')
    finally:
        process_pending(force=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    del tf_listener
    final_position = samples[-1]['position_m'] if samples else None
    xy_error = math.dist(final_position[:2], args.target[:2]) if samples else None
    height_error = abs(final_position[2] - args.target[2]) if samples else None
    q = samples[-1]['orientation_xyzw'] if samples else None
    long_axis_vertical = abs(1 - 2 * (q[0] * q[0] + q[1] * q[1])) if q else None
    upright = long_axis_vertical is not None and long_axis_vertical >= math.cos(
        math.radians(args.upright_tolerance_deg))
    fresh = bool(samples) and time.monotonic() - started - samples[-1]['seconds'] < 1.0
    clearance, transfer_count = transfer_clearance(
        samples, args.target, args.block_size, args.table_top_z)
    contact_displacements = [
        sample['relative_displacement_norm_m'] for sample in samples
        if sample['joint_measurement']['contact_state'] in {
            'bilateral_loaded', 'bilateral_stationary_loaded'}
        and sample['relative_displacement_norm_m'] is not None
    ]
    loaded_jaw_asymmetries = [
        abs(sample['jaw_m']['J8'] - sample['jaw_m']['J9']) for sample in samples
        if sample['joint_measurement']['contact_state'] in {
            'bilateral_loaded', 'bilateral_stationary_loaded'}
        and set(sample['jaw_m']) == {'J8', 'J9'}
    ]
    slip = slip_detector.event
    release = release_stability(samples, args.target)
    result = {
        'frame': 'base_footprint', 'unit': 'm', 'sample_count': len(samples),
        'target': args.target, 'final_position': final_position,
        'final_orientation_xyzw': q,
        'max_height': max(s['position_m'][2] for s in samples) if samples else None,
        'transfer_min_bottom_clearance': clearance,
        'transfer_sample_count': transfer_count,
        'final_xy_error': xy_error, 'final_height_error': height_error,
        'long_axis_vertical_cosine': long_axis_vertical, 'fresh_feedback': fresh,
        'telemetry': {
            'tcp_fresh_sample_count': sum(
                sample['tcp_measurement']['status'] == 'fresh' for sample in samples),
            'joint_fresh_sample_count': sum(
                sample['joint_measurement']['status'] == 'fresh' for sample in samples),
            'contact_reference_available': contact_reference is not None,
        },
        'max_contact_relative_displacement_m': (
            max(contact_displacements) if contact_displacements else None),
        'max_loaded_jaw_asymmetry_m': (
            max(loaded_jaw_asymmetries) if loaded_jaw_asymmetries else None),
        'slip_threshold_m': args.slip_threshold,
        'slip_confirm_samples': args.slip_confirm_samples,
        'slip_detected': slip is not None,
        'slip_event': slip,
        'runtime_fault_thresholds': {
            'drop_clearance_m': args.drop_clearance_threshold,
            'tilt_deg': args.tilt_threshold_deg,
            'confirm_samples': args.fault_confirm_samples,
            'pose_stream_timeout_seconds': args.pose_stream_timeout,
        },
        'runtime_fault_detected': fault_detector.event is not None,
        'runtime_fault_event': fault_detector.event,
        'safety_event_published': published_safety_event is not None,
        'safety_event': published_safety_event,
        'sim_fault_command_published': published_sim_fault is not None,
        'sim_fault_command': published_sim_fault,
        **release,
        'placement_pass': (fresh and xy_error <= args.tolerance and
                           height_error <= args.height_tolerance and upright),
    }
    if args.output:
        args.output.write_text(json.dumps({'summary': result, 'samples': samples}, indent=2) + '\n')
    print(json.dumps(result), flush=True)
    return 0 if result['placement_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
