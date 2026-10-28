from pathlib import Path
import math
import sys
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(PROJECT_ROOT / 'ros2_ur_ws/scripts'))
from watch_pallet_block import (
    closest_joint_feedback, jaw_contact_state, rotate_vector_inverse,
    release_stability, RuntimeFaultDetector, SlipDetector, slip_event,
    tcp_relative_position)


class WatchPalletBlockTest(unittest.TestCase):
    @staticmethod
    def fault_sample(seconds, position, state='bilateral_loaded',
                     orientation=(0.0, 0.0, 0.0, 1.0),
                     joint_status='fresh', tcp_status='fresh'):
        return {
            'seconds': seconds,
            'position_m': list(position),
            'orientation_xyzw': list(orientation),
            'joint_measurement': {
                'status': joint_status,
                'contact_state': state,
            },
            'tcp_measurement': {'status': tcp_status},
        }

    def test_inverse_rotation_and_tcp_relative_position(self):
        half = math.sqrt(0.5)
        orientation = [0.0, 0.0, half, half]
        rotated = rotate_vector_inverse([1.0, 0.0, 0.0], orientation)
        self.assertAlmostEqual(0.0, rotated[0], places=7)
        self.assertAlmostEqual(-1.0, rotated[1], places=7)
        self.assertAlmostEqual(0.0, rotated[2], places=7)
        relative = tcp_relative_position([1.0, 2.0, 3.0], orientation, [2.0, 2.0, 3.0])
        self.assertAlmostEqual(0.0, relative[0], places=7)
        self.assertAlmostEqual(-1.0, relative[1], places=7)

    def test_contact_state_requires_bilateral_fresh_load(self):
        loaded = {
            'J8': {'position': 0.01, 'velocity': 0.0, 'effort': 2.0},
            'J9': {'position': 0.01, 'velocity': 0.0, 'effort': 2.5},
        }
        self.assertEqual('bilateral_stationary_loaded', jaw_contact_state(loaded))
        moving = {name: dict(values) for name, values in loaded.items()}
        moving['J9']['velocity'] = 0.002
        self.assertEqual('bilateral_loaded', jaw_contact_state(moving))
        one_sided = {name: dict(values) for name, values in loaded.items()}
        one_sided['J8']['effort'] = 0.5
        self.assertEqual('not_loaded', jaw_contact_state(one_sided))
        missing_effort = {name: dict(values) for name, values in loaded.items()}
        missing_effort['J8']['effort'] = None
        self.assertEqual('unavailable', jaw_contact_state(missing_effort))

    def test_closest_joint_feedback_uses_message_time(self):
        history = [
            {'stamp_seconds': 10.0, 'jaws': {}},
            {'stamp_seconds': 10.1, 'jaws': {'J8': {}}},
            {'stamp_seconds': 10.2, 'jaws': {'J9': {}}},
        ]
        feedback, alignment = closest_joint_feedback(history, 10.13)
        self.assertEqual(10.1, feedback['stamp_seconds'])
        self.assertAlmostEqual(0.03, alignment)
        self.assertEqual((None, None), closest_joint_feedback([], 10.0))

    def test_slip_event_only_uses_loaded_contact(self):
        def sample(seconds, displacement, state):
            return {
                'seconds': seconds,
                'position_m': [0.0, 0.0, 0.05],
                'relative_displacement_norm_m': displacement,
                'joint_measurement': {'contact_state': state},
            }

        samples = [
            sample(1.0, 0.020, 'not_loaded'),
            sample(2.0, 0.004, 'bilateral_stationary_loaded'),
            sample(3.0, 0.006, 'bilateral_loaded'),
            sample(4.0, 0.007, 'bilateral_stationary_loaded'),
            sample(5.0, 0.008, 'bilateral_loaded'),
        ]
        event = slip_event(samples, 0.005)
        self.assertEqual(5.0, event['seconds'])
        self.assertEqual(3.0, event['first_exceeded_seconds'])
        self.assertEqual(3, event['confirm_samples'])
        self.assertEqual('bilateral_loaded', event['contact_state'])
        self.assertEqual(0.005, event['threshold_m'])
        self.assertIsNone(slip_event(samples[:4], 0.005))
        with self.assertRaises(ValueError):
            slip_event(samples, 0.005, confirm_samples=0)

    def test_slip_detector_publishes_only_one_event(self):
        detector = SlipDetector(0.005, confirm_samples=2)
        samples = [
            {
                'seconds': value,
                'position_m': [0.0, 0.0, 0.05],
                'relative_displacement_norm_m': 0.006,
                'joint_measurement': {'contact_state': 'bilateral_loaded'},
            }
            for value in (1.0, 2.0, 3.0)
        ]
        self.assertIsNone(detector.update(samples[0]))
        self.assertIsNotNone(detector.update(samples[1]))
        self.assertIsNone(detector.update(samples[2]))
        self.assertEqual(2.0, detector.event['confirmed_seconds'])

    def test_runtime_drop_requires_prior_lift_and_debounce(self):
        detector = RuntimeFaultDetector(
            [0.3, -0.3, 0.025], [0.03, 0.03, 0.05], confirm_samples=3)
        low = self.fault_sample(1.0, [-0.3, 0.3, 0.025])
        self.assertIsNone(detector.update(low))
        self.assertIsNone(detector.update(self.fault_sample(2.0, [-0.3, 0.3, 0.060])))
        self.assertTrue(detector.lift_confirmed)
        self.assertIsNone(detector.update(self.fault_sample(3.0, [-0.1, 0.1, 0.028])))
        self.assertIsNone(detector.update(self.fault_sample(4.0, [-0.1, 0.1, 0.028])))
        event = detector.update(self.fault_sample(5.0, [-0.1, 0.1, 0.028]))
        self.assertEqual('BLOCK_DROP_DETECTED', event['code'])
        self.assertAlmostEqual(0.003, event['bottom_clearance_m'])

    def test_runtime_tilt_is_debounced_after_lift(self):
        detector = RuntimeFaultDetector(
            [0.3, -0.3, 0.025], [0.03, 0.03, 0.05],
            tilt_threshold_deg=20.0, confirm_samples=2)
        detector.update(self.fault_sample(1.0, [-0.3, 0.3, 0.060]))
        half_angle = math.radians(30.0) / 2
        tilted = [math.sin(half_angle), 0.0, 0.0, math.cos(half_angle)]
        self.assertIsNone(detector.update(
            self.fault_sample(2.0, [-0.2, 0.2, 0.060], orientation=tilted)))
        event = detector.update(
            self.fault_sample(3.0, [-0.1, 0.1, 0.060], orientation=tilted))
        self.assertEqual('TILT_DETECTED', event['code'])
        self.assertAlmostEqual(math.cos(math.radians(30.0)),
                               event['long_axis_vertical_cosine'])

    def test_runtime_stale_is_armed_by_load_and_debounced(self):
        detector = RuntimeFaultDetector(
            [0.3, -0.3, 0.025], [0.03, 0.03, 0.05], confirm_samples=3)
        unarmed = self.fault_sample(
            1.0, [-0.3, 0.3, 0.025], state='unavailable', tcp_status='stale')
        self.assertIsNone(detector.update(unarmed))
        detector.update(self.fault_sample(2.0, [-0.3, 0.3, 0.060]))
        for seconds in (3.0, 4.0):
            self.assertIsNone(detector.update(self.fault_sample(
                seconds, [-0.2, 0.2, 0.060], state='stale',
                joint_status='stale', tcp_status='stale')))
        event = detector.update(self.fault_sample(
            5.0, [-0.2, 0.2, 0.060], state='stale',
            joint_status='stale', tcp_status='stale'))
        self.assertEqual('TELEMETRY_STALE', event['code'])
        self.assertEqual(['joint', 'tcp'], event['stale_measurements'])

    def test_runtime_detector_disarms_after_confirmed_target_release(self):
        detector = RuntimeFaultDetector(
            [0.3, -0.3, 0.025], [0.03, 0.03, 0.05], confirm_samples=2)
        detector.update(self.fault_sample(1.0, [0.0, 0.0, 0.060]))
        for seconds in (2.0, 3.0):
            self.assertIsNone(detector.update(self.fault_sample(
                seconds, [0.3, -0.3, 0.025], state='not_loaded')))
        self.assertTrue(detector.released)
        self.assertIsNone(detector.update(self.fault_sample(
            4.0, [0.3, -0.3, 0.025], state='stale',
            joint_status='stale', tcp_status='stale')))

    def test_pose_stream_watchdog_only_triggers_while_armed(self):
        detector = RuntimeFaultDetector(
            [0.3, -0.3, 0.025], [0.03, 0.03, 0.05], confirm_samples=2)
        self.assertIsNone(detector.pose_stream_watchdog(1.0, 2.0, 0.75))
        detector.update(self.fault_sample(2.0, [-0.3, 0.3, 0.060]))
        self.assertIsNone(detector.pose_stream_watchdog(2.5, 0.5, 0.75))
        event = detector.pose_stream_watchdog(3.0, 1.0, 0.75)
        self.assertEqual('TELEMETRY_STALE', event['code'])
        self.assertEqual(['block_pose_stream'], event['stale_measurements'])
        self.assertEqual(0.75, event['timeout_seconds'])

    def test_pose_stream_watchdog_stays_disarmed_after_release(self):
        detector = RuntimeFaultDetector(
            [0.3, -0.3, 0.025], [0.03, 0.03, 0.05], confirm_samples=2)
        detector.update(self.fault_sample(1.0, [0.0, 0.0, 0.060]))
        detector.update(self.fault_sample(
            2.0, [0.3, -0.3, 0.025], state='not_loaded'))
        detector.update(self.fault_sample(
            3.0, [0.3, -0.3, 0.025], state='not_loaded'))
        self.assertIsNone(detector.pose_stream_watchdog(5.0, 2.0, 0.75))

    def test_release_stability_detects_release_without_recontact(self):
        def sample(seconds, state, x=0.3):
            return {
                'seconds': seconds,
                'position_m': [x, -0.3, 0.025],
                'joint_measurement': {'contact_state': state},
            }

        samples = [
            sample(1.0, 'not_loaded', x=-0.2),
            sample(2.0, 'bilateral_stationary_loaded'),
            sample(3.0, 'not_loaded'),
            sample(4.0, 'not_loaded'),
            sample(5.0, 'not_loaded'),
            sample(6.0, 'not_loaded', x=0.302),
        ]
        result = release_stability(samples, [0.3, -0.3, 0.025])
        self.assertTrue(result['release_detected'])
        self.assertEqual(3.0, result['release_seconds'])
        self.assertFalse(result['post_release_recontact_detected'])
        self.assertAlmostEqual(0.002, result['max_post_release_displacement_m'])

    def test_release_stability_debounces_release_and_reports_recontact(self):
        def sample(seconds, state):
            return {
                'seconds': seconds,
                'position_m': [0.3, -0.3, 0.025],
                'joint_measurement': {'contact_state': state},
            }

        samples = [
            sample(1.0, 'bilateral_loaded'),
            sample(2.0, 'not_loaded'),
            sample(3.0, 'bilateral_loaded'),
            sample(4.0, 'not_loaded'),
            sample(5.0, 'not_loaded'),
            sample(6.0, 'not_loaded'),
            sample(7.0, 'bilateral_loaded'),
        ]
        result = release_stability(samples, [0.3, -0.3, 0.025])
        self.assertEqual(4.0, result['release_seconds'])
        self.assertTrue(result['post_release_recontact_detected'])
        self.assertEqual(7.0, result['post_release_recontact_seconds'])


if __name__ == '__main__':
    unittest.main()
