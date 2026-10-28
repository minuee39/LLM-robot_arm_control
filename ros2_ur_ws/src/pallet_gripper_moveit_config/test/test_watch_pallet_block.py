from pathlib import Path
import math
import json
import subprocess
import sys
import tempfile
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(PROJECT_ROOT / 'ros2_ur_ws/scripts'))
from watch_pallet_block import (
    arm_measurement, ContactReference, closest_joint_feedback, jaw_contact_state, rotate_vector_inverse,
    monitor_completion_allowed, monitor_end_reason,
    release_stability, RuntimeFaultDetector, SlipDetector, slip_event,
    tcp_relative_position)


class WatchPalletBlockTest(unittest.TestCase):
    def test_supervised_serialization_survives_sigint_and_preserves_error(self):
        # Inject a real SIGINT inside json.dumps in an isolated process, not
        # into the test runner. Exercise both normal failure and saved error.
        script_dir = str(PROJECT_ROOT / 'ros2_ur_ws/scripts')
        for has_error in (False, True):
            with tempfile.TemporaryDirectory() as temporary:
                output = Path(temporary) / 'motion.json'
                script = f'''
import json, os, signal, sys
from pathlib import Path
sys.path.insert(0, {script_dir!r})
from watch_pallet_block import protect_monitor_finalization, write_monitor_result
original_handler = signal.getsignal(signal.SIGINT)
original_dumps = json.dumps
def interrupted_dumps(*args, **kwargs):
    os.kill(os.getpid(), signal.SIGINT)
    return original_dumps(*args, **kwargs)
json.dumps = interrupted_dumps
protect_monitor_finalization(True)
error = RuntimeError('original monitor error') if {has_error!r} else None
try:
    code = write_monitor_result({{'placement_pass': False}}, [{{'seconds': 300}}],
                               Path({str(output)!r}), error, original_handler)
except RuntimeError as caught:
    assert error is caught
    code = 1
assert signal.getsignal(signal.SIGINT) == original_handler
raise SystemExit(code)
'''
                completed = subprocess.run([sys.executable, '-c', script],
                                           capture_output=True, text=True, timeout=10)
                self.assertEqual(completed.returncode, 1, completed.stderr)
                payload = json.loads(output.read_text())
                self.assertFalse(payload['summary']['placement_pass'])
                self.assertEqual(payload['samples'], [{'seconds': 300}])

    def test_monitor_duration_only_normal_without_supervision(self):
        self.assertEqual(monitor_end_reason(False, False, True, 300, 300),
                         'duration_complete')
        self.assertEqual(monitor_end_reason(True, False, True, 300, 300), 'timeout')
        self.assertTrue(monitor_completion_allowed(False, 'duration_complete'))
        self.assertFalse(monitor_completion_allowed(True, 'timeout'))

    def test_monitor_supervisor_stop_requires_interruption(self):
        self.assertEqual(monitor_end_reason(True, True, True, 1, 300), 'supervisor_stop')
        self.assertEqual(monitor_end_reason(False, True, True, 1, 300), 'interrupted')
        self.assertTrue(monitor_completion_allowed(True, 'supervisor_stop'))
        self.assertFalse(monitor_completion_allowed(True, 'duration_complete'))

    def test_monitor_shutdown_and_error_fail_closed(self):
        self.assertEqual(monitor_end_reason(True, False, False, 1, 300), 'ros_shutdown')
        self.assertEqual(monitor_end_reason(True, False, True, 1, 300, error=True), 'error')
        self.assertEqual(monitor_end_reason(True, False, True, 1, 300), 'error')
        for reason in ('error', 'ros_shutdown', 'interrupted', 'timeout'):
            self.assertFalse(monitor_completion_allowed(True, reason))

    def test_contact_reference_captures_moving_loaded_jaws(self):
        detector = ContactReference()
        self.assertIsNone(detector.update(1, [0, 0, 0], [1, 2, 3], 'bilateral_loaded'))
        self.assertIsNone(detector.update(2, [1, 0, 0], [1, 2, 3], 'bilateral_loaded'))
        # Gap above 0.5 seconds breaks confirmation.
        self.assertIsNone(detector.update(2.1, [2, 0, 0], [1, 2, 3], 'bilateral_loaded'))
        reference = detector.update(2.2, [3, 0, 0], [1, 2, 3], 'bilateral_loaded')
        self.assertEqual(reference['seconds'], 2)
        self.assertEqual(reference['relative'], [1, 0, 0])
        self.assertEqual(detector.update(3, [99, 0, 0], [0, 0, 0], 'not_loaded'), reference)

    def test_contact_reference_does_not_confirm_duplicates_or_bad_feedback(self):
        detector = ContactReference()
        self.assertIsNone(detector.update(1, [0, 0, 0], [0, 0, 0], 'bilateral_loaded'))
        self.assertIsNone(detector.update(1, [0, 0, 0], [0, 0, 0], 'bilateral_loaded'))
        self.assertIsNone(detector.update(1.1, None, [0, 0, 0], 'bilateral_loaded'))
        self.assertIsNone(detector.update(1.2, [float('nan'), 0, 0], [0, 0, 0], 'bilateral_loaded'))
        self.assertIsNone(detector.update(1.3, [0, 0, 0], [0, 0, 0], 'not_loaded'))
        self.assertIsNone(detector.update(1.4, [0, 0, 0], [0, 0, 0], 'bilateral_loaded'))
        self.assertIsNone(detector.update(1.5, [0, 0, 0], [0, 0, 0], 'bilateral_loaded'))
        self.assertEqual(detector.update(1.6, [0, 0, 0], [0, 0, 0], 'bilateral_loaded')['seconds'], 1.4)

    def test_arm_snapshot_requires_all_finite_positions(self):
        positions = {f'J{i}': i*0.1 for i in range(1, 7)}
        result = arm_measurement(positions, {}, {'J1': float('nan')})
        self.assertEqual(len(result), 6)
        self.assertEqual(result['J1']['position_rad'], 0.1)
        self.assertIsNone(result['J1']['velocity_rad_s'])
        self.assertIsNone(result['J1']['effort_nm'])
        self.assertEqual(arm_measurement(dict(positions, J3=float('nan')), {}, {}), {})
        del positions['J6']
        self.assertEqual(arm_measurement(positions, {}, {}), {})

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
        self.assertIsNone(detector.pose_stream_watchdog(2.76, 0.76, 0.75))
        event = detector.pose_stream_watchdog(3.0, 1.0, 0.75)
        self.assertEqual('TELEMETRY_STALE', event['code'])
        self.assertEqual(['block_pose_stream'], event['stale_measurements'])
        self.assertEqual(0.75, event['timeout_seconds'])
        self.assertEqual(2, event['confirm_samples'])

    def test_pose_stream_watchdog_resets_after_one_transient_gap(self):
        detector = RuntimeFaultDetector(
            [0.3, -0.3, 0.025], [0.03, 0.03, 0.05], confirm_samples=3)
        detector.update(self.fault_sample(1.0, [-0.3, 0.3, 0.060]))
        self.assertIsNone(detector.pose_stream_watchdog(1.76, 0.76, 0.75))
        self.assertIsNone(detector.pose_stream_watchdog(1.86, 0.10, 0.75))
        self.assertIsNone(detector.pose_stream_watchdog(2.62, 0.76, 0.75))
        self.assertIsNone(detector.pose_stream_watchdog(2.72, 0.86, 0.75))
        event = detector.pose_stream_watchdog(2.82, 0.96, 0.75)
        self.assertEqual('TELEMETRY_STALE', event['code'])
        self.assertEqual(3, event['confirm_samples'])

    def test_pose_stream_watchdog_requires_elapsed_confirmation_time(self):
        detector = RuntimeFaultDetector(
            [0.3, -0.3, 0.025], [0.03, 0.03, 0.05], confirm_samples=3)
        detector.update(self.fault_sample(1.0, [-0.3, 0.3, 0.060]))
        for seconds in (1.76, 1.78, 1.80):
            self.assertIsNone(detector.pose_stream_watchdog(
                seconds, seconds - 1.0, 0.75))
        event = detector.pose_stream_watchdog(1.91, 0.91, 0.75)
        self.assertEqual('TELEMETRY_STALE', event['code'])
        self.assertAlmostEqual(1.75, event['first_exceeded_seconds'])

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
