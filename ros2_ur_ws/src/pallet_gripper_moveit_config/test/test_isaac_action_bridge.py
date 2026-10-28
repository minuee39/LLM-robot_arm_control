from pathlib import Path
import json
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from isaac_action_bridge import SAFETY_STOP_CODES, SafetyStop, StableTarget


class StableTargetTest(unittest.TestCase):
    def test_requires_continuous_low_velocity_feedback(self):
        target = StableTarget(duration=0.4, velocity_tolerance=0.001)
        self.assertFalse(target.update(True, [0.0002, -0.0002], 1.0))
        self.assertFalse(target.update(True, [0.0002, -0.0002], 1.3))
        self.assertTrue(target.update(True, [0.0002, -0.0002], 1.4))

    def test_resets_for_motion_missing_velocity_or_goal_error(self):
        target = StableTarget(duration=0.4, velocity_tolerance=0.001)
        self.assertFalse(target.update(True, [0.0, 0.0], 1.0))
        self.assertFalse(target.update(True, [0.002, 0.0], 1.3))
        self.assertFalse(target.update(True, [0.0, None], 1.5))
        self.assertFalse(target.update(False, [0.0, 0.0], 1.7))
        self.assertFalse(target.update(True, [0.0, 0.0], 2.0))
        self.assertTrue(target.update(True, [0.0, 0.0], 2.4))


class SafetyStopTest(unittest.TestCase):
    def event(self, code='SLIP_DETECTED', stamp=100.0):
        return json.dumps({'code': code, 'published_at_unix': stamp})

    def test_is_opt_in_and_accepts_only_fresh_supported_events(self):
        disabled = SafetyStop(False)
        self.assertEqual((False, 'disabled'), disabled.ingest(self.event(), 100.5))
        enabled = SafetyStop(True, max_age_seconds=2.0)
        self.assertEqual((False, 'stale_event'), enabled.ingest(self.event(stamp=90), 100))
        self.assertEqual(
            (False, 'unsupported_event'),
            enabled.ingest(self.event(code='UNKNOWN'), 100))
        self.assertEqual((True, 'latched'), enabled.ingest(self.event(), 100.5))
        self.assertEqual('SLIP_DETECTED', enabled.event['code'])

    def test_every_declared_safety_code_is_accepted(self):
        for code in SAFETY_STOP_CODES:
            with self.subTest(code=code):
                stop = SafetyStop(True)
                self.assertTrue(stop.ingest(self.event(code=code), 100.1)[0])

    def test_latch_is_one_shot_until_explicit_reset(self):
        stop = SafetyStop(True)
        self.assertTrue(stop.ingest(self.event(), 100.1)[0])
        self.assertEqual((False, 'already_latched'), stop.ingest(self.event(), 100.2))
        stop.reset()
        self.assertIsNone(stop.event)
        self.assertTrue(stop.ingest(self.event(stamp=101), 101.1)[0])

    def test_rejects_malformed_and_future_events(self):
        stop = SafetyStop(True)
        self.assertEqual((False, 'invalid_json'), stop.ingest('{', 100))
        self.assertEqual((False, 'unsupported_event'), stop.ingest('[]', 100))
        self.assertEqual((False, 'stale_event'), stop.ingest(self.event(stamp=102), 100))


if __name__ == '__main__':
    unittest.main()
