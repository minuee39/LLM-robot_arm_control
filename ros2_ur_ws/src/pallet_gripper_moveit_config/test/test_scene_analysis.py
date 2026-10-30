from copy import deepcopy
from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
from analyze_pallet_scene import analyze, pose_metrics


def fixture():
    rows = [{'frame': 'base_footprint', 'unit': 'm', 'unix_seconds': 100+i,
             'blocks': {name: {'position_m': [0.24, -0.28, 0.025],
                               'orientation_wxyz': [1, 0, 0, 0]}
                        for name in ('red_block', 'green_block', 'blue_block')}}
            for i in range(6)]
    report = {'passed': True, 'created_at': 105.2, 'runs': [
        {'object_id': 'red_block', 'passed': True, 'returncode': 0,
         'attempts': [{'attempt': 1, 'returncode': 0}], 'target': [0.24, -0.28, 0.025],
         'motion_summary': {'release_detected': True, 'release_seconds': 2,
                            'monitoring': {'started_at_unix': 100}}}]}
    return rows, report


class SceneAnalysisTest(unittest.TestCase):
    def test_passed_job_and_all_blocks_preserved(self):
        rows, report = fixture()
        result = analyze(rows, report)
        self.assertTrue(result['scene_passed'])
        self.assertTrue(result['first_attempt_passed'])
        self.assertTrue(result['regression_passed'])
        self.assertEqual(result['blocks']['red_block']['sample_count'], 4)

    def test_retry_success_not_counted_as_first_attempt_regression(self):
        rows, report = fixture()
        report['runs'][0]['attempts'] = [{'attempt': 1, 'returncode': 1},
                                       {'attempt': 2, 'returncode': 0}]
        result = analyze(rows, report)
        self.assertTrue(result['scene_passed'])
        self.assertFalse(result['first_attempt_passed'])
        self.assertFalse(result['regression_passed'])

    def test_missing_or_invalid_attempt_evidence_never_regression_passes(self):
        rows, report = fixture()
        for attempts in (None, [], [{'attempt': True, 'returncode': 0}],
                         [{'attempt': 1, 'returncode': False}], [{'attempt': 2, 'returncode': 0}]):
            invalid = deepcopy(report)
            invalid['runs'][0]['attempts'] = attempts
            self.assertFalse(analyze(rows, invalid)['regression_passed'])

    def test_completed_block_displaced_by_later_task(self):
        rows, report = fixture()
        rows[-1]['blocks']['red_block']['position_m'][0] += 0.03
        self.assertFalse(analyze(rows, report)['scene_passed'])

    def test_untouched_block_tipped(self):
        rows, report = fixture()
        rows[3]['blocks']['blue_block']['orientation_wxyz'] = [math.sqrt(0.5), math.sqrt(0.5), 0, 0]
        result = analyze(rows, report)
        self.assertTrue(result['job_passed'])
        self.assertFalse(result['blocks']['blue_block']['passed'])
        self.assertFalse(result['scene_passed'])

    def test_pre_release_motion_not_treated_as_placed_displacement(self):
        rows, report = fixture()
        rows[1]['blocks']['red_block']['position_m'][2] = 0.125
        self.assertTrue(analyze(rows, report)['scene_passed'])

    def test_missing_block_rejected(self):
        rows, report = fixture()
        del rows[2]['blocks']['green_block']
        with self.assertRaises(ValueError):
            analyze(rows, report)

    def test_stale_or_gapped_observation_rejected(self):
        rows, report = fixture()
        for invalid in (rows[:2], rows[:2]+rows[4:]):
            with self.subTest(rows=invalid), self.assertRaises(ValueError):
                analyze(invalid, report)

    def test_bad_release_rejected(self):
        rows, report = fixture()
        for key, value in [('release_detected', False), ('release_seconds', float('nan')),
                           ('release_seconds', -1), ('release_seconds', 10)]:
            modified = deepcopy(report)
            modified['runs'][0]['motion_summary'][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                analyze(rows, modified)

    def test_duplicate_or_unknown_run_rejected(self):
        rows, report = fixture()
        duplicate = deepcopy(report)
        duplicate['runs'].append(deepcopy(duplicate['runs'][0]))
        unknown = deepcopy(report)
        unknown['runs'][0]['object_id'] = 'other'
        for invalid in (duplicate, unknown):
            with self.assertRaises(ValueError):
                analyze(rows, invalid)

    def test_failed_job_never_scene_passes(self):
        rows, report = fixture()
        report['passed'] = False
        report['runs'][0]['passed'] = False
        self.assertFalse(analyze(rows, report)['scene_passed'])

    def test_invalid_pose_target_clock_rejected(self):
        rows, report = fixture()
        block = rows[0]['blocks']['red_block']
        for target in ([0, 0], [float('nan'), 0, 0], [True, 0, 0]):
            with self.assertRaises(ValueError):
                pose_metrics(block, target)
        for key, value in [('unix_seconds', float('nan')), ('unix_seconds', 100), ('frame', 'map')]:
            invalid = deepcopy(rows)
            invalid[2][key] = value
            with self.assertRaises(ValueError):
                analyze(invalid, report)
        invalid = deepcopy(block)
        invalid['orientation_wxyz'] = [0, 0, 0, 0]
        with self.assertRaises(ValueError):
            pose_metrics(invalid, [0, 0, 0])


if __name__ == '__main__':
    unittest.main()
