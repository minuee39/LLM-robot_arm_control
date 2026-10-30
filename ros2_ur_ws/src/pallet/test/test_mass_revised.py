from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from validate_mass_revised import validate


class MassRevisedTest(unittest.TestCase):
    def test_description_integrity_and_known_dynamics_blockers(self):
        self.assertEqual(validate(ROOT), ['no.2', 'no.3'])
