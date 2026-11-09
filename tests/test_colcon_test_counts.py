# Copyright 2026 OpenAMRobot contributors
# Licensed under the MIT License (see LICENSE).
"""Incomplete result XML must never become successful test-count evidence."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'tools/ci/colcon_test_counts.py'
spec = importlib.util.spec_from_file_location('colcon_test_counts', SCRIPT)
counts = importlib.util.module_from_spec(spec)
spec.loader.exec_module(counts)
GOOD = '<testsuite tests="2" failures="0" errors="0" skipped="1"/>'


class ResultIntegrity(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name)
        self.pkg = self.base / 'pkg'
        self.pkg.mkdir()
        (self.pkg / 'valid.xml').write_text(GOOD)

    def test_valid_counts_preserve_skipped_boundary(self):
        self.assertEqual(counts.count(self.base, 'pkg'),
                         {'tests': 2, 'failures': 0, 'errors': 0,
                          'skipped': 1, 'files': 1, 'executed': 1})

    def test_valid_non_junit_is_not_counted(self):
        (self.pkg / 'metadata.xml').write_text('<package><name>pkg</name></package>')
        self.assertEqual(counts.count(self.base, 'pkg')['files'], 1)

    def test_mixed_valid_and_malformed_results_raise(self):
        (self.pkg / 'broken.xml').write_text('<testsuite tests="8"')
        with self.assertRaisesRegex(ValueError, 'broken.xml'):
            counts.count(self.base, 'pkg')

    def test_cli_rejects_mixed_results(self):
        (self.pkg / 'broken.xml').write_text('<testsuite')
        result = subprocess.run([sys.executable, str(SCRIPT), '--build-base',
                                 str(self.base), 'pkg'], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('broken.xml', result.stdout + result.stderr)

    def test_allow_empty_does_not_allow_malformed_results(self):
        (self.pkg / 'valid.xml').unlink()
        (self.pkg / 'broken.xml').write_text('<testsuite')
        other = self.base / 'other'
        other.mkdir()
        (other / 'good.xml').write_text(GOOD)
        result = subprocess.run([sys.executable, str(SCRIPT), '--build-base',
                                 str(self.base), '--allow-empty', 'pkg=tracked no tests',
                                 'pkg', 'other'], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('broken.xml', result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
