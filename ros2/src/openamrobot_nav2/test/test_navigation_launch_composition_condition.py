#!/usr/bin/env python3
"""Regression tests for lowercase use_composition launch conditions (#48).

Verifies that UnlessCondition(use_composition) and IfCondition(use_composition)
evaluate correctly for all boolean string representations: 'true', 'false',
'True', 'False'.  Uses a real LaunchContext to test the actual ROS 2 launch
condition evaluation, not just the source text.

These tests cover both navigation_launch.py and localization_launch.py.
"""
import os
import unittest

from launch import LaunchContext
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import TextSubstitution


# Paths to the launch files under test.
_LAUNCH_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'launch',
)
_NAV_LAUNCH = os.path.join(_LAUNCH_DIR, 'navigation_launch.py')
_LOC_LAUNCH = os.path.join(_LAUNCH_DIR, 'localization_launch.py')


class TestUnlessConditionBehavior(unittest.TestCase):
    """UnlessCondition must evaluate correctly for all boolean spellings.

    When use_composition is truthy ('true', 'True'), the non-composed group
    must NOT run (UnlessCondition returns False).

    When use_composition is falsy ('false', 'False'), the non-composed group
    MUST run (UnlessCondition returns True).
    """

    def _eval_unless(self, value: str) -> bool:
        """Evaluate UnlessCondition with the given string value."""
        lc = LaunchContext()
        condition = UnlessCondition(TextSubstitution(text=value))
        return condition.evaluate(lc)

    def _eval_if(self, value: str) -> bool:
        """Evaluate IfCondition with the given string value."""
        lc = LaunchContext()
        condition = IfCondition(TextSubstitution(text=value))
        return condition.evaluate(lc)

    # --- UnlessCondition tests (non-composed group) ---

    def test_unless_lowercase_true(self):
        """use_composition:=true → non-composed group should NOT run."""
        self.assertFalse(self._eval_unless('true'))

    def test_unless_titlecase_true(self):
        """use_composition:=True → non-composed group should NOT run."""
        self.assertFalse(self._eval_unless('True'))

    def test_unless_lowercase_false(self):
        """use_composition:=false → non-composed group SHOULD run."""
        self.assertTrue(self._eval_unless('false'))

    def test_unless_titlecase_false(self):
        """use_composition:=False → non-composed group SHOULD run."""
        self.assertTrue(self._eval_unless('False'))

    # --- IfCondition tests (composed group) ---

    def test_if_lowercase_true(self):
        """use_composition:=true → composed group SHOULD run."""
        self.assertTrue(self._eval_if('true'))

    def test_if_titlecase_true(self):
        """use_composition:=True → composed group SHOULD run."""
        self.assertTrue(self._eval_if('True'))

    def test_if_lowercase_false(self):
        """use_composition:=false → composed group should NOT run."""
        self.assertFalse(self._eval_if('false'))

    def test_if_titlecase_false(self):
        """use_composition:=False → composed group should NOT run."""
        self.assertFalse(self._eval_if('False'))

    # --- Mutual exclusion: exactly one group runs at a time ---

    def test_mutual_exclusion_lowercase_true(self):
        """With 'true', composed runs, non-composed does not."""
        self.assertTrue(self._eval_if('true'))
        self.assertFalse(self._eval_unless('true'))

    def test_mutual_exclusion_lowercase_false(self):
        """With 'false', non-composed runs, composed does not."""
        self.assertFalse(self._eval_if('false'))
        self.assertTrue(self._eval_unless('false'))

    def test_mutual_exclusion_titlecase_true(self):
        """With 'True', composed runs, non-composed does not."""
        self.assertTrue(self._eval_if('True'))
        self.assertFalse(self._eval_unless('True'))

    def test_mutual_exclusion_titlecase_false(self):
        """With 'False', non-composed runs, composed does not."""
        self.assertFalse(self._eval_if('False'))
        self.assertTrue(self._eval_unless('False'))


class TestLaunchFilesSyntax(unittest.TestCase):
    """Verify both launch files compile without syntax errors."""

    def test_navigation_launch_compiles(self):
        """navigation_launch.py must be valid Python."""
        with open(_NAV_LAUNCH) as f:
            compile(f.read(), _NAV_LAUNCH, 'exec')

    def test_localization_launch_compiles(self):
        """localization_launch.py must be valid Python."""
        with open(_LOC_LAUNCH) as f:
            compile(f.read(), _LOC_LAUNCH, 'exec')


class TestLocalizationLaunchConditionPattern(unittest.TestCase):
    """Verify launch files use UnlessCondition, not PythonExpression."""

    def test_no_python_expression_negation(self):
        """localization_launch.py must not use PythonExpression."""
        with open(_LOC_LAUNCH) as f:
            source = f.read()
        self.assertNotIn(
            "PythonExpression(['not '",
            source,
            "localization_launch.py still uses PythonExpression negation",
        )
        self.assertIn('UnlessCondition', source)

    def test_navigation_no_python_expression_negation(self):
        """navigation_launch.py must not use PythonExpression for negation."""
        with open(_NAV_LAUNCH) as f:
            source = f.read()
        self.assertNotIn(
            "PythonExpression(['not '",
            source,
            "navigation_launch.py still uses PythonExpression negation",
        )
        self.assertIn('UnlessCondition', source)


if __name__ == '__main__':
    unittest.main()
