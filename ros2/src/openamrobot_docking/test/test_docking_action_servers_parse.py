# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software
# and associated documentation files (the "Software"), to deal in the Software without restriction,
# including without limitation the rights to use, copy, modify, merge, publish, distribute,
# sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or
# substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
# NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

"""Regressions for the action-count parser in docking_action_servers.sh.

The harness decides pass/fail from `ros2 action info … -t`. If a truncated, reworded or failed
query could be read as a number, then a negative control expecting "no servers" — or a mismatch —
would pass for the wrong reason. These cases pin the three shapes that must stay UNKNOWN.

No ROS required: the script's `--parse-count` mode prints the parser's verdict for given output.
"""
import os
import subprocess

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'docking_action_servers.sh')


def parse(output, exit_code=0):
    """Return the parser's verdict for one action-info result."""
    result = subprocess.run(
        ['bash', SCRIPT, '--parse-count', output, str(exit_code)],
        capture_output=True, text=True, check=True)
    return result.stdout.strip()


def test_non_numeric_count_is_not_a_count():
    # "Action servers: x" is what a truncated or reworded upstream output looks like.
    assert parse('Action servers: x') == 'UNKNOWN'
    assert parse('Action servers: ') == 'UNKNOWN'
    assert parse('Action servers: 1 (plus something)') == 'UNKNOWN'


def test_missing_count_is_not_a_count():
    assert parse('') == 'UNKNOWN'
    assert parse('No actions matched /dock_robot') == 'UNKNOWN'
    assert parse('Action servers: 1', exit_code=1) == 'UNKNOWN'


def test_well_formed_counts_still_parse():
    # Positive control for the two above: the rejection must not come from refusing everything.
    assert parse('Action servers: 1') == '1'
    assert parse('Action servers: 0') == '0'
