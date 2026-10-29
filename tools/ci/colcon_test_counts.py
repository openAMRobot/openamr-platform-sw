#!/usr/bin/env python3
# Copyright 2026 OpenAMRobot contributors
#
# Licensed under the MIT License (see LICENSE).

"""Per-package test counts from the JUnit files colcon test writes (PSW-01).

Reads build/<package>/**/*.xml for every package named on the command line, prints a
Markdown table (also appended to $GITHUB_STEP_SUMMARY when set) and exits non-zero when
the executed test count is zero, overall or for a package not listed with --allow-empty.
SKIP is not PASS: skipped tests are counted separately and never as executed.

Usage:

  python3 tools/ci/colcon_test_counts.py --build-base ros2/build \
      --allow-empty openamrobot_docking=reason pkg_a pkg_b ...
"""

import argparse
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def suites(path):
    """Return the <testsuite> elements of one JUnit file, or [] when it is not JUnit."""
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError:
        return []
    if root.tag == 'testsuite':
        return [root]
    if root.tag == 'testsuites':
        return list(root.iter('testsuite'))
    return []


def count(build_base, package):
    """Return the summed counts of the JUnit files under build_base/package."""
    totals = {'tests': 0, 'failures': 0, 'errors': 0, 'skipped': 0, 'files': 0}
    for xml in sorted((Path(build_base) / package).rglob('*.xml')):
        found = suites(xml)
        if not found:
            continue
        totals['files'] += 1
        for s in found:
            # nested suites carry their own counts; only leaf suites hold test cases
            if s.find('testsuite') is not None:
                continue
            totals['tests'] += int(s.get('tests', 0))
            totals['failures'] += int(s.get('failures', 0))
            totals['errors'] += int(s.get('errors', 0))
            totals['skipped'] += int(s.get('skipped', 0) or s.get('disabled', 0))
    totals['executed'] = totals['tests'] - totals['skipped']
    return totals


def main(argv=None):
    """Print the table, append it to the job summary and return the exit code."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--build-base', default='build')
    parser.add_argument('--allow-empty', action='append', default=[],
                        help='PACKAGE=REASON: a package expected to run no tests')
    parser.add_argument('packages', nargs='+')
    args = parser.parse_args(argv)
    allow = dict(a.split('=', 1) for a in args.allow_empty)

    rows, problems, total = [], [], 0
    for pkg in args.packages:
        c = count(args.build_base, pkg)
        total += c['executed']
        note = ''
        if c['executed'] == 0:
            if pkg in allow:
                note = 'no tests: ' + allow[pkg]
            else:
                problems.append('%s executed zero tests' % pkg)
                note = 'ZERO TESTS (failure)'
        rows.append('| %s | %d | %d | %d | %d | %d | %s |' % (
            pkg, c['executed'], c['failures'], c['errors'], c['skipped'], c['files'], note))
    if total == 0:
        problems.append('the workspace executed zero tests')

    lines = ['## colcon test: per-package counts', '',
             '| Package | Executed | Failures | Errors | Skipped (not passed) | JUnit files '
             '| Note |',
             '|---|---|---|---|---|---|---|'] + rows + ['', 'Executed in total: %d' % total]
    if problems:
        lines += ['', '**Failed:** ' + '; '.join(problems)]
    text = '\n'.join(lines) + '\n'
    print(text)
    summary = os.environ.get('GITHUB_STEP_SUMMARY')
    if summary:
        with open(summary, 'a', encoding='utf-8') as stream:
            stream.write(text)
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
