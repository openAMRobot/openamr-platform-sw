"""The udev rules parse, and every /dev/openamr name used by the repository exists in them."""
import glob
import os
import re
import shutil
import subprocess

import pytest

from conftest import REPO

RULES = os.path.join(REPO, 'install', 'udev', '99-openamrobot.rules')

# key, optional {attr} or sub-key, operator, quoted value
TOKEN = re.compile(
    r'^(?P<key>[A-Z_]+)(\{[^}]+\})?\s*(?P<op>==|!=|\+=|-=|:=|=)\s*"(?P<val>[^"\n]*)"$')
MATCH_KEYS = {'ACTION', 'DEVPATH', 'KERNEL', 'KERNELS', 'NAME', 'SYMLINK', 'SUBSYSTEM',
              'SUBSYSTEMS', 'DRIVER', 'DRIVERS', 'ATTR', 'ATTRS', 'TAG', 'TAGS', 'ENV',
              'PROGRAM', 'RESULT', 'TEST'}
ASSIGN_KEYS = {'NAME', 'SYMLINK', 'OWNER', 'GROUP', 'MODE', 'TAG', 'ENV', 'RUN', 'LABEL',
               'GOTO', 'OPTIONS', 'IMPORT', 'SECLABEL'}


def _active_lines():
    with open(RULES, encoding='utf-8') as f:
        for number, raw in enumerate(f, 1):
            line = raw.strip()
            if line and not line.startswith('#'):
                yield number, line


def _tokens(line):
    # split on commas that are outside double quotes
    parts, current, quoted = [], '', False
    for ch in line:
        if ch == '"':
            quoted = not quoted
        if ch == ',' and not quoted:
            parts.append(current.strip())
            current = ''
        else:
            current += ch
    assert not quoted, 'unterminated quote: ' + line
    parts.append(current.strip())
    return parts


def rule_symlinks():
    """Return the set of SYMLINK+= names in the active rules."""
    names = set()
    for _, line in _active_lines():
        for token in _tokens(line):
            m = TOKEN.match(token)
            if m and m.group('key') == 'SYMLINK' and m.group('op') in ('+=', '=', ':='):
                names.update(m.group('val').split())
    return names


def rule_net_names():
    """Return the NAME= network interface names in the active rules."""
    names = set()
    for _, line in _active_lines():
        for token in _tokens(line):
            m = TOKEN.match(token)
            if m and m.group('key') == 'NAME' and m.group('op') in ('=', ':='):
                names.add(m.group('val'))
    return names


def test_rules_file_exists_and_has_rules():
    assert os.path.isfile(RULES)
    assert len(list(_active_lines())) >= 8


def test_every_line_parses():
    for number, line in _active_lines():
        tokens = _tokens(line)
        matched = []
        for token in tokens:
            m = TOKEN.match(token)
            assert m, '%s:%d: cannot parse %r' % (RULES, number, token)
            matched.append(m)
        assert any(m.group('op') in ('==', '!=') for m in matched), \
            'line %d has no match key' % number
        assert any(m.group('op') not in ('==', '!=') for m in matched), \
            'line %d has no assignment' % number
        for m in matched:
            if m.group('op') in ('==', '!='):
                assert m.group('key') in MATCH_KEYS, 'line %d: %s' % (number, m.group('key'))
            else:
                assert m.group('key') in ASSIGN_KEYS, 'line %d: %s' % (number, m.group('key'))


def test_udevadm_verify_if_available():
    if shutil.which('udevadm') is None:
        pytest.skip('udevadm not installed on this runner; the parser test above still ran')
    result = subprocess.run(['udevadm', 'verify', RULES], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def test_vendor_ids_are_four_hex_digits_and_todo_ids_are_not_active():
    for number, line in _active_lines():
        assert 'TODO' not in line, 'line %d is active but still has a TODO value' % number
        for m in re.finditer(r'idVendor\}\s*==\s*"([^"]*)"', line):
            assert re.fullmatch(r'[0-9a-f]{4}', m.group(1)), 'line %d' % number
        for m in re.finditer(r'idProduct\}\s*==\s*"([^"]*)"', line):
            for alt in m.group(1).split('|'):
                assert re.fullmatch(r'[0-9a-f]{4}', alt), 'line %d' % number


def test_expected_device_names_present():
    names = rule_symlinks()
    for expected in ('openamr/lidar', 'openamr/gemini_336l', 'openamr/teensy',
                     'openamr/nucleo_stlink_vcp', 'openamr/nucleo_stlink'):
        assert expected in names


def test_todo_names_documented_for_the_missing_ids():
    # The names that cannot be active yet must be listed as TODO(owner) comments.
    with open(RULES, encoding='utf-8') as f:
        text = f.read()
    for name in ('lidar_s3', 'lidar_a1', 'zed_mini', 'nucleo_cdc', 'can_right', 'can_left'):
        assert name in text
    assert text.count('TODO(Karthikeyan)') >= 4
    assert 'TODO(Evans)' in text


def _scan_files():
    patterns = ['ros2/src/**/launch/*.py', 'ros2/src/**/config/*.yaml',
                'install/systemd/*', 'install/jetson/*', 'docker/**/*', 'docker-compose.yml']
    for pattern in patterns:
        for path in glob.glob(os.path.join(REPO, pattern), recursive=True):
            if os.path.isfile(path):
                yield path


def test_every_dev_openamr_name_used_exists_in_rules():
    names = rule_symlinks()
    used = {}
    for path in _scan_files():
        with open(path, encoding='utf-8', errors='replace') as f:
            for number, line in enumerate(f, 1):
                if line.lstrip().startswith('#'):
                    continue
                for m in re.finditer(r'/dev/(openamr/[A-Za-z0-9_]+)', line):
                    used.setdefault(m.group(1), []).append('%s:%d' % (path, number))
    for name, places in used.items():
        assert name in names, '%s is used in %s but no active rule creates it' % (name, places)


def test_can_interface_names_used_exist_in_rules_or_are_marked_todo():
    # can_right and can_left stay TODO until the adapter serials are known; nothing in the
    # install folder may enable them yet.
    assert not ({'can_right', 'can_left'} & rule_net_names())
