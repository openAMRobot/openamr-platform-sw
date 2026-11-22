# SPDX-License-Identifier: MIT
# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
"""Keep tools/bench read-only: no frame sending, no publishing, no motion, no link change.

A text check: it proves the scripts do not contain these commands, not that the robot is safe.
Run: python3 -m pytest -q tools/bench/test
"""
import pathlib
import re
import subprocess

import pytest

BENCH = pathlib.Path(__file__).resolve().parents[1]
SCRIPTS = sorted(p for p in BENCH.iterdir() if p.suffix in ('.sh', '.py') and p.is_file())
EXPECTED = {'bench_common.sh', 'can_readonly.sh', 'lidar_identify.sh', 'ros_graph_snapshot.sh'}

# Each pattern is a way to send a CAN frame, publish (any /cmd_vel publish goes through one
# of these), request an action, change a parameter or a link, or start the base bridge.
FORBIDDEN = {
    'cansend': r'\bcansend\b',
    'cangen': r'\bcangen\b',
    'canplayer': r'\bcanplayer\b',
    'ros2 topic pub': r'\bros2\s+topic\s+pub\b',
    'ros2 service call': r'\bros2\s+service\s+call\b',
    'ros2 action send_goal': r'\bros2\s+action\s+send_goal\b',
    'ros2 param set': r'\bros2\s+param\s+(?:set|load)\b',
    'ros2 launch': r'\bros2\s+launch\b',
    'micro-ROS agent': r'\bmicro_ros_agent\b',
    'ip link set': r'\bip\s+link\s+set\b',
    'publisher in code': r'\bcreate_publisher\b|\.publish\s*\(',
    'Twist message': r'\bTwist(?:Stamped)?\b',
    'cmd_vel publish': r'cmd_vel[^\n]*\bpub\b|\bpub\b[^\n]*cmd_vel',
}


def findings(text):
    """Return (line number, rule) for every forbidden command in text."""
    out = []
    for lineno, line in enumerate(text.splitlines(), 1):
        for rule, pattern in FORBIDDEN.items():
            if re.search(pattern, line):
                out.append((lineno, rule))
    return out


def test_expected_scripts_present():
    assert EXPECTED <= {p.name for p in SCRIPTS}


@pytest.mark.parametrize('script', SCRIPTS, ids=lambda p: p.name)
def test_no_sending_or_motion_command(script):
    assert findings(script.read_text()) == [], f'{script.name} contains a forbidden command'


@pytest.mark.parametrize('planted', [
    'cansend can0 601#2B40600000000000',
    'ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"',
    'ros2 topic pub --once /cmd_vel_teleop std_msgs/msg/Empty',
    'pub = node.create_publisher(Twist, "/cmd_vel", 10)',
    'sudo ip link set can0 up type can bitrate 500000',
    'ros2 launch openamrobot_drivers drivers.launch.py',
])
def test_planted_line_is_found(tmp_path, planted):
    """The scan itself works: every real script with one planted line fails it."""
    for script in SCRIPTS:
        copy = tmp_path / script.name
        copy.write_text(script.read_text() + planted + '\n')
        assert findings(copy.read_text()), f'{planted!r} not found in {script.name}'


@pytest.mark.parametrize('script', SCRIPTS, ids=lambda p: p.name)
def test_header_and_strict_mode(script):
    text = script.read_text()
    assert 'SPDX-License-Identifier: MIT' in text.splitlines()[1]
    if script.name != 'bench_common.sh':
        assert 'set -euo pipefail' in text


@pytest.mark.parametrize('script', SCRIPTS, ids=lambda p: p.name)
def test_listeners_are_bounded(script):
    """Commands that run until stopped are only started through step (timeout)."""
    unbounded = re.compile(r'\b(?:ros2\s+topic\s+(?:hz|echo)|candump|tf2_echo|sllidar_ros2\s+sllidar_node|view_frames)\b')
    for lineno, line in enumerate(script.read_text().splitlines(), 1):
        if line.lstrip().startswith('#'):
            continue
        code = re.sub(r'"[^"]*"|\bhave\s+\S+', '', line)
        if unbounded.search(code) and not re.search(r'^\s*(?:step|timeout)\b', line.lstrip('|& ')):
            raise AssertionError(f'{script.name}:{lineno} unbounded: {line.strip()}')


@pytest.mark.parametrize('script', SCRIPTS, ids=lambda p: p.name)
def test_bash_syntax(script):
    if script.suffix == '.sh':
        subprocess.run(['bash', '-n', str(script)], check=True)


@pytest.mark.parametrize('args', [
    ['--iface', 'can0#123'],
    ['--iface', 'can0', '601#2B40600000000000'],
    ['--iface', 'cansend'],
    ['--iface', 'can0', '--bitrate', '500000'],
    ['--iface', 'can0', 'up'],
    ['--frame', 'x'],
    ['--iface', 'can0;reboot'],
    ['--seconds', '999'],
])
def test_can_readonly_refuses_sending_arguments(tmp_path, args):
    proc = subprocess.run(['bash', str(BENCH / 'can_readonly.sh'), *args, '--out', str(tmp_path)],
                          capture_output=True, text=True, timeout=30)
    assert proc.returncode == 2, proc.stdout + proc.stderr
    assert 'REFUSED' in proc.stderr
    assert not any(tmp_path.iterdir()), 'refused run must not start a capture'

