"""systemd units verify, are ordered, restart safely and never start motion."""
import configparser
import glob
import os
import shutil
import subprocess

import pytest

from conftest import REPO

UNIT_DIR = os.path.join(REPO, 'install', 'systemd')
SERVICES = ['openamr-agent.service', 'openamr-bringup.service', 'openamr-ui.service']
RUN_SCRIPT = os.path.join(UNIT_DIR, 'bin', 'openamr-run.sh')


def _unit(name):
    parser = configparser.ConfigParser(strict=False, interpolation=None)
    parser.optionxform = str
    parser.read(os.path.join(UNIT_DIR, name), encoding='utf-8')
    return parser


def _words(parser, section, key):
    return ' '.join(parser.get(section, key, fallback='').split()).split(' ')


def test_systemd_analyze_verify(tmp_path):
    if shutil.which('systemd-analyze') is None:
        pytest.skip('systemd-analyze is not installed on this runner')
    # The installed path /opt/openamr/bin does not exist on a runner. Point it at the
    # repository copy and drop User= (the openamr user does not exist either).
    for path in glob.glob(os.path.join(UNIT_DIR, 'openamr*.service')) + [
            os.path.join(UNIT_DIR, 'openamr.target')]:
        with open(path, encoding='utf-8') as f:
            text = f.read().replace('/opt/openamr/bin/openamr-run.sh', RUN_SCRIPT)
        text = ''.join(line for line in text.splitlines(True)
                       if not line.startswith(('User=', 'Group=', 'SupplementaryGroups=')))
        (tmp_path / os.path.basename(path)).write_text(text, encoding='utf-8')
    files = sorted(str(p) for p in tmp_path.iterdir())
    result = subprocess.run(['systemd-analyze', 'verify'] + files,
                            capture_output=True, text=True, cwd=str(tmp_path))
    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    problems = [line for line in output.splitlines() if 'openamr' in line]
    assert not problems, output


def test_ordering_clock_then_agent_then_bringup_then_ui():
    agent, bringup, ui = (_unit(n) for n in SERVICES)
    assert 'time-sync.target' in _words(agent, 'Unit', 'After')
    assert 'openamr-agent.service' in _words(bringup, 'Unit', 'After')
    assert 'time-sync.target' in _words(bringup, 'Unit', 'After')
    assert 'openamr-bringup.service' in _words(ui, 'Unit', 'After')


def test_bringup_requires_agent_and_ui_does_not_require_bringup():
    bringup, ui = _unit(SERVICES[1]), _unit(SERVICES[2])
    assert 'openamr-agent.service' in _words(bringup, 'Unit', 'Requires')
    assert 'openamr-bringup.service' not in _words(ui, 'Unit', 'Requires')


def test_restart_policy_is_bounded():
    for name in SERVICES:
        unit = _unit(name)
        assert unit.get('Service', 'Restart') == 'on-failure', name
        assert int(unit.get('Unit', 'StartLimitBurst')) <= 5, name
        assert int(unit.get('Unit', 'StartLimitIntervalSec')) >= 60, name
        assert not unit.get('Service', 'Restart') == 'always'


def test_environment_file_shared_and_complete():
    for name in SERVICES:
        assert _unit(name).get('Service', 'EnvironmentFile') == '/etc/openamr/openamr.env'
    env = {}
    with open(os.path.join(UNIT_DIR, 'openamr.env'), encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                key, _, value = line.partition('=')
                assert key.isidentifier() and ' ' not in key, line
                assert not value.startswith('export')
                env[key] = value
    for key in ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI', 'OPENAMR_MAP',
                'OPENAMR_AGENT_DEVICE'):
        assert key in env, key
    assert env['RMW_IMPLEMENTATION'] == 'rmw_cyclonedds_cpp'
    assert env['ROS_DOMAIN_ID'].isdigit()
    assert env['OPENAMR_MAP'] == '', 'no default map may be shipped'
    assert env['OPENAMR_ROSBRIDGE_ADDRESS'] != '0.0.0.0'


def test_cyclonedds_file_is_well_formed():
    import xml.etree.ElementTree as ET
    root = ET.parse(os.path.join(UNIT_DIR, 'cyclonedds.xml')).getroot()
    assert root.tag.endswith('CycloneDDS')


def test_no_unit_or_script_commands_motion():
    forbidden = ('cmd_vel', 'goal_pose', 'initialpose', 'resume', 'navigate_to_pose',
                 'ros2 topic pub', 'ros2 action send_goal', 'ros2 service call')
    paths = [os.path.join(UNIT_DIR, n) for n in SERVICES] + [RUN_SCRIPT]
    for path in paths:
        with open(path, encoding='utf-8') as f:
            code = [line for line in f if not line.lstrip().startswith('#')]
        text = ''.join(code).lower()
        for word in forbidden:
            assert word not in text, '%s contains %s' % (path, word)


def test_hardening_and_no_root():
    for name in SERVICES:
        unit = _unit(name)
        assert unit.get('Service', 'User') == 'openamr'
        assert unit.get('Service', 'NoNewPrivileges') == 'true'


def test_target_wants_all_services():
    target = _unit('openamr.target')
    assert set(SERVICES) <= set(_words(target, 'Unit', 'Wants'))


def test_run_script_syntax_and_shellcheck():
    assert subprocess.run(['bash', '-n', RUN_SCRIPT]).returncode == 0
    if shutil.which('shellcheck'):
        result = subprocess.run(['shellcheck', RUN_SCRIPT], capture_output=True, text=True)
        assert result.returncode == 0, result.stdout


def test_run_script_refuses_missing_map_and_open_bind(tmp_path):
    env = dict(os.environ, ROS_DISTRO='nonexistent', OPENAMR_MAP='')
    # sourcing a missing setup.bash must fail before the checks: use a fake ROS root instead
    fake = tmp_path / 'ros'
    fake.mkdir()
    script = (tmp_path / 'run.sh')
    text = open(RUN_SCRIPT, encoding='utf-8').read().replace(
        '/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash', str(fake / 'setup.bash'))
    script.write_text(text, encoding='utf-8')
    (fake / 'setup.bash').write_text('')
    r = subprocess.run(['bash', str(script), 'bringup'], env=env, capture_output=True, text=True)
    assert r.returncode == 3 and 'OPENAMR_MAP' in r.stderr
    env['OPENAMR_ROSBRIDGE_ADDRESS'] = '0.0.0.0'
    r = subprocess.run(['bash', str(script), 'ui'], env=env, capture_output=True, text=True)
    assert r.returncode == 3 and 'all interfaces' in r.stderr
    env['OPENAMR_AGENT_DEVICE'] = str(tmp_path / 'nodevice')
    r = subprocess.run(['bash', str(script), 'agent'], env=env, capture_output=True, text=True)
    assert r.returncode == 3
