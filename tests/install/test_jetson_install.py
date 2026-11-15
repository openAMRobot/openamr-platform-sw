"""The Jetson install skeleton: shellcheck clean, every step checked, chrony policy sane."""
import os
import re
import shutil
import subprocess

import pytest

from conftest import REPO

JETSON = os.path.join(REPO, 'install', 'jetson')
SCRIPT = os.path.join(JETSON, 'install-jetson.sh')


def _text(path):
    with open(path, encoding='utf-8') as f:
        return f.read()


def test_script_syntax_and_shellcheck():
    assert subprocess.run(['bash', '-n', SCRIPT]).returncode == 0
    if shutil.which('shellcheck'):
        r = subprocess.run(['shellcheck', SCRIPT], capture_output=True, text=True)
        assert r.returncode == 0, r.stdout


def test_every_step_has_do_and_check_functions():
    text = _text(SCRIPT)
    steps = re.search(r'^STEPS=\((.*?)\)', text, re.M).group(1).split()
    assert len(steps) >= 10
    for step in steps:
        assert re.search(r'^check_%s\(\)' % step, text, re.M), step
        assert re.search(r'^do_%s\(\)' % step, text, re.M), step


def test_list_matches_steps():
    r = subprocess.run([SCRIPT, 'list'], capture_output=True, text=True)
    assert r.returncode == 0
    assert 'identity' in r.stdout.split() and 'chrony' in r.stdout.split()


def test_identity_check_fails_on_wrong_release(tmp_path):
    # A Jetson Linux R36 (JetPack 6) image must not pass the identity check.
    tegra = tmp_path / 'nv_tegra_release'
    tegra.write_text('# R36 (release), REVISION: 4.0, GCID: 1, BOARD: generic\n')
    osr = tmp_path / 'os-release'
    osr.write_text('VERSION_ID="22.04"\n')
    env = dict(os.environ, NV_TEGRA_RELEASE_FILE=str(tegra), OS_RELEASE_FILE=str(osr),
               OPENAMR_INSTALL_LOG=str(tmp_path / 'log'))
    r = subprocess.run([SCRIPT, 'check', 'identity'], env=env, capture_output=True, text=True)
    assert r.returncode == 1
    assert '[FAIL]  identity' in r.stderr


def test_identity_check_passes_on_expected_release_except_arch(tmp_path):
    tegra = tmp_path / 'nv_tegra_release'
    tegra.write_text('# R39 (release), REVISION: 2.1, GCID: 1, BOARD: generic\n')
    osr = tmp_path / 'os-release'
    osr.write_text('VERSION_ID="24.04"\n')
    env = dict(os.environ, NV_TEGRA_RELEASE_FILE=str(tegra), OS_RELEASE_FILE=str(osr),
               OPENAMR_INSTALL_LOG=str(tmp_path / 'log'))
    r = subprocess.run([SCRIPT, 'check', 'identity'], env=env, capture_output=True, text=True)
    if os.uname().machine == 'aarch64':
        assert r.returncode == 0
    else:
        assert r.returncode == 1 and 'not aarch64' in r.stderr


def test_dry_run_changes_nothing_and_never_enables_services(tmp_path):
    env = dict(os.environ, OPENAMR_INSTALL_LOG=str(tmp_path / 'log'))
    r = subprocess.run([SCRIPT, '--dry-run', 'run', 'udev'], env=env,
                       capture_output=True, text=True)
    assert 'DRY-RUN: sudo install' in r.stderr
    text = _text(SCRIPT)
    assert 'systemctl enable --now chrony' in text
    assert not re.search(r'systemctl enable.*openamr', text)


def test_pins_are_named():
    text = _text(SCRIPT)
    for pin in ('EXPECT_L4T_RELEASE', 'ZED_SDK_VERSION', 'ZED_WRAPPER_TAG', 'ORBBEC_FW_EXPECTED'):
        assert pin in text
    assert '39.2.1' in _text(os.path.join(JETSON, 'README.md'))


def _directives(path):
    out = {}
    for line in _text(path).splitlines():
        line = line.strip()
        if line and not line.startswith('#'):
            key, _, rest = line.partition(' ')
            out.setdefault(key, []).append(rest.strip())
    return out


def test_chrony_policy():
    d = _directives(os.path.join(JETSON, 'chrony', 'openamr.conf'))
    assert d['makestep'] == ['1.0 3']          # step only at boot, then slew
    assert 'local' in d and 'orphan' in d['local'][0]
    assert 'allow' in d and 'rtcsync' in d
    assert any(x.startswith('ntp.') or 'pool' in x for x in d['pool'])
    client = _directives(os.path.join(JETSON, 'chrony', 'openamr-tablet.conf'))
    assert len(client['server']) == 1          # the tablet has one source: the Jetson


def test_readme_covers_rosbridge_note():
    text = _text(os.path.join(JETSON, 'README.md'))
    for word in ('topics_glob', 'services_glob', '0.0.0.0', 'ssl', '/cmd_vel'):
        assert word in text
