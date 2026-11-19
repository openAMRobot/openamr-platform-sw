"""The vcan guard: name, virtual device and CAN link type. And the MTU the script sets.

The tests build a fake sysfs tree, because no test may depend on a hardware CAN adapter.
"""
import os
import subprocess

import pytest

from conftest import PACKAGE_DIR
from openamrobot_sil import canbus

SCRIPT = os.path.join(PACKAGE_DIR, 'scripts', 'sil_vcan.sh')
PCI = 'devices/pci0000:00/0000:00:14.0/usb1/1-2/1-2:1.0/net'   # a USB CAN adapter
VIRTUAL = 'devices/virtual/net'


def add_iface(root, name, parent, link_type=280, mtu=72):
    device = root / parent / name
    device.mkdir(parents=True)
    (device / 'type').write_text('%d\n' % link_type)
    (device / 'mtu').write_text('%d\n' % mtu)
    (root / 'class' / 'net').mkdir(parents=True, exist_ok=True)
    os.symlink(os.path.relpath(device, root / 'class' / 'net'), root / 'class' / 'net' / name)


@pytest.fixture
def sysfs(tmp_path, monkeypatch):
    (tmp_path / VIRTUAL).mkdir(parents=True)
    monkeypatch.setattr(canbus, 'SYSFS_ROOT', str(tmp_path))
    return tmp_path


SYSFS_LINE = 'SYSFS=/sys\n'


def run_script(sysfs, *args, path=None):
    """Run a copy of the script whose sysfs root line points at the fake tree.

    The script itself takes its sysfs root from no environment variable, so the copy is the
    only way a test moves it.
    """
    with open(SCRIPT, encoding='ascii') as f:
        text = f.read()
    assert text.count(SYSFS_LINE) == 1, 'sil_vcan.sh must set SYSFS=/sys on one line'
    copy = sysfs.parent / ('sil_vcan_%s.sh' % sysfs.name)
    copy.write_text(text.replace(SYSFS_LINE, 'SYSFS=%s\n' % sysfs))
    env = dict(os.environ)
    if path is not None:
        env['PATH'] = path
    return subprocess.run(['sh', str(copy)] + list(args), capture_output=True, text=True, env=env)


def test_virtual_can_interface_is_accepted(sysfs):
    add_iface(sysfs, 'vcan_drive', VIRTUAL)
    assert canbus.virtual_can_reason('vcan_drive') is None


def test_non_virtual_interface_named_vcan_drive_is_refused(sysfs):
    add_iface(sysfs, 'vcan_drive', PCI)   # a real adapter renamed to vcan_drive
    assert 'not under /sys/devices/virtual/net/' in canbus.virtual_can_reason('vcan_drive')
    with pytest.raises(canbus.NotVirtualCan, match='hardware interface'):
        canbus.open_bus('vcan_drive')


def test_missing_interface_is_refused(sysfs):
    with pytest.raises(canbus.NotVirtualCan, match='does not exist'):
        canbus.open_bus('vcan_drive')


def test_virtual_interface_that_is_not_can_is_refused(sysfs):
    add_iface(sysfs, 'vcan_dummy', VIRTUAL, link_type=1)   # dummy or veth: Ethernet type
    with pytest.raises(canbus.NotVirtualCan, match='not CAN'):
        canbus.open_bus('vcan_dummy')


def test_symlink_that_leaves_the_virtual_directory_is_refused(sysfs):
    add_iface(sysfs, 'can0', PCI)
    os.symlink(sysfs / PCI / 'can0', sysfs / VIRTUAL / 'vcan_alias')
    os.symlink(sysfs / VIRTUAL / 'vcan_alias', sysfs / 'class' / 'net' / 'vcan_alias')
    with pytest.raises(canbus.NotVirtualCan, match='hardware interface'):
        canbus.open_bus('vcan_alias')


@pytest.mark.parametrize('name', ['can0', 'eth0', '', 'vcan/../can0'])
def test_names_that_are_not_vcan_are_refused(sysfs, name):
    assert canbus.virtual_can_reason(name) == 'the name does not start with "vcan"'


def test_sysfs_root_is_not_taken_from_the_environment(sysfs, monkeypatch):
    add_iface(sysfs, 'vcan_drive', PCI)
    other = sysfs / 'other'
    (other / VIRTUAL).mkdir(parents=True)
    add_iface(other, 'vcan_drive', VIRTUAL)
    for name in ('SIL_VCAN_SYSFS_ROOT', 'SYSFS_ROOT', 'SYSFS_PATH'):
        monkeypatch.setenv(name, str(other))
    assert canbus.virtual_can_reason('vcan_drive') is not None


def test_script_sysfs_root_is_not_taken_from_the_environment(sysfs):
    """The unpatched script ignores SIL_VCAN_SYSFS_ROOT: a fake tree cannot make it accept an
    interface that the real /sys does not show as virtual CAN (review of #149)."""
    if os.path.exists('/sys/class/net/vcan_envtest'):
        pytest.skip('vcan_envtest exists on this host; test needs it absent (#149)')
    add_iface(sysfs, 'vcan_envtest', VIRTUAL, mtu=16)
    env = dict(os.environ, SIL_VCAN_SYSFS_ROOT=str(sysfs), SYSFS_ROOT=str(sysfs))
    r = subprocess.run(['sh', SCRIPT, 'check', 'vcan_envtest'],
                       capture_output=True, text=True, env=env)
    assert r.returncode != 0, r.stdout
    assert 'vcan_envtest present' not in r.stdout
    assert run_script(sysfs, 'check', 'vcan_envtest').returncode == 0   # the fake tree is valid


@pytest.mark.parametrize('action', ['up', 'check', 'down'])
def test_script_refuses_a_non_virtual_interface_named_vcan_drive(sysfs, action):
    add_iface(sysfs, 'vcan_bms', VIRTUAL, mtu=16)
    add_iface(sysfs, 'vcan_drive', PCI)
    r = run_script(sysfs, action, 'vcan_bms', 'vcan_drive')
    assert r.returncode == 2
    assert 'refused: vcan_drive is not under /sys/devices/virtual/net/' in r.stderr
    assert r.stdout == ''   # refused before anything is done to vcan_bms


def test_script_refuses_a_virtual_interface_that_is_not_can(sysfs):
    add_iface(sysfs, 'vcan_dummy', VIRTUAL, link_type=1)
    r = run_script(sysfs, 'check', 'vcan_dummy')
    assert r.returncode == 2 and 'not a CAN interface' in r.stderr


@pytest.mark.parametrize('arg', ['can0', 'vcan_x:classic', 'vcan_x:fd:fd', 'vcan/x'])
def test_script_refuses_bad_arguments(sysfs, arg):
    assert run_script(sysfs, 'up', arg).returncode == 2


def test_script_check_compares_the_mtu(sysfs):
    add_iface(sysfs, 'vcan_drive', VIRTUAL, mtu=16)
    add_iface(sysfs, 'vcan_arm', VIRTUAL, mtu=72)
    ok = run_script(sysfs, 'check', 'vcan_drive', 'vcan_arm:fd')
    assert ok.returncode == 0, ok.stderr
    assert 'vcan_drive present, virtual CAN, mtu 16' in ok.stdout
    assert 'vcan_arm present, virtual CAN, mtu 72' in ok.stdout
    wrong = run_script(sysfs, 'check', 'vcan_arm')   # a classic bus must not be at 72
    assert wrong.returncode == 1 and 'MTU is 72, expected 16' in wrong.stderr


def test_script_sets_mtu_16_for_classic_and_72_for_can_fd(sysfs, tmp_path):
    """With a recording stand-in for ip and sudo: the commands the script issues."""
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    log = tmp_path / 'ip.log'
    (bin_dir / 'ip').write_text('#!/bin/sh\necho "$@" >> "%s"\n' % log)
    (bin_dir / 'sudo').write_text('#!/bin/sh\n[ "$1" = "-n" ] && shift\nexec "$@"\n')
    (bin_dir / 'modprobe').write_text('#!/bin/sh\nexit 0\n')
    for tool in ('ip', 'sudo', 'modprobe'):
        os.chmod(bin_dir / tool, 0o755)
    add_iface(sysfs, 'vcan_drive', VIRTUAL, mtu=72)    # kernel default of a new vcan
    add_iface(sysfs, 'vcan_arm', VIRTUAL, mtu=16)
    add_iface(sysfs, 'vcan_aux', VIRTUAL, mtu=16)      # already right: no down, no mtu
    r = run_script(sysfs, 'up', 'vcan_drive', 'vcan_arm:fd', 'vcan_aux',
                   path='%s:%s' % (bin_dir, os.environ['PATH']))
    assert r.returncode == 0, r.stderr
    assert log.read_text().splitlines() == [
        'link set dev vcan_drive down', 'link set dev vcan_drive mtu 16',
        'link set dev vcan_drive up',
        'link set dev vcan_arm down', 'link set dev vcan_arm mtu 72',
        'link set dev vcan_arm up',
        'link set dev vcan_aux up',
    ]
    assert 'vcan_drive up, mtu 16' in r.stdout and 'vcan_arm up, mtu 72' in r.stdout
