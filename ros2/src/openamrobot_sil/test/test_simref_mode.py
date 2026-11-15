"""stm32:=simref without vcan or Gazebo: the drive emulator's CiA 402 statusword and failure
injections, the bridge's socket client, the Nav2 parameters of the mode and the launch file.
Protocol and configuration evidence only."""
import importlib.util
import json
import os
import socket
import tempfile
import time

from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node
import pytest
import yaml

from conftest import PACKAGE_DIR, REPO_ROOT, wait_for
from openamrobot_sil.can_sil import canopen_common
from openamrobot_sil.simref_link import NOMINAL_PINS, SimrefLink
from openamrobot_sil.zlac_gazebo import GazeboZlac8015d

SdoAbort, SdoClient = canopen_common.SdoAbort, canopen_common.SdoClient
SIMREF_CONFIG = os.path.join(PACKAGE_DIR, 'config', 'sil_simref.yaml')
HOST_CONFIG = os.path.join(PACKAGE_DIR, 'config', 'sil.yaml')


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def sil_launch():
    return _load('sil_launch_simref', os.path.join(PACKAGE_DIR, 'launch', 'sil.launch.py'))


@pytest.fixture(scope='module')
def nav2_launch():
    return _load('simref_nav2', os.path.join(PACKAGE_DIR, 'launch', 'simref_nav2.launch.py'))


@pytest.fixture
def cia_drive(make_bus):
    emu = GazeboZlac8015d(make_bus(), 1, cia402_statusword=True, commissioned_link_loss_ms=250).start()
    sdo = SdoClient(make_bus(), 1)
    yield emu, sdo
    emu.stop()


def sw(sdo):
    return sdo.read(0x6041, 0)


# ---- drive emulator ---------------------------------------------------------------------
def test_cia402_statusword_follows_the_enable_sequence(cia_drive):
    emu, sdo = cia_drive
    assert wait_for(lambda: sw(sdo) & 0x4F == 0x40)        # switch-on disabled
    for word, expect in ((0x06, 0x21), (0x07, 0x23), (0x0F, 0x27)):
        sdo.write(0x6040, 0, word, 2)
        assert wait_for(lambda e=expect: sw(sdo) & 0x6F == e), hex(sw(sdo))
    assert emu.enabled
    sdo.write(0x6040, 0, 0x06, 2)
    assert wait_for(lambda: sw(sdo) & 0x6F == 0x21)
    assert not emu.enabled


def test_commissioned_drive_reads_2000h_and_keeps_it_over_a_power_cycle(cia_drive):
    emu, sdo = cia_drive
    assert sdo.read(0x2000, 0) == 250
    emu.power_cycle()
    assert sdo.read(0x2000, 0) == 250
    assert wait_for(lambda: sw(sdo) & 0x4F == 0x40)


def test_transition_delay_holds_the_statusword_and_the_enable(cia_drive):
    emu, sdo = cia_drive
    sdo.write(0x6040, 0, 0x06, 2)
    sdo.write(0x6040, 0, 0x07, 2)
    assert wait_for(lambda: sw(sdo) & 0x6F == 0x23)
    emu.inject_transition_delay(0.3)
    t0 = time.monotonic()
    sdo.write(0x6040, 0, 0x0F, 2)
    time.sleep(0.15)
    assert sw(sdo) & 0x6F == 0x23 and not emu.enabled
    assert wait_for(lambda: sw(sdo) & 0x6F == 0x27, 1.0)
    assert time.monotonic() - t0 >= 0.29
    assert emu.enabled


def test_abort_injection_refuses_the_enable_word(cia_drive):
    emu, sdo = cia_drive
    emu.inject_abort_controlword(0x0F)
    sdo.write(0x6040, 0, 0x06, 2)
    sdo.write(0x6040, 0, 0x07, 2)
    with pytest.raises(SdoAbort):
        sdo.write(0x6040, 0, 0x0F, 2)
    assert not emu.enabled
    emu.inject_abort_controlword(None)
    sdo.write(0x6040, 0, 0x0F, 2)
    assert wait_for(lambda: emu.enabled)


def test_host_mode_drive_keeps_the_vendored_statusword(make_bus):
    emu = GazeboZlac8015d(make_bus(), 1).start()
    try:
        sdo = SdoClient(make_bus(), 1)
        assert sdo.read(0x2000, 0) == 0          # factory value: not commissioned
        sdo.write(0x6040, 0, 0x06, 2)
        time.sleep(0.05)
        assert sw(sdo) & 0x6F == 0x20            # vendored: no ready bit before enable
    finally:
        emu.stop()


# ---- bridge socket client ---------------------------------------------------------------
@pytest.fixture
def fake_reference():
    d = tempfile.mkdtemp()
    path = os.path.join(d, 'ref.sock')
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    srv.bind(path)
    srv.settimeout(1.0)
    yield srv, path
    srv.close()


def test_link_messages(fake_reference):
    srv, path = fake_reference
    link = SimrefLink(path, epoch=42)
    try:
        link.keepalive()
        got = [srv.recvfrom(256)[0].decode() for _ in range(2)]
        assert got == ['subscribe', 'session 42']
        link.send_pins()
        assert srv.recv(256).decode() == 'pins %d %d %d %d %d' % tuple(
            NOMINAL_PINS[k] for k in ('y32', 'estop_contact', 'reset', 'charger_fault', 'contactor_fb'))
        link.cmd(0.2, -0.1)
        assert srv.recv(256).decode() == 'cmd 42 1 0.200000 -0.100000'
        link.supervisory('jetson_stop_release')
        assert srv.recv(256).decode() == 'sup jetson_stop_release 42 2'
        link.set_pin('y32', 0)
        assert srv.recv(256).decode() == 'pin y32 0'
        link.arms('STOWED')
        assert srv.recv(256).decode() == 'arms STOWED 0 0 STOWED 0 0 STOWED'
        with pytest.raises(ValueError):
            link.set_pin('motor_enable', 1)
        with pytest.raises(ValueError):
            link.supervisory('enable_drive')
        client = link.client_path
        srv.sendto(json.dumps({'state': 'LOCKED'}).encode(), client)
        assert wait_for(lambda: link.poll() is not None or link.status is not None, 1.0)
        assert link.status['state'] == 'LOCKED'
    finally:
        link.close()


def test_link_survives_a_missing_reference(tmp_path):
    link = SimrefLink(str(tmp_path / 'absent.sock'))
    try:
        assert link.cmd(0.1, 0.0) is False   # no exception: the periodic resend recovers
    finally:
        link.close()


# ---- Nav2 parameters ----------------------------------------------------------------------
def test_simref_nav2_keeps_the_velocity_chain(nav2_launch):
    src_path = os.path.join(REPO_ROOT, 'ros2', 'src', 'openamrobot_nav2', 'config', 'nav2_params.yaml')
    with open(src_path, encoding='utf-8') as f:
        src = yaml.safe_load(f)
    p = nav2_launch.simref_nav2_params(src)
    cm, scm = p['collision_monitor']['ros__parameters'], src['collision_monitor']['ros__parameters']
    assert cm['cmd_vel_in_topic'] == scm['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert cm['cmd_vel_out_topic'] == scm['cmd_vel_out_topic'] == 'cmd_vel'
    assert cm['polygons'] == scm['polygons']
    assert p['velocity_smoother'] == src['velocity_smoother'] | {
        'ros__parameters': {**src['velocity_smoother']['ros__parameters'], 'use_sim_time': True}}
    assert p['bt_navigator']['ros__parameters']['global_frame'] == 'odom'
    assert p['global_costmap']['global_costmap']['ros__parameters']['global_frame'] == 'odom'
    assert cm['source_timeout'] == 0.0
    assert all(cm[s]['topic'] == nav2_launch.NO_SCAN_TOPIC for s in cm['observation_sources'])
    assert src['bt_navigator']['ros__parameters']['global_frame'] == 'map'  # source unchanged


def test_simref_nav2_remaps_match_navigation_launch(nav2_launch):
    text = open(os.path.join(REPO_ROOT, 'ros2', 'src', 'openamrobot_nav2', 'launch',
                             'navigation_launch.py'), encoding='utf-8').read()
    assert text.count("('cmd_vel', 'cmd_vel_nav')") >= 3   # controller, behaviors, smoother
    mine = open(os.path.join(PACKAGE_DIR, 'launch', 'simref_nav2.launch.py'), encoding='utf-8').read()
    assert mine.count("('cmd_vel', 'cmd_vel_nav')") == 3


# ---- launch file ------------------------------------------------------------------------
def test_simref_is_a_launch_choice(sil_launch):
    assert sil_launch.STM32_CHOICES == ('host', 'renode', 'simref')


def test_simref_sha_is_pinned(sil_launch):
    sha = sil_launch.SIMREF_FW_SHA
    assert len(sha) == 40 and all(c in '0123456789abcdef' for c in sha), sha


def _blocked(actions):
    return any(isinstance(a, LogInfo) and 'BLOCKED' in ''.join(getattr(t, 'text', str(t)) for t in a.msg)
               for a in actions)


def _fake_binary(tmp_path, sha):
    b = tmp_path / 'simref_process'
    b.write_text('#!/bin/sh\nexit 0\n')
    b.chmod(0o755)
    (tmp_path / 'simref_process.sha').write_text(sha + '\n')
    return str(b)


def test_simref_mode_starts_the_reference_and_the_bridge_not_the_host_client(sil_launch, tmp_path):
    actions = sil_launch.can_nodes(SIMREF_CONFIG, 'simref', True, _fake_binary(tmp_path, sil_launch.SIMREF_FW_SHA))
    nodes = sorted(a.node_executable for a in actions if isinstance(a, Node))
    assert nodes == ['ctl017_emulator', 'daly_bms_emulator', 'simref_bridge', 'zlac8015d_emulator']
    procs = [a for a in actions if isinstance(a, ExecuteProcess) and not isinstance(a, Node)]
    assert len(procs) == 1
    cmd = [str(getattr(c, 'text', c)) for part in procs[0].cmd for c in (part if isinstance(part, list) else [part])]
    joined = ' '.join(cmd)
    assert '--can-drive vcan_drive' in joined and '--can-aux vcan_aux' in joined


def test_simref_binary_from_another_commit_is_blocked(sil_launch, tmp_path):
    actions = sil_launch.can_nodes(SIMREF_CONFIG, 'simref', True, _fake_binary(tmp_path, '0' * 40))
    assert not any(isinstance(a, Node) and a.node_executable == 'simref_bridge' for a in actions)
    assert _blocked(actions)


def test_missing_simref_binary_is_blocked(sil_launch, tmp_path):
    actions = sil_launch.can_nodes(SIMREF_CONFIG, 'simref', True, str(tmp_path / 'none'))
    assert _blocked(actions)


def test_nav2_only_with_simref(sil_launch):
    with pytest.raises(RuntimeError):
        sil_launch.build_actions('host', False, False, False, HOST_CONFIG, '', '', nav2=True)


def test_simref_config_differs_from_sil_yaml_only_where_marked():
    with open(HOST_CONFIG, encoding='utf-8') as f:
        host = yaml.safe_load(f)
    with open(SIMREF_CONFIG, encoding='utf-8') as f:
        sref = yaml.safe_load(f)
    assert sref['buses'] == host['buses']
    assert sref['lift']['permit']['timeout_ms'] == 150 and sref['lift']['permit']['period_ms'] == 50
    assert sref['drive']['commissioned_link_loss_ms'] == host['drive']['link_loss_ms']
    for k in ('commissioned_link_loss_ms', 'cia402_statusword'):
        sref['drive'].pop(k)
    assert sref['drive'] == host['drive']
    sref['lift']['permit']['timeout_ms'] = host['lift']['permit']['timeout_ms']
    assert sref['lift'] == host['lift']
    assert sref['bms'] == host['bms']


def test_link_poll_never_blocks(fake_reference):
    """poll() runs in the bridge's executor next to the /cmd_vel callback: with no status
    pending it must return at once (a blocking poll delayed /cmd_vel by up to 50 ms a call)."""
    _, path = fake_reference
    link = SimrefLink(path, epoch=1)
    try:
        t0 = time.monotonic()
        for _ in range(20):
            assert link.poll() is None
        assert time.monotonic() - t0 < 0.02
    finally:
        link.close()
