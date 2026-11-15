"""FirmwareTxAudit: the check of every frame the image's FDCAN peripherals transmitted.

Host tests of the rules. The Renode tests feed it the real tap. Each control below gives the
audit one frame the image must never send and requires a finding: a clean audit means
something only because these fail it.
"""
import pytest

from openamrobot_sil import renode_fw

UPLOAD = bytes([0x40, 0x6C, 0x60, 0x01, 0, 0, 0, 0])


def kinds(label, ident, data, **kw):
    return renode_fw.FirmwareTxAudit().classify(label, ident, bytes(data), **kw)


def test_uploads_from_fdcan1_to_node_1_are_the_only_allowed_frames():
    audit = renode_fw.FirmwareTxAudit()
    for index, sub in ((0x606C, 1), (0x606C, 2), (0x6041, 0), (0x603F, 0), (0x2026, 5)):
        audit.feed('fdcan1', 0x601, bytes([0x40, index & 0xFF, index >> 8, sub, 0, 0, 0, 0]))
    s = audit.summary()
    assert s['frames'] == s['uploads'] == 5 and s['violations'] == 0
    assert all(count == 0 for count in s['findings'].values())
    assert s['upload_objects']['606C:01'] == 1 and s['by_label'] == {'fdcan1': 5}


def test_reading_a_restricted_object_is_not_a_write():
    for index in renode_fw.RESTRICTED_INDEXES:
        data = bytes([0x40, index & 0xFF, index >> 8, 0, 0, 0, 0, 0])
        assert kinds('fdcan1', 0x601, data) == []


@pytest.mark.parametrize('label,ident,data,expected', [
    ('fdcan1', 0x000, [0x01, 0x01], ['nmt']),                       # NMT start node 1
    ('fdcan1', 0x000, [0x81, 0x00], ['nmt']),                       # NMT reset all nodes
    ('fdcan3', 0x000, [0x01, 0x10], ['nmt']),                       # NMT on CAN3
    ('fdcan1', 0x601, [0x2B, 0x40, 0x60, 0, 0x0F, 0, 0, 0], ['sdo_download', 'restricted_write']),
    ('fdcan1', 0x601, [0x23, 0xFF, 0x60, 1, 100, 0, 0, 0], ['sdo_download', 'restricted_write']),
    ('fdcan1', 0x601, [0x2B, 0x00, 0x20, 0, 250, 0, 0, 0], ['sdo_download', 'restricted_write']),
    ('fdcan1', 0x601, [0x2F, 0x60, 0x60, 0, 3, 0, 0, 0], ['sdo_download']),      # 6060h
    ('fdcan1', 0x601, [0x21, 0x60, 0x60, 0, 8, 0, 0, 0], ['sdo_download']),      # segmented
    ('fdcan1', 0x601, [0x00, 1, 2, 3, 4, 5, 6, 7], ['sdo_download']),            # segment
    ('fdcan1', 0x601, [0xC6, 0x40, 0x60, 0, 2, 0, 0, 0], ['sdo_download']),      # block
    ('fdcan3', 0x610, [0x2F, 0x11, 0x20, 1, 1, 0, 0, 0], ['sdo_download']),      # CTL-017
    ('fdcan3', 0x610, [0x2F, 0x00, 0x20, 1, 1, 0, 0, 0], ['sdo_download', 'restricted_write']),
    ('fdcan1', 0x201, [0, 0, 0, 0, 0, 0, 0, 0], ['restricted_write']),           # 60FFh by PDO
    ('fdcan1', 0x301, [0x0F, 0x00], ['restricted_write']),                       # 6040h by PDO
    ('fdcan3', 0x210, [1, 1, 0], ['restricted_write']),                          # 200h range
    ('fdcan3', 0x410, [1, 0, 0, 0, 0, 0], ['pdo_write']),                        # lift permit
    ('fdcan1', 0x501, [0], ['pdo_write']),
    ('fdcan1', 0x601, [0x60, 0, 0, 0, 0, 0, 0, 0], ['sdo_other']),               # upload segment
    ('fdcan1', 0x601, [0x80, 0x6C, 0x60, 1, 0, 0, 4, 5], ['sdo_other']),         # abort
    ('fdcan1', 0x601, [0x40, 0x6C, 0x60, 1], ['sdo_other']),                     # short upload
    ('fdcan1', 0x601, [0x40, 0x6C, 0x60, 1, 1, 0, 0, 0], ['sdo_other']),         # data not 0
    ('fdcan1', 0x602, list(UPLOAD), ['not_allowed']),                            # other node
    ('fdcan2', 0x601, list(UPLOAD), ['not_allowed']),                            # other bus
    ('fdcan3', 0x610, list(UPLOAD), ['not_allowed']),
    ('fdcan1', 0x080, [], ['other']),                                            # SYNC
    ('fdcan3', 0x701, [0x05], ['other']),                                        # heartbeat
    ('fdcan1', 0x181, [0, 0], ['other']),                                        # transmit PDO
    ('fdcan1', 0x600, list(UPLOAD), ['other']),                                  # node 0
])
def test_control_every_forbidden_frame_is_a_finding(label, ident, data, expected):
    audit = renode_fw.FirmwareTxAudit()
    assert audit.feed(label, ident, bytes(data), 1.5) == expected
    s = audit.summary()
    assert s['violations'] == 1 and s['uploads'] == 0
    assert {k for k, v in s['findings'].items() if v} == set(expected)
    assert s['examples'] and label in s['examples'][0]


def test_extended_remote_and_fd_frames_are_findings():
    assert kinds('fdcan2', 0x18900140, [0] * 8, extended=True) == ['other']   # a Daly poll
    assert kinds('fdcan1', 0x601, [], remote=True) == ['other']
    assert kinds('fdcan1', 0x601, list(UPLOAD), fd=True) == ['other']


def test_one_forbidden_frame_in_a_long_run_is_found():
    audit = renode_fw.FirmwareTxAudit()
    for i in range(5000):
        audit.feed('fdcan1', 0x601, UPLOAD, i * 0.02)
    assert audit.summary()['violations'] == 0
    audit.feed('fdcan1', 0x601, bytes([0x2B, 0x40, 0x60, 0, 6, 0, 0, 0]), 100.0)
    for i in range(5000):
        audit.feed('fdcan1', 0x601, UPLOAD, 100.0 + i * 0.02)
    s = audit.summary()
    assert s['violations'] == 1 and s['findings']['restricted_write'] == 1
    assert s['uploads'] == 10000 and 't=100.000000' in s['examples'][0]


def test_tap_file_round_trip(tmp_path):
    path = tmp_path / 'fwtx.log'
    path.write_text('(0.118000) fdcan1 601#406C600100000000\n'
                    '\n'
                    '(0.138000) fdcan1 601#2B4060000F000000\n'
                    '(0.140000) fdcan2 18900140#0000000000000000\n'
                    '(0.150000) fdcan3 000#0110\n'
                    '(0.160000) fdcan1 601#R\n'
                    '(0.170000) fdcan1 601##0406C600100000000\n')
    frames = renode_fw.read_candump(str(path))
    assert [f['id'] for f in frames] == [0x601, 0x601, 0x18900140, 0, 0x601, 0x601]
    assert frames[2]['extended'] and frames[4]['remote'] and frames[5]['fd']
    assert frames[0]['data'] == UPLOAD and frames[0]['t'] == pytest.approx(0.118)
    s = renode_fw.FirmwareTxAudit().feed_file(str(path)).summary()
    assert s['frames'] == 6 and s['uploads'] == 1 and s['violations'] == 5
    assert s['findings']['nmt'] == 1 and s['findings']['sdo_download'] == 1
    assert s['by_label'] == {'fdcan1': 4, 'fdcan2': 1, 'fdcan3': 1}


def test_bus_audit_counts_nmt_and_receive_pdos_of_the_node():
    audit = renode_fw.SdoAudit(1)
    audit.feed(0x000, bytes([1, 1]), 0.0)
    audit.feed(0x201, bytes(8), 0.1)
    audit.feed(0x301, bytes([6, 0]), 0.2)
    audit.feed(0x202, bytes(8), 0.3)         # another node
    audit.feed(0x181, bytes(8), 0.4)         # a transmit PDO
    s = audit.summary()
    assert s['nmt'] == 1 and s['pdo_writes'] == 2 and s['requests'] == 0


def test_all_three_fdcans_are_tapped_with_and_without_bridges():
    for aux in (None, ('vcan_bms', 'vcan_aux')):
        text = renode_fw.resc('/e', '/a', '/c', '/h', 'vcan_drive', aux_interfaces=aux,
                              fw_tap='/t', hub_tap='/u')
        assert text.count('CreateSocketCANBridge') == (3 if aux else 1)
        for fdcan, hub in renode_fw.FW_CAN:
            assert 'sil_fw_tap "sysbus.%s" "%s" @/t' % (fdcan, fdcan) in text
            assert 'sil_hub_tap "%s" "%s" @/u' % (hub, hub) in text
            assert 'connector Connect sysbus.%s %s' % (fdcan, hub) in text
    with pytest.raises(ValueError, match='vcan only'):
        renode_fw.resc('/e', '/a', '/c', '/h', 'can0')
    with pytest.raises(ValueError, match='vcan only'):
        renode_fw.resc('/e', '/a', '/c', '/h', 'vcan_drive', aux_interfaces=('can1', 'vcan_aux'))
    with pytest.raises(ValueError, match='both answer'):
        renode_fw.resc('/e', '/a', '/c', '/h', 'vcan_drive', stub='/stub.py')


def test_time_rule_bounds():
    a = {'virtual_us': 1000000, 'wall_us': 4000000}
    b = {'virtual_us': 2000000, 'wall_us': 8000000}
    assert renode_fw.time_ratio(a, b) == 0.25 and renode_fw.time_ratio(a, a) is None
    # A 300 ms firmware timeout cannot fire before 295 ms of wall time have passed.
    assert renode_fw.not_before_wall_s(300000) == pytest.approx(0.295)
    assert renode_fw.not_before_wall_s(1000) == 0.0
    # The liveness wait is a harness bound and grows with the virtual time asked for.
    assert renode_fw.liveness_wait_s(300000) == pytest.approx(16.0)
    assert renode_fw.liveness_wait_s(3000000) > renode_fw.liveness_wait_s(300000)
