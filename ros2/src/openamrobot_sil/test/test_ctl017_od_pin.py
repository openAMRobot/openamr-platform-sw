"""The CTL-017 object dictionary pinned from openamr-upperbody-fw, against what the SIL uses.

config/ctl017_od_pin.json is the generated table of docs/ctl017_object_dictionary.md at the
commit in renode_fw.CTL017_OD_COMMIT (scripts/sil_pin_ctl017_od.py; CI checks the pin against
a fetch of that commit). The CAN3 reception test of the firmware image is BLOCKED until an
image initialises FDCAN3; these tests keep the SIL side and the pin equal meanwhile.
The dictionary is a proposal of its owner repository, not adopted and not hardware-tested.
"""
import json
import os

from conftest import PACKAGE_DIR
from openamrobot_sil import config, renode_fw
from openamrobot_sil.can_sil import ctl017_od_v1

PIN = os.path.join(PACKAGE_DIR, 'config', 'ctl017_od_pin.json')


def pin():
    with open(PIN, encoding='utf-8') as f:
        return json.load(f)


def test_pin_names_the_commit_and_file():
    p = pin()
    assert p['repository'] == renode_fw.CTL017_OD_REPOSITORY == 'openAMRobot/openamr-upperbody-fw'
    assert p['commit'] == renode_fw.CTL017_OD_COMMIT and len(p['commit']) == 40
    assert p['path'] == 'docs/ctl017_object_dictionary.md' and len(p['sha256']) == 64
    assert len(p['objects']) == 126


def test_emulator_dictionary_equals_the_pinned_table():
    pinned = [(o['index'], o['sub'], o['name'], o['type'], o['access'], o['pdo'], o['default'])
              for o in pin()['objects']]
    assert pinned == list(ctl017_od_v1.OD_V1)


def test_sil_identifiers_equal_the_pinned_identifiers():
    cob = pin()['cob_ids']
    cfg = config.load(os.path.join(PACKAGE_DIR, 'config', 'sil.yaml'))
    node = cfg['buses']['aux']['ctl017_node_id']
    assert node == 0x10
    assert int(cob['NMT'], 16) == 0x000 and int(cob['EMCY'], 16) == 0x080 + node
    assert int(cob['TPDO1 LiftStatus'], 16) == 0x180 + node
    assert int(cob['TPDO2 LiftStatus2'], 16) == 0x280 + node
    assert int(cob['RPDO1 LiftCommand'], 16) == 0x200 + node
    assert int(cob['RPDO2 AuxCommand'], 16) == 0x300 + node
    assert cob['SDO server'] == 'rx 0x%03x, tx 0x%03x' % (0x600 + node, 0x580 + node)
    assert int(cob['Heartbeat CTL-017'], 16) == 0x700 + node
    assert int(cob['Heartbeat consumed'], 16) == 0x700 + cfg['buses']['aux']['core_node_id']


def test_object_2000h_is_a_different_object_on_can3_than_on_can1():
    """2000h is the link-loss time on the drive (CAN1) and LiftStatus, read only, on the
    CTL-017 (CAN3). The firmware audit treats a write to 2000h as a finding on every bus."""
    lift_status = [o for o in pin()['objects'] if o['index'] == 0x2000]
    assert len(lift_status) == 8 and all(o['access'] in ('ro', 'const') for o in lift_status)
    assert 0x2000 in renode_fw.RESTRICTED_INDEXES
