#!/usr/bin/env python3
"""Pin the CTL-017 object dictionary of openamr-upperbody-fw for the CAN3 tests.

Reads docs/ctl017_object_dictionary.md of an openamr-upperbody-fw checkout at the commit
named in openamrobot_sil/renode_fw.py (CTL017_OD_COMMIT) and writes or checks
config/ctl017_od_pin.json: the commit, the SHA-256 of the file, the identifier table of its
section 1 and the generated object table of its section 10. Read only: nothing in
openamr-upperbody-fw is changed.

Usage: sil_pin_ctl017_od.py write|check UPPERBODY_FW_CHECKOUT PIN_JSON
       check exits 1 when the checkout is not at the pinned commit or the pin differs.
"""
import hashlib
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from openamrobot_sil import renode_fw  # noqa: E402

BEGIN, END = '<!-- BEGIN GENERATED OD TABLE', '<!-- END GENERATED OD TABLE'


def cells(line):
    return [c.strip() for c in line.strip().strip('|').split('|')]


def parse(text):
    cob = {}
    for line in text.split('## 2.')[0].splitlines():
        c = cells(line)
        if len(c) == 3 and re.fullmatch(r'(rx )?0x[0-9A-Fa-f]{3}.*', c[1]):
            cob[c[0]] = c[1]
    objects = []
    for line in text[text.index(BEGIN):text.index(END)].splitlines():
        c = cells(line)
        if len(c) == 7 and c[0].startswith('0x'):
            objects.append({'index': int(c[0], 16), 'sub': int(c[1]), 'name': c[2],
                            'type': c[3], 'access': c[4], 'pdo': c[5] == 'yes',
                            'default': int(c[6], 0)})
    return cob, objects


def build(checkout):
    head = subprocess.run(['git', '-C', checkout, 'rev-parse', 'HEAD'], check=True,
                          capture_output=True, text=True).stdout.strip()
    if head != renode_fw.CTL017_OD_COMMIT:
        sys.exit('FAILED: %s is at %s, pinned %s' % (checkout, head, renode_fw.CTL017_OD_COMMIT))
    with open(os.path.join(checkout, renode_fw.CTL017_OD_PATH), 'rb') as f:
        blob = f.read()
    cob, objects = parse(blob.decode('utf-8'))
    return {'repository': renode_fw.CTL017_OD_REPOSITORY, 'commit': head,
            'path': renode_fw.CTL017_OD_PATH, 'sha256': hashlib.sha256(blob).hexdigest(),
            'status': 'proposal v1, not adopted by its owner, not hardware-tested',
            'cob_ids': cob, 'objects': objects}


def main(mode, checkout, pin_path):
    pin = build(checkout)
    if mode == 'write':
        with open(pin_path, 'w', encoding='utf-8') as f:
            json.dump(pin, f, indent=1, sort_keys=True)
            f.write('\n')
        print('wrote %s: %d objects, sha256 %s' % (pin_path, len(pin['objects']), pin['sha256']))
        return
    with open(pin_path, encoding='utf-8') as f:
        stored = json.load(f)
    if stored != pin:
        sys.exit('FAILED: %s differs from %s at %s' % (pin_path, pin['path'], pin['commit']))
    print('ok: %s equals %s at %s (%d objects)' % (
        pin_path, pin['path'], pin['commit'], len(pin['objects'])))


if __name__ == '__main__':
    if len(sys.argv) != 4 or sys.argv[1] not in ('write', 'check'):
        sys.exit(__doc__)
    main(*sys.argv[1:])
