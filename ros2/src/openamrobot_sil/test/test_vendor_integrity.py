"""The vendored platform-fw can_sil files are byte for byte the pinned ones."""
import hashlib
import os

from conftest import REPO_ROOT
from openamrobot_sil import can_sil


def test_vendored_files_match_the_recorded_hashes():
    with open(os.path.join(can_sil.VENDOR_DIR, 'SHA256SUMS'), encoding='utf-8') as f:
        rows = [line.split() for line in f if line.strip()]
    assert len(rows) == 8
    for digest, name in rows:
        with open(os.path.join(can_sil.VENDOR_DIR, name), 'rb') as f:
            assert hashlib.sha256(f.read()).hexdigest() == digest, name


def test_pinned_sha_is_recorded_in_provenance_and_notice():
    for path in (os.path.join(can_sil.VENDOR_DIR, 'PROVENANCE.md'),
                 os.path.join(REPO_ROOT, 'NOTICE.md')):
        with open(path, encoding='utf-8') as f:
            assert can_sil.PINNED_SHA in f.read(), path


def test_shim_exposes_the_vendored_modules():
    assert can_sil.zlac8015d_emulator.__file__.startswith(can_sil.VENDOR_DIR)
    assert can_sil.ctl017_emulator.NODE_ID == 0x10
    assert len(can_sil.ctl017_od_v1.OD_V1) > 100
