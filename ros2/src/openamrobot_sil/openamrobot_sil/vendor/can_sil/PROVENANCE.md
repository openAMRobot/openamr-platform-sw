# Vendored: openamr-platform-fw tools/can_sil

- Source: https://github.com/openAMRobot/openamr-platform-fw, directory `tools/can_sil`.
- Pinned commit: `ad4b0237e1822cbe4fc245d55d2fd68fb6e5209d` (head of draft PR #53, branch
  `night-shift/fw2-can-sil-fix-2026-11-14`, fetched 14 Nov 2026). The PR is a draft proposal
  and is not merged. When it merges or changes, re-vendor at the new SHA.
- Licence: MIT, copyright OpenAMRobot (Botshare LTD). `LICENSE` here is the licence file of
  that repository at the same commit.
- Files, byte for byte (`SHA256SUMS`, checked by `test/test_vendor_integrity.py`):
  `canopen_common.py`, `zlac8015d_emulator.py`, `ctl017_emulator.py`, `ctl017_od_v1.py`,
  `tests/conftest.py`, `tests/test_zlac8015d.py`, `tests/test_ctl017.py`.
- Modifications: none. `__init__.py` and this file are additions of this repository.
- Not vendored: `can_trace.py`, `gen_ctl017_od.py`, `tools/bench` and their tests
  (`test_bench_scripts.py`, `test_can_trace.py`, `test_ctl017_od.py`,
  `test_ctl017_od_drift.py`). They are bench and generator tools that this package does not
  use. They stay in openamr-platform-fw.
- The vendored modules import each other by flat name (`from canopen_common import ...`).
  `openamrobot_sil/can_sil.py` puts this directory on `sys.path` and re-exports them, so the
  files need no edit.
