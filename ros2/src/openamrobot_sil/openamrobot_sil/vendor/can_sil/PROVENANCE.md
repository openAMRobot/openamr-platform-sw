# Vendored: openamr-platform-fw tools/can_sil

- Source: https://github.com/openAMRobot/openamr-platform-fw, directory `tools/can_sil`.
- Pinned source commit: `f88bf4820d0db7d27ffc6dfad905f57c1483283d` (correction from firmware PR #75,
  stacked on the authoritative CAN-SIL source branch in PR #53; branch
  `ci/can-sil-track-vcan-skips-74`, fetched 25 Nov 2026). The correction is being verified
  for the digital-twin simulation branch; it is not a production firmware release or a merge to `main`.
  Once #75 is incorporated into #53, update this pin to the accepted upstream branch commit.
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
