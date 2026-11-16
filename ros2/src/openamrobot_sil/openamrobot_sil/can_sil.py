"""Access to the vendored openamr-platform-fw tools/can_sil modules, unmodified.

The vendored files import each other by flat module name. This module adds their directory
to sys.path once and re-exports them. Provenance: vendor/can_sil/PROVENANCE.md.
"""
import os
import sys

VENDOR_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'vendor', 'can_sil')
PINNED_SHA = 'ad4b0237e1822cbe4fc245d55d2fd68fb6e5209d'

if VENDOR_DIR not in sys.path:
    sys.path.insert(0, VENDOR_DIR)

import canopen_common  # noqa: E402
import ctl017_emulator  # noqa: E402
import ctl017_od_v1  # noqa: E402
import zlac8015d_emulator  # noqa: E402

__all__ = ['canopen_common', 'ctl017_emulator', 'ctl017_od_v1', 'zlac8015d_emulator',
           'VENDOR_DIR', 'PINNED_SHA']
