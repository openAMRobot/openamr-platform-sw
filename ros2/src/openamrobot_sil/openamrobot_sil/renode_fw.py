"""Firmware in the loop: the openamr-platform-fw telemetry image in Renode on the drive bus.

What is real: the firmware binary, built from openamr-platform-fw at FW_COMMIT with the CMake
preset FW_PRESET. What is not: the MCU (Renode's STM32H743 description plus the platform-fw
STM32H723 additions), the drive (the SIL ZLAC8015D emulator), the bus (Linux vcan).

This module holds the pinned values and the logic that needs no ROS, no Renode and no vcan:
the Renode script, the banner check, the console parser, the wheel speed conversion and the
SDO audit of the drive bus. The ROS nodes and the tests use it. It decides nothing about
motion: it reads what the image prints and what is on the bus.

Run as a script, it prints the pinned values as shell assignments for
scripts/sil_renode_firmware.sh.
"""
import json
import math
import sys

# ---- pinned values ----------------------------------------------------------------------
FW_REPOSITORY = 'openAMRobot/openamr-platform-fw'
FW_COMMIT = 'fcf81f8e2fe2fa9000685ab233907adbfadeec2a'
FW_PRESET = 'RenodeTest'
FW_PROJECT_DIR = 'boards/stm32/telemetry_image'
FW_ELF = FW_PROJECT_DIR + '/build/' + FW_PRESET + '/telemetry_image.elf'
FW_ADDITIONS = 'tests/renode/stm32h723-additions.repl'
FW_PROJECT_NAME = 'openamr-platform-fw telemetry_image'
FW_PARAMS = 'test'      # banner "params" of an image built with TLM_TEST_PARAMS (RenodeTest)

# Renode release and tarball hash as pinned in openamr-platform-fw
# .github/workflows/firmware-nightly.yml at FW_COMMIT (RENODE_URL, RENODE_SHA256).
RENODE_VERSION = '1.16.1'
RENODE_URL = ('https://github.com/renode/renode/releases/download/v1.16.1/'
              'renode-1.16.1.linux-portable-dotnet.tar.gz')
RENODE_SHA256 = '00e113cdbd0f5354cf2f64bbe3f5a070d8958409542fca66e45ac97d982938c0'
RENODE_PLATFORM = 'platforms/cpus/stm32h743.repl'
# PWR CSR1 ACTVOSRDY read stub, as in platform-fw tools/ci/renode_boot.py and renode_telemetry.py.
RENODE_CSR1_TAG = 'sysbus Tag <0x58024804, 0x58024807> "CSR1" 0x2000'

# Harness settings (this repository). The image's idle task spins, so Renode's virtual time
# runs as fast as Renode executes instructions divided by the CPU's MIPS setting. 5 MIPS puts
# Renode near wall time on a 4-core runner; the throttle stops it from running ahead.
RENODE_MIPS = 5
THROTTLE_SLICE_MS = 5

# Values of the RenodeTest build (platform-fw tlm_params.h, TLM_TEST_PARAMS block), virtual time.
FW_SDO_TIMEOUT_US = 40000
FW_HB_TIMEOUT_US = 300000
FW_REPORT_PERIOD_US = 100000
FW_TICK_US = 20000

UNKNOWN = 'unknown'
UPLOAD_REQUEST = 0x40
SDO_RX_BASE = 0x600
SDO_TX_BASE = 0x580
# 606Ch is reported in drive units. The SIL ZLAC8015D emulator (vendored platform-fw
# tools/can_sil) uses 0.1 r/min per unit, so that scale is used here. The real drive's unit is
# not settled (platform-fw zlac_telemetry.h TODO(Evans)); this is a SIL value.
RPM_PER_UNIT = 0.1
RAD_S_PER_RPM = 2.0 * math.pi / 60.0


class BannerError(RuntimeError):
    """The image that booted is not the pinned RenodeTest build."""


# ---- Renode script ----------------------------------------------------------------------
def resc(elf, additions, console, helpers, can_interface=None, mips=RENODE_MIPS,
         slice_ms=THROTTLE_SLICE_MS):
    """Renode monitor script: machine, image, console, CAN1 and the SocketCAN bridge.

    The machine set-up is the one of platform-fw tools/ci/renode_telemetry.py: the STM32H743
    description, the platform-fw additions, the CSR1 tag, a reset macro that loads the ELF,
    a file backend on USART3 and a CAN hub with FDCAN1. In place of the platform-fw drive stub,
    Renode's SocketCAN bridge joins the hub to the Linux interface can_interface, where the
    SIL drive emulator runs. can_interface None leaves the hub without a bridge (Renode-only
    tests on machines without vcan; the image then sees a silent drive).
    """
    lines = [
        'mach create "openamr"',
        'machine LoadPlatformDescription @%s' % RENODE_PLATFORM,
        'machine LoadPlatformDescription @%s' % additions,
        RENODE_CSR1_TAG,
        'macro reset',
        '"""',
        '    sysbus LoadELF @%s' % elf,
        '    cpu VectorTableOffset 0x08000000',
        '"""',
        'runMacro $reset',
        'cpu PerformanceInMips %d' % int(mips),
        'sysbus.usart3 CreateFileBackend @%s true' % console,
        'emulation CreateCANHub "can1"',
        'connector Connect sysbus.fdcan1 can1',
    ]
    if can_interface:
        lines += [
            'machine CreateSocketCANBridge "can1_bridge" "%s"' % can_interface,
            'connector Connect can1_bridge can1',
        ]
    lines += [
        'include @%s' % helpers,
        'sil_follow_systick',
        'sil_throttle_start %d' % int(slice_ms),
        'logLevel 3',
        'start',
    ]
    return '\n'.join(lines) + '\n'


def reset_commands(slice_ms=THROTTLE_SLICE_MS):
    """Monitor commands for a machine reset inside the same Renode instance.

    The machine is paused first, so the image does not run before the SystemCoreClock hook and
    the throttle are set again: Renode drops both on a reset.
    """
    return ['pause', 'machine Reset', 'sil_follow_systick',
            'sil_throttle_start %d' % int(slice_ms), 'start']


# ---- banner -----------------------------------------------------------------------------
def check_banner(banner, commit=FW_COMMIT, params=FW_PARAMS):
    """Raise BannerError unless the banner is the one of the pinned RenodeTest build."""
    if banner.get('project') != FW_PROJECT_NAME:
        raise BannerError('banner project %r, expected %r' % (
            banner.get('project'), FW_PROJECT_NAME))
    if banner.get('params') != params:
        raise BannerError('banner params %r, expected %r: the image is not the %s build' % (
            banner.get('params'), params, FW_PRESET))
    printed = banner.get('commit')
    if (not isinstance(printed, str) or len(printed) < 7 or printed.endswith('-dirty')
            or not commit.startswith(printed)):
        raise BannerError('banner commit %r is not the pinned %s' % (printed, commit))
    return True


# ---- console ----------------------------------------------------------------------------
class ConsoleParser:
    """Splits the USART3 byte stream into lines and decodes the JSON records."""

    def __init__(self):
        self._partial = ''
        self.bad_lines = []

    def feed(self, text):
        """Return [(raw line, record or None)] for every complete line in text."""
        self._partial += text
        *complete, self._partial = self._partial.split('\n')
        out = []
        for raw in complete:
            raw = raw.strip('\r\x00 ')
            if not raw:
                continue
            try:
                record = json.loads(raw)
            except ValueError:
                record = None
            if not isinstance(record, dict) or 't' not in record:
                self.bad_lines.append(raw[:120])
                record = None
            out.append((raw, record))
        return out


def wheel_rad_s(status):
    """(ch1, ch2) wheel speeds in rad/s from a status line. NaN where the image does not report
    a valid sample. A value that is not known is never turned into 0."""
    out = []
    for ch in ('ch1', 'ch2'):
        o = status['drive']['wheel'][ch]
        raw = o.get('raw')
        if o.get('v') == 'valid' and isinstance(raw, int) and not isinstance(raw, bool):
            out.append(raw * RPM_PER_UNIT * RAD_S_PER_RPM)
        else:
            out.append(float('nan'))
    return tuple(out)


# ---- SDO audit of the drive bus ---------------------------------------------------------
class SdoAudit:
    """Checks every SDO frame of one node on a captured bus.

    Request: COB-ID 600h + node. Response: 580h + node. A request is outstanding until a
    response with the same index and sub-index (or an abort for it) arrives, or until
    min_gap_s has passed (the requester's timeout, measured on the capture clock).

    Findings:
      - non_upload: a request whose command byte is not 40h (initiate upload). The firmware
        image cannot send one, so on the drive bus such a frame comes from another client.
      - overlaps: a request while another one is still outstanding.
    Who sent an upload request cannot be seen on the wire; the caller compares `uploads` with
    the number of requests the image says it sent (its "tx" counter).
    """

    def __init__(self, node_id=1, min_gap_s=0.020):
        self.node_id = node_id
        self.min_gap_s = min_gap_s
        self.requests = 0
        self.uploads = 0
        self.non_upload = 0
        self.responses = 0
        self.unmatched_responses = 0
        self.overlaps = 0
        self.timeouts = 0
        self.examples = []
        self._open = None    # (t, index, sub)

    def _note(self, text):
        if len(self.examples) < 20:
            self.examples.append(text)

    def feed(self, arbitration_id, data, t, extended=False):
        if extended:
            return
        data = bytes(data)
        if arbitration_id == SDO_RX_BASE + self.node_id:
            self.requests += 1
            cs = data[0] if data else None
            index = int.from_bytes(data[1:3], 'little') if len(data) >= 3 else None
            sub = data[3] if len(data) >= 4 else None
            if cs != UPLOAD_REQUEST or len(data) != 8 or data[4:] != bytes(4):
                self.non_upload += 1
                self._note('t=%.6f non-upload request %s' % (t, data.hex()))
            else:
                self.uploads += 1
            if self._open is not None:
                if t - self._open[0] < self.min_gap_s:
                    self.overlaps += 1
                    self._note('t=%.6f request %s while %04X:%02X from t=%.6f is outstanding' % (
                        t, data.hex(), self._open[1], self._open[2], self._open[0]))
                else:
                    self.timeouts += 1
            self._open = (t, index, sub)
        elif arbitration_id == SDO_TX_BASE + self.node_id:
            self.responses += 1
            index = int.from_bytes(data[1:3], 'little') if len(data) >= 3 else None
            sub = data[3] if len(data) >= 4 else None
            if self._open is not None and (index, sub) == self._open[1:]:
                self._open = None
            else:
                self.unmatched_responses += 1

    def summary(self):
        return {'node': self.node_id, 'requests': self.requests, 'uploads': self.uploads,
                'non_upload': self.non_upload, 'responses': self.responses,
                'unmatched_responses': self.unmatched_responses, 'overlaps': self.overlaps,
                'timeouts': self.timeouts, 'min_gap_s': self.min_gap_s,
                'examples': list(self.examples)}


def candump_line(t, interface, arbitration_id, data, extended=False):
    """One frame in candump -L format."""
    ident = ('%08X' if extended else '%03X') % arbitration_id
    return '(%.6f) %s %s#%s' % (t, interface, ident, bytes(data).hex().upper())


def _shell():
    for name in ('FW_REPOSITORY', 'FW_COMMIT', 'FW_PRESET', 'FW_PROJECT_DIR', 'FW_ELF',
                 'FW_ADDITIONS', 'RENODE_VERSION', 'RENODE_URL', 'RENODE_SHA256'):
        print("%s='%s'" % (name, globals()[name]))


if __name__ == '__main__':
    if sys.argv[1:] == ['shell']:
        _shell()
    else:
        sys.exit('usage: renode_fw.py shell')
