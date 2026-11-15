#!/usr/bin/env python3
"""Write the synthetic drive bus trace traces/zlac_synthetic_v1.candump (candump -L format).

Not a capture. It has the shape of one: a boot-up message, a heartbeat every 100 ms, and
per 100 ms one request and one answer for each object the telemetry image reads. The wheel
values ramp, so a replay can be told from a constant. Channel 2 is the negative of channel 1.
Units are drive units as the SIL uses them (0.1 r/min). The master-side frames (requests, one
NMT start, one receive PDO) are in the trace on purpose: a replay must leave them out.

Usage: sil_make_trace.py OUT_FILE
"""
import struct
import sys

SECONDS = 6.0
T0 = 1700000000.0


def wheel(step):
    """Ramp 0 to 300 units in 2 s, hold, ramp down in the last 2 s. Never exactly 0 after
    the first sample, so a reported 0 would be a wrong value, not a recorded one."""
    t = step * 0.1
    if t < 2.0:
        return int(150 * t) + 1
    if t < 4.0:
        return 301
    return max(1, 301 - int(150 * (t - 4.0)))


def main(path):
    lines = []

    def add(t, ident, data):
        lines.append('(%.6f) can0 %03X#%s' % (T0 + t, ident, bytes(data).hex().upper()))

    def upload(t, index, sub, size, value):
        head = struct.pack('<HB', index, sub)
        add(t, 0x601, bytes([0x40]) + head + bytes(4))
        cs = 0x43 | ((4 - size) << 2)
        raw = (value & 0xFFFFFFFF).to_bytes(4, 'little')[:size] + bytes(4 - size)
        add(t + 0.0008, 0x581, bytes([cs]) + head + raw)

    add(0.0, 0x701, [0x00])
    add(0.002, 0x000, [0x01, 0x01])                       # master side: NMT start
    add(0.004, 0x201, struct.pack('<ii', 100, 100))       # master side: a receive PDO
    for step in range(int(SECONDS * 10)):
        t = 0.05 + step * 0.1
        add(t, 0x701, [0x05])
        w = wheel(step)
        upload(t + 0.010, 0x606C, 1, 4, w)
        upload(t + 0.030, 0x606C, 2, 4, -w)
        upload(t + 0.050, 0x6041, 0, 2, 0x0237)
        upload(t + 0.070, 0x603F, 0, 4, 0)
        upload(t + 0.090, 0x2026, 5, 2, 1)
    lines.sort(key=lambda line: float(line[1:line.index(')')]))
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main(sys.argv[1])
