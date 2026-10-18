#!/usr/bin/env python3
"""Check a recorded docking run against the issue #35 command-chain criteria.

Input is the CSV from cmdmon.py. Offline; needs no ROS.

C1 during docking, /cmd_vel has exactly one publisher and it is collision_monitor
C2 collision_monitor has no lifecycle transition during docking (stays active)
C3 every source is rate limited before the protective stage: per message at 20 Hz,
   speed may rise by at most 0.05 m/s (max_accel 1.0 m/s^2), fall by at most
   0.125 m/s (max_decel -2.5 m/s^2) and change by at most 0.15 rad/s (3.0 rad/s^2).
   Larger steps on the final /cmd_vel are allowed only where collision_monitor
   changed the smoothed command (its stop/approach output is deliberately not
   smoothed); a step whose value matches no smoothed command is a bypass
C4 the run reached 'docked'
usage: check_run.py RUN.csv
"""
import csv
import sys

STEP_UP, STEP_DOWN, STEP_W, TOL = 0.05, 0.125, 0.15, 1e-3


def _monitor_reduced(cur, sm):
    """collision_monitor only scales a command down or zeroes it."""
    return sm is not None and all(abs(c) <= abs(m) + 1e-4 for c, m in zip(cur, sm))


def main(path):
    rows = list(csv.DictReader(open(path)))
    st = [float(r['sim']) for r in rows if r['kind'] == 'status' and r['extra'] == 'docking']
    en = [float(r['sim']) for r in rows if r['kind'] == 'status' and r['extra'] == 'docked']
    res = {}
    res['C4 docked'] = bool(st and en)
    if not res['C4 docked']:
        t0, t1 = (st[0] if st else 0.0), 1e9
    else:
        t0, t1 = st[0], en[0]
    win = [r for r in rows if t0 <= float(r['sim']) <= t1]
    pubs = {r['source'] for r in win if r['kind'] == 'cmd' and r['topic'] == '/cmd_vel'}
    res['C1 single final publisher'] = pubs == {'collision_monitor'}
    lc = [r['extra'] for r in win
          if r['kind'] == 'lifecycle' and r['topic'] == '/collision_monitor']
    res['C2 monitor stays active'] = not lc
    dv = dw = worst_v = 0.0
    last_sm = None
    prev, prev_sm = None, False
    for r in win:
        if r['kind'] != 'cmd':
            continue
        if r['topic'] == '/cmd_vel_smoothed':
            last_sm = (float(r['vx']), float(r['wz']))
        elif r['topic'] == '/cmd_vel':
            cur = (float(r['vx']), float(r['wz']))
            cur_sm = last_sm is not None and all(abs(a - b) < 1e-4 for a, b in zip(cur, last_sm))
            bypass = not cur_sm and not _monitor_reduced(cur, last_sm)
            # smoother-to-smoother steps must respect the limits; monitor reductions and
            # their release back to the smoothed value are the protective stage's own
            # output; any value that is neither is a bypass and its step counts.
            if prev is not None and ((cur_sm and prev_sm) or bypass):
                up = abs(cur[0]) - abs(prev[0])
                limit = STEP_UP if up > 0 else STEP_DOWN
                over_v = max(0.0, abs(cur[0] - prev[0]) - limit)
                worst_v = max(worst_v, over_v)
                dv = max(dv, abs(cur[0] - prev[0]))
                dw = max(dw, abs(cur[1] - prev[1]))
            prev, prev_sm = cur, cur_sm
    res['C3 rate limited'] = worst_v <= TOL and dw <= STEP_W + TOL
    print(f'{path}')
    print(f'  /cmd_vel publisher set(s) during docking: {sorted(pubs)}')
    more = ' ...' if len(lc) > 4 else ''
    print(f'  collision_monitor transitions during docking: {lc[:4]}{more}')
    print(f'  max step not explained by smoother or monitor: {dv:.3f} m/s, {dw:.3f} rad/s'
          f' (linear excess over the accel/decel bound {worst_v:.3f} m/s)')
    for k, ok in res.items():
        print(f'  {"PASS" if ok else "FAIL"} {k}')
    return 0 if all(res.values()) else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
