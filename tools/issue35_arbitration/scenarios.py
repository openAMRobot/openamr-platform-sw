#!/usr/bin/env python3
"""Issue #35 arbitration scenarios, SIMULATION ONLY (Gazebo, prototype launch).

Never run against a robot: the scenarios publish velocity commands. The script
refuses to start unless /clock is publishing (a simulator is running).

Each scenario publishes synthetic sources into the twist_mux inputs, records
/cmd_vel_mux, /cmd_vel (final) and Gazebo ground truth (/gt_poses, see
README.md), and checks a pass criterion. Times are SIM time (the simulator runs
slower than real time). Prints one RESULT line per scenario and writes CSV.

usage: scenarios.py OUTDIR [scenario ...]
"""
import csv
import math
import os
import sys
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

ROBOT_INDEX = 6     # robot model entry in /world/walled_world/pose/info (names are dropped)


class Rig(Node):

    def __init__(self, outdir):
        super().__init__('issue35_scenarios')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.outdir = outdir
        self.pubs = {t: self.create_publisher(Twist, t, 10)
                     for t in ('cmd_vel_nav', 'cmd_vel_dock', 'cmd_vel_joy', 'cmd_vel_stop')}
        self.lock_pub = self.create_publisher(Bool, 'stop_lock', 10)
        self.log = []
        self.pose = None
        self.clock_seen = False
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        for t in ('/cmd_vel_mux', '/cmd_vel'):
            self.create_subscription(Twist, t, self._mk(t), 50)
        self.create_subscription(TFMessage, '/gt_poses', self._gt, 10)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_clock(self, _):
        self.clock_seen = True

    def _mk(self, topic):
        def cb(m):
            self.log.append((self.now(), topic, m.linear.x, m.angular.z))
        return cb

    def _gt(self, m):
        if len(m.transforms) > ROBOT_INDEX:
            tf = m.transforms[ROBOT_INDEX].transform
            yaw = 2.0 * math.atan2(tf.rotation.z, tf.rotation.w)
            self.pose = (tf.translation.x, tf.translation.y, yaw)
            self.log.append((self.now(), 'gt', tf.translation.x, yaw))

    def spin_for(self, sim_s, sources=None, rate_hz=20.0):
        """Spin for sim_s seconds, publishing {topic: (vx, wz) or Bool} at rate_hz (wall)."""
        sources = sources or {}
        end = self.now() + sim_s
        while self.now() < end:
            for t, v in sources.items():
                if t == 'stop_lock':
                    self.lock_pub.publish(Bool(data=v))
                else:
                    msg = Twist()
                    msg.linear.x, msg.angular.z = v
                    self.pubs[t].publish(msg)
            t_next = time.monotonic() + 1.0 / rate_hz
            while time.monotonic() < t_next:
                rclpy.spin_once(self, timeout_sec=0.005)

    def series(self, topic, t0=-1e9, t1=1e9):
        return [(t, a, b) for t, k, a, b in self.log if k == topic and t0 <= t <= t1]

    def dump(self, name):
        with open(os.path.join(self.outdir, f'{name}.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['sim', 'topic', 'a', 'b'])
            w.writerows(self.log)
        self.log = []


def first_time(series, pred, after):
    for t, a, b in series:
        if t >= after and pred(a, b):
            return t
    return None


def settle_zero(rig):
    rig.spin_for(1.0, {'cmd_vel_nav': (0.0, 0.0)})
    rig.spin_for(1.5)
    rig.log = []


def gt_yaw_travel(rig, t0, t1):
    g = rig.series('gt', t0, t1)
    total, prev = 0.0, None
    for _, _, yaw in g:
        if prev is not None:
            d = (yaw - prev + math.pi) % (2 * math.pi) - math.pi
            total += abs(d)
        prev = yaw
    return total


# Scenarios use rotation in place (wz) so the robot stays clear of walls; the
# translation cases use a small forward speed in open space.

def s_priority_handover(rig):
    """Nav 0.3 rad/s; dock 0.2 rad/s preempts it; dock loss hands back to nav."""
    settle_zero(rig)
    t0 = rig.now()
    rig.spin_for(1.0, {'cmd_vel_nav': (0.0, 0.3)})
    t_dock = rig.now()
    rig.spin_for(1.0, {'cmd_vel_nav': (0.0, 0.3), 'cmd_vel_dock': (0.0, 0.2)})
    t_dock_end = rig.now()
    rig.spin_for(1.5, {'cmd_vel_nav': (0.0, 0.3)})
    rig.spin_for(1.0, {'cmd_vel_nav': (0.0, 0.0)})
    mux = rig.series('/cmd_vel_mux', t0)
    sel = first_time(mux, lambda a, b: abs(b - 0.2) < 1e-6, t_dock)
    back = first_time(mux, lambda a, b: abs(b - 0.3) < 1e-6, t_dock_end)
    leak = [x for x in rig.series('/cmd_vel_mux', t_dock + 0.1, t_dock_end)
            if abs(x[2] - 0.3) < 1e-6]
    ok = sel is not None and back is not None and not leak and (back - t_dock_end) <= 0.3 + 0.15
    rig.dump('priority_handover')
    return ok, (f'dock selected {None if sel is None else round(sel - t_dock, 3)} s after start; '
                f'nav back {None if back is None else round(back - t_dock_end, 3)} s after dock '
                f'stopped (docking timeout 0.3 s); nav samples leaked while dock live: '
                f'{len(leak)}')


def s_stop_lock_over_teleop(rig):
    """Teleop 0.4 rad/s; stop lock + zero source engage; teleop must be masked."""
    settle_zero(rig)
    rig.spin_for(1.5, {'cmd_vel_joy': (0.0, 0.4)})
    t_lock = rig.now()
    rig.spin_for(2.0, {'cmd_vel_joy': (0.0, 0.4), 'stop_lock': True,
                       'cmd_vel_stop': (0.0, 0.0)})
    mux = rig.series('/cmd_vel_mux', t_lock + 0.1)
    fin = rig.series('/cmd_vel', t_lock)
    z = first_time(fin, lambda a, b: abs(b) < 1e-3, t_lock)
    leak = [x for x in mux if abs(x[2]) > 1e-6]
    yaw_after = gt_yaw_travel(rig, z or t_lock, t_lock + 2.0)
    rig.spin_for(0.5, {'stop_lock': False})
    rig.dump('stop_lock_over_teleop')
    ok = z is not None and not leak
    return ok, (f'final /cmd_vel zero {None if z is None else round(z - t_lock, 3)} s after lock; '
                f'teleop samples forwarded under lock: {len(leak)}; yaw after zero '
                f'{math.degrees(yaw_after):.1f} deg')


def s_lock_without_zero_source(rig):
    """Lock only, no zero publisher: twist_mux goes silent; record what reaches /cmd_vel."""
    settle_zero(rig)
    rig.spin_for(1.5, {'cmd_vel_joy': (0.0, 0.4)})
    t_lock = rig.now()
    rig.spin_for(3.0, {'cmd_vel_joy': (0.0, 0.4), 'stop_lock': True})
    mux = rig.series('/cmd_vel_mux', t_lock + 0.05)
    fin = rig.series('/cmd_vel', t_lock)
    z = first_time(fin, lambda a, b: abs(b) < 1e-3, t_lock)
    held = [x for x in fin if x[0] < (z or 1e9) and abs(x[2] - 0.4) < 1e-6]
    rig.spin_for(0.5, {'stop_lock': False})
    rig.dump('lock_without_zero_source')
    # Expected (documented behaviour): twist_mux publishes nothing; the smoother holds
    # the last command until velocity_timeout, then ramps to zero.
    ok = not mux and z is not None
    return ok, (f'mux messages after lock: {len(mux)}; final /cmd_vel held 0.4 for '
                f'{(held[-1][0] - t_lock) if held else 0:.2f} s, zero after '
                f'{None if z is None else round(z - t_lock, 3)} s (smoother velocity_timeout)')


def _loss(rig, topic, cmd, name):
    settle_zero(rig)
    rig.spin_for(2.0, {topic: cmd})
    t_loss = rig.now()
    p0 = rig.pose
    rig.spin_for(3.0)
    fin = rig.series('/cmd_vel', t_loss)
    z = first_time(fin, lambda a, b: abs(a) < 1e-3 and abs(b) < 1e-3, t_loss)
    p1 = rig.pose
    travel = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) if p0 and p1 else float('nan')
    yaw = gt_yaw_travel(rig, t_loss, t_loss + 3.0)
    rig.dump(name)
    ok = z is not None and (z - t_loss) <= 1.0 + 0.5
    return ok, (f'final /cmd_vel zero {None if z is None else round(z - t_loss, 3)} s after the '
                f'last {topic} message; ground-truth travel after loss {travel:.3f} m, '
                f'{math.degrees(yaw):.1f} deg')


def s_teleop_loss(rig):
    return _loss(rig, 'cmd_vel_joy', (0.10, 0.0), 'teleop_loss')


def s_dock_loss(rig):
    return _loss(rig, 'cmd_vel_dock', (0.10, 0.0), 'dock_loss')


def s_back_to_start(rig):
    """Reverse to roughly where the translation scenarios started (bookkeeping only)."""
    rig.spin_for(4.0, {'cmd_vel_nav': (-0.10, 0.0)})
    rig.spin_for(2.0, {'cmd_vel_nav': (0.0, 0.0)})
    rig.dump('back_to_start')
    return True, 'repositioned'


SCENARIOS = {
    'priority_handover': s_priority_handover,
    'stop_lock_over_teleop': s_stop_lock_over_teleop,
    'lock_without_zero_source': s_lock_without_zero_source,
    'teleop_loss': s_teleop_loss,
    'back1': s_back_to_start,
    'dock_loss': s_dock_loss,
    'back2': s_back_to_start,
}


def main():
    outdir = sys.argv[1]
    names = sys.argv[2:] or list(SCENARIOS)
    os.makedirs(outdir, exist_ok=True)
    rclpy.init()
    rig = Rig(outdir)
    t_end = time.monotonic() + 10.0
    while not rig.clock_seen and time.monotonic() < t_end:
        rclpy.spin_once(rig, timeout_sec=0.2)
    if not rig.clock_seen:
        print('REFUSED: no /clock, simulator not running')
        return 2
    rc = 0
    for n in names:
        ok, detail = SCENARIOS[n](rig)
        if not n.startswith('back'):
            print(f'RESULT {n}: {"PASS" if ok else "FAIL"}: {detail}', flush=True)
            rc |= 0 if ok else 1
    return rc


if __name__ == '__main__':
    sys.exit(main())
