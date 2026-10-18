#!/usr/bin/env python3
"""Issue #35: check option (a) as proposed against twist_mux 4.5.0 (Jazzy).

Runs ONE standalone twist_mux in the namespace /a_check with the proposed table
(nav 10 / 0.5 s, dock 20 / 0.2 s) plus the thread's teleop + lock rows at 255,
and only talks to topics under /a_check, so nothing reaches a robot or the sim.

Checks:
  A1 output message type with the package default use_stamped (expect TwistStamped)
  A2 all inputs time out: does twist_mux publish a zero? (expect: nothing)
  A3 lock at 255 vs teleop at 255: is teleop masked? (expect: not masked)
  A4 lock at 255 vs teleop at 100: is teleop masked? (expect: masked)
usage: option_a_checks.py OUTDIR
"""
import os
import subprocess
import sys
import tempfile
import time

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

CFG = """
/a_check/twist_mux:
  ros__parameters:
{stamped}
    topics:
      navigation: {{topic: cmd_vel_nav_out, timeout: 0.5, priority: 10}}
      docking:    {{topic: cmd_vel_dock, timeout: 0.2, priority: 20}}
      teleop:     {{topic: cmd_vel_teleop, timeout: 0.25, priority: {teleop}}}
    locks:
      estop:      {{topic: estop_lock, timeout: 0.0, priority: 255}}
"""


def run_mux(cfg_text):
    f = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    f.write(cfg_text)
    f.close()
    return subprocess.Popen(
        ['ros2', 'run', 'twist_mux', 'twist_mux', '--ros-args', '-r', '__ns:=/a_check',
         '--params-file', f.name], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
        start_new_session=True)


class Probe(Node):
    def __init__(self, stamped):
        super().__init__('a_check_probe', namespace='/a_check')
        self.stamped = stamped
        mtype = TwistStamped if stamped else Twist
        self.pub = {t: self.create_publisher(mtype, t, 10)
                    for t in ('cmd_vel_nav_out', 'cmd_vel_dock', 'cmd_vel_teleop')}
        self.lock = self.create_publisher(Bool, 'estop_lock', 10)
        self.out = []
        self.create_subscription(mtype, 'cmd_vel_out',
                                 lambda m: self.out.append((time.monotonic(), m)), 50)

    def send(self, topic, wz):
        if self.stamped:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.twist.angular.z = wz
        else:
            m = Twist()
            m.angular.z = wz
        self.pub[topic].publish(m)

    def run(self, secs, srcs, lock=None):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            for t, wz in srcs.items():
                self.send(t, wz)
            if lock is not None:
                self.lock.publish(Bool(data=lock))
            t_next = time.monotonic() + 0.05
            while time.monotonic() < t_next:
                rclpy.spin_once(self, timeout_sec=0.005)


def wz(m):
    return m.twist.angular.z if hasattr(m, 'twist') else m.angular.z


def out_type():
    r = subprocess.run(['ros2', 'topic', 'info', '/a_check/cmd_vel_out'],
                       capture_output=True, text=True)
    return [ln.split(':', 1)[1].strip() for ln in r.stdout.splitlines() if ln.startswith('Type')]


def main():
    outdir = sys.argv[1]
    os.makedirs(outdir, exist_ok=True)
    rclpy.init()
    lines = []

    # A1 + A2 with the package default (no use_stamped line) and teleop at 255.
    mux = run_mux(CFG.format(stamped='', teleop=255))
    probe = Probe(stamped=True)
    time.sleep(3.0)
    probe.run(1.0, {'cmd_vel_dock': 0.2})
    types = out_type()
    stamped = types == ['geometry_msgs/msg/TwistStamped']
    verdict = 'TwistStamped, incompatible with the Twist base and Gazebo bridge'
    lines.append(f'A1 default use_stamped: /a_check/cmd_vel_out type {types} '
                 f'({verdict if stamped else "check"})')
    n_before = len(probe.out)
    t_stop = time.monotonic()
    probe.run(1.5, {})
    after = [m for t, m in probe.out[n_before:] if t > t_stop + 0.01]
    zeros = sum(1 for m in after if wz(m) == 0.0)
    lines.append(f'A2 all inputs silent for 1.5 s after dock 0.2 rad/s: twist_mux published '
                 f'{len(after)} message(s), {zeros} of them zero')
    # A3 lock at 255, teleop at 255.
    probe.out.clear()
    probe.run(1.0, {'cmd_vel_teleop': 0.3}, lock=True)
    fwd = [m for _, m in probe.out if abs(wz(m) - 0.3) < 1e-6]
    lines.append(f'A3 lock 255 engaged, teleop 255 publishing: {len(fwd)} teleop message(s) '
                 f'forwarded ({"NOT masked" if fwd else "masked"})')
    probe.run(0.3, {}, lock=False)
    probe.destroy_node()
    os.killpg(mux.pid, 2)
    mux.wait(timeout=10)

    # A4 lock at 255, teleop at 100 (use_stamped false).
    mux = run_mux(CFG.format(stamped='    use_stamped: false', teleop=100))
    probe = Probe(stamped=False)
    time.sleep(3.0)
    probe.run(1.0, {'cmd_vel_teleop': 0.3}, lock=False)
    n0 = len(probe.out)
    probe.run(1.0, {'cmd_vel_teleop': 0.3}, lock=True)
    fwd = [m for _, m in probe.out[n0 + 2:] if abs(wz(m) - 0.3) < 1e-6]
    lines.append(f'A4 lock 255 engaged, teleop 100 publishing: {len(fwd)} teleop message(s) '
                 f'forwarded after the lock ({"masked" if not fwd else "NOT masked"}); '
                 f'output type {out_type()}')
    probe.destroy_node()
    os.killpg(mux.pid, 2)
    mux.wait(timeout=10)

    with open(os.path.join(outdir, 'option_a_checks.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
