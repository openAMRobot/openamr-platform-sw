#!/usr/bin/env python3
"""
Graph-invariant check for the software_stop lock/input pair (issue #35).

Mohamed Sayed's review: the mux's lock topic accepts a direct false from
ANY publisher, and cmd_vel_stop's 255 priority means a nonzero Twist on it
would be forwarded unchanged - neither is something safety_stop_gate_node.py
can enforce from inside itself (ROS topics have no publisher ACL). What it
CAN do is be the only node that ever publishes on either topic, which is an
invariant checkable from outside, the same way Nik's own Gate A read checks
/cmd_vel: "ros2 topic info /cmd_vel -v shows exactly 1 publisher,
collision_monitor." This script is that same check applied to the two
topics safety_stop_gate_node.py is supposed to solely own.

Run against a live graph (sim or bench), after bringing up navigation_launch
(which brings up safety_stop_gate_node). Exits nonzero and prints which
topic/extra publisher broke the invariant, so it's usable as a bench gate,
not just a human-read report.

Scratch review tool for issue #35 (not repository code) - same status as
the rest of tools/issue35_arbitration/.

usage: stop_lock_invariants.py [--expect-node safety_stop_gate_node]
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node

TOPICS = ['/safety/stop_lock', 'cmd_vel_stop']


def check(node: Node, expect_node: str):
    problems = []
    for topic in TOPICS:
        infos = node.get_publishers_info_by_topic(topic)
        names = sorted(i.node_name for i in infos)
        if names != [expect_node]:
            problems.append(f'{topic}: publishers = {names!r}, expected [{expect_node!r}]')
    return problems


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--expect-node', default='safety_stop_gate_node')
    parser.add_argument(
        '--settle-s', type=float, default=2.0,
        help='time to let discovery catch up before checking')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('stop_lock_invariants_check')
    try:
        # Graph (publisher/subscriber) info is populated by discovery as it
        # arrives, not synchronously on node creation - give it time, and
        # spin so any pending discovery callbacks are actually processed.
        end = time.monotonic() + args.settle_s
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        problems = check(node, args.expect_node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if problems:
        print('STOP-LOCK INVARIANT VIOLATED:')
        for p in problems:
            print(f'  - {p}')
        sys.exit(1)
    print(f'OK: {", ".join(TOPICS)} each have exactly one publisher '
          f'({args.expect_node!r})')


if __name__ == '__main__':
    main()
