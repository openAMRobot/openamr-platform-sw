#!/usr/bin/env python3
"""
Graph-invariant check for the software_stop lock/input pair (issue #35).

Mohamed Sayed's review: the mux's lock topic accepts a direct false from
ANY publisher, and cmd_vel_stop's 255 priority means a nonzero Twist on it
would be forwarded unchanged - neither is something safety_stop_gate_node.py
can enforce from inside itself (ROS topics have no publisher ACL). What it
CAN do is be the only node that ever publishes on any of these topics, which
is an invariant checkable from outside, the same way Nik's own Gate A read
checks /cmd_vel: "ros2 topic info /cmd_vel -v shows exactly 1 publisher,
collision_monitor." This script is that same check applied to the topics
safety_stop_gate_node.py is supposed to solely own.

cmd_vel_gated (platform-sw#90, Alex's review) is the actual downstream
enforcement point now - velocity_smoother listens there, not on cmd_vel_mux
directly - so a second publisher on cmd_vel_gated would defeat the post-mux
lock check the same way a second publisher on cmd_vel_stop would defeat the
mux-level input, which is exactly why it is checked here too, not just
cmd_vel_stop.

Also checks /safety/stop_lock_release and /safety/stop_lock_engage each have
exactly one server (platform-sw#90 review, Alex, minor point): a second node
offering either service would let something other than safety_stop_gate_node
answer a release/engage call, which the topic-publisher check above cannot
catch (services, not topics). This does not by itself say who is ALLOWED to
CALL release - that is a UI-contract question (openamrobot-ui#50), not a
graph-shape one this tool can check.

Also checks /cmd_vel has exactly one publisher, collision_monitor
(platform-sw#90 review, Alex, 13 Nov): this is the final topic to the base,
one step past cmd_vel_gated/velocity_smoother. Alex withdrew his 11 Nov
wording about clamping a direct /cmd_vel publish here - nothing sits after
collision_monitor to catch a second publisher - so this is checked as its
own singleton the same way the gate's own topics are, not folded into
safety_stop_gate_node's expect_node (collision_monitor is a different node).
One publisher is necessary, not sufficient - it says nothing about the
smoother/collision-monitor's own output being correct, only that nothing
else can also drive the base.

Run against a live graph (sim or bench), after bringing up navigation_launch
(which brings up safety_stop_gate_node). Exits nonzero and prints which
topic/extra publisher broke the invariant, so it's usable as a bench gate,
not just a human-read report.

--no-check-cmd-vel skips the /cmd_vel/collision_monitor check: this script's
CI invocation (nav-status-tests.yml) starts safety_stop_gate_node standalone,
not the full navigation_launch stack, so collision_monitor never runs there
and /cmd_vel would never get a publisher - that is a reduced-scope test
invocation, not a graph violation, and should not be confused with one. Full
navigation_launch runs (sim/bench) should leave this check on (the default).

Scratch review tool for issue #35 (not repository code) - same status as
the rest of tools/issue35_arbitration/.

usage: stop_lock_invariants.py [--expect-node safety_stop_gate_node] [--no-check-cmd-vel]
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node

TOPICS = ['/safety/stop_lock', 'cmd_vel_stop', 'cmd_vel_gated']
SERVICES = ['/safety/stop_lock_release', '/safety/stop_lock_engage']
# /cmd_vel is owned by collision_monitor, not the gate - checked as its own
# singleton (topic -> expected sole publisher node), separate from TOPICS
# above which all expect --expect-node (safety_stop_gate_node by default).
OTHER_SINGLETON_TOPICS = {'/cmd_vel': 'collision_monitor'}


def check(node: Node, expect_node: str, check_cmd_vel: bool = True):
    problems = []
    for topic in TOPICS:
        infos = node.get_publishers_info_by_topic(topic)
        names = sorted(i.node_name for i in infos)
        if names != [expect_node]:
            problems.append(f'{topic}: publishers = {names!r}, expected [{expect_node!r}]')
    if check_cmd_vel:
        for topic, owner in OTHER_SINGLETON_TOPICS.items():
            infos = node.get_publishers_info_by_topic(topic)
            names = sorted(i.node_name for i in infos)
            if names != [owner]:
                problems.append(f'{topic}: publishers = {names!r}, expected [{owner!r}]')
    for service in SERVICES:
        count = node.count_services(service)
        if count != 1:
            problems.append(f'{service}: {count} server(s), expected exactly 1')
    return problems


def missing_for_readiness(node, expect_node, check_cmd_vel: bool = True):
    """
    Return what is still absent before the invariant is meaningful to check.

    Empty means the expected node is in the graph and every watched topic
    and service has at least one endpoint - i.e. the gate has finished
    coming up. This is deliberately only an "exists at all" check, never
    an "exactly one" check: the singleton invariant itself stays entirely
    in check(), so waiting for readiness can never mask a duplicate
    publisher (platform-sw#90, Mohamed's review - the earlier version
    checked a fixed 2 s after backgrounding the gate, which could not
    tell a slow start from a gate that had exited).
    """
    missing = []
    if expect_node not in node.get_node_names():
        missing.append(f'node {expect_node!r} not in the graph')
    for topic in TOPICS:
        if not node.get_publishers_info_by_topic(topic):
            missing.append(f'{topic}: no publisher yet')
    if check_cmd_vel:
        for topic in OTHER_SINGLETON_TOPICS:
            if not node.get_publishers_info_by_topic(topic):
                missing.append(f'{topic}: no publisher yet')
    for service in SERVICES:
        if node.count_services(service) < 1:
            missing.append(f'{service}: no server yet')
    return missing


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--expect-node', default='safety_stop_gate_node')
    parser.add_argument(
        '--settle-s', type=float, default=2.0,
        help='time to let discovery catch up AFTER readiness, before checking')
    parser.add_argument(
        '--await-ready-s', type=float, default=30.0,
        help='bounded wait for the gate to appear at all before checking')
    parser.add_argument(
        '--no-check-cmd-vel', dest='check_cmd_vel', action='store_false',
        help='skip the /cmd_vel (collision_monitor) singleton check - use '
             'this for a reduced-scope invocation that never starts '
             'collision_monitor, e.g. the gate run standalone in '
             'nav-status-tests.yml, not a full navigation_launch stack')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('stop_lock_invariants_check')
    try:
        # Bounded wait for the gate to actually be up. Timing out here is
        # its own distinct failure ("never became ready"), not an invariant
        # violation - a gate that crashed on startup and a gate that is
        # merely slow both used to read as "zero publishers", which proves
        # nothing about the graph shape either way.
        ready_end = time.monotonic() + args.await_ready_s
        missing = missing_for_readiness(node, args.expect_node, args.check_cmd_vel)
        while missing and time.monotonic() < ready_end:
            rclpy.spin_once(node, timeout_sec=0.1)
            missing = missing_for_readiness(node, args.expect_node, args.check_cmd_vel)
        if missing:
            print(
                f'STOP-LOCK CHECK NOT READY after {args.await_ready_s}s - the gate '
                f'never came up, so the invariant was never evaluated:')
            for m in missing:
                print(f'  - {m}')
            sys.exit(2)

        # Graph (publisher/subscriber) info is populated by discovery as it
        # arrives, not synchronously on node creation - give it time, and
        # spin so any pending discovery callbacks are actually processed.
        # This settle still runs AFTER readiness, so a second publisher that
        # is slower to be discovered is still caught by check() below.
        end = time.monotonic() + args.settle_s
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        problems = check(node, args.expect_node, args.check_cmd_vel)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if problems:
        print('STOP-LOCK INVARIANT VIOLATED:')
        for p in problems:
            print(f'  - {p}')
        sys.exit(1)
    cmd_vel_note = (
        "; /cmd_vel has exactly one publisher ('collision_monitor')"
        if args.check_cmd_vel else '; /cmd_vel check skipped (--no-check-cmd-vel)')
    print(f'OK: {", ".join(TOPICS)} each have exactly one publisher '
          f'({args.expect_node!r}){cmd_vel_note}; {", ".join(SERVICES)} each '
          f'have exactly one server')


if __name__ == '__main__':
    main()
