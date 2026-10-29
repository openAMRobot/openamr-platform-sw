# Copyright 2026 OpenAMRobot contributors
#
# Licensed under the MIT License (see LICENSE).

"""
ROS graph helpers for the simulation launch tests (PSW-03, PSW-15).

Plain rclpy calls with bounded waits: lifecycle state of a node, the node list a
lifecycle manager manages, and the publishers of a topic. DDS discovery of a full
simulation graph can take tens of seconds, so every query polls until its deadline.
"""

import os
import time

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
import rclpy

ACTIVE = State.PRIMARY_STATE_ACTIVE
MANAGERS = ['lifecycle_manager_localization', 'lifecycle_manager_navigation']


def call(node, srv_type, name, request, timeout):
    """Call a service once it appears; None when it does not answer before the timeout."""
    client = node.create_client(srv_type, name)
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        return future.result()
    finally:
        node.destroy_client(client)


def lifecycle_state(node, name, timeout=5.0):
    """Primary state id of a lifecycle node, or None when it does not answer."""
    res = call(node, GetState, '/%s/get_state' % name, GetState.Request(), timeout)
    return None if res is None else res.current_state.id


def managed_nodes(node, manager, timeout=10.0):
    """Return the node_names parameter of a Nav2 lifecycle manager, or None."""
    req = GetParameters.Request(names=['node_names'])
    res = call(node, GetParameters, '/%s/get_parameters' % manager, req, timeout)
    if res is None or not res.values:
        return None
    return list(res.values[0].string_array_value)


def wait_all_active(node, deadline_s, log=print):
    """
    Wait until every node managed by the two Nav2 lifecycle managers is active.

    Returns (elapsed seconds, {node: state id}); elapsed is None when the deadline passed.
    """
    start = time.monotonic()
    expected, states = {}, {}
    while time.monotonic() - start < deadline_s:
        for manager in MANAGERS:
            if manager not in expected:
                names = managed_nodes(node, manager)
                if names:
                    expected[manager] = names
                    log('%s manages %s' % (manager, ', '.join(names)))
        names = [n for m in MANAGERS for n in expected.get(m, [])]
        if len(expected) == len(MANAGERS):
            states = {n: lifecycle_state(node, n) for n in names}
            if all(s == ACTIVE for s in states.values()):
                return time.monotonic() - start, states
        time.sleep(2.0)
    return None, states


def publishers(node, topic, settle_s=15.0):
    """
    Return the sorted fully qualified names of the publishers of a topic.

    Polls for settle_s seconds and keeps the largest set seen, so a publisher that
    is discovered late is not missed.
    """
    seen = set()
    end = time.monotonic() + settle_s
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.5)
        for info in node.get_publishers_info_by_topic(topic):
            ns = info.node_namespace.rstrip('/')
            seen.add('%s/%s' % (ns, info.node_name))
    return sorted(seen)


def domain_note():
    return 'ROS_DOMAIN_ID=%s' % os.environ.get('ROS_DOMAIN_ID', '0')
