"""
Gate-crash bound on the final /cmd_vel (platform-sw#90, Alex's 13 Nov review,
item 6): "with the gate process killed while a nonzero command is flowing,
record the time from the kill to the last nonzero /cmd_vel message; it must
be 0.5 s or less."

This is deliberately a real OS process kill, not an in-process
destroy_node() - the scenario under test is safety_stop_gate_node crashing
(the watchdog/arbitration authority dying), which can only be modelled by
actually killing its process. test_stop_contract_full_chain.py's gate is an
in-process SafetyStopGateNode() object for exactly the opposite reason (fast,
controllable #119 contract tests) and cannot stand in here.

Full real chain, every hop a real subprocess: safety_stop_gate_node (killed
mid-test) -> twist_mux -> [this file's own gate, real] -> cmd_vel_gated ->
velocity_smoother -> cmd_vel_smoothed -> collision_monitor -> cmd_vel (the
literal topic Alex's wording names). The gate's own 4 fake action/cancel
servers, dock_trigger status/abort and goal_authority_events heartbeats are
the same shapes as test_stop_contract_full_chain.py uses to satisfy #119's
release conditions (a)-(c), reused here only to get the gate released once
before the crash - this file adds no new release-contract coverage of its
own.

collision_monitor needs a working TF tree (odom->base_link) and continuously
fresh /scan_filtered + /local_costmap/published_footprint data, confirmed by
direct experiment (not assumed): without them it never forwards ANY
velocity, zero or nonzero, from construction onward - not a timeout/stale
condition, a hard requirement for it to pass anything through at all. No
existing test in this package drives collision_monitor from outside in
isolation, so this fake TF/scan/footprint setup is new, built and verified
against the real installed nav2_collision_monitor before being used here.

Needs the real ros2 CLI AND twist_mux, nav2_velocity_smoother,
nav2_collision_monitor, tf2_ros (static_transform_publisher) installed -
skipped everywhere else, same convention as the rest of this test package.
Run for real with:

    colcon build --packages-select openamrobot_nav2
    source install/setup.bash
    python3 -m pytest -v -s \
        src/openamrobot_nav2/test/test_gate_crash_full_chain_cmd_vel_bound.py

New, unreviewed test infrastructure (subprocess-managed, five real nodes) -
if it is flaky in your environment, that is useful information on its own,
not just noise to retry past.
"""
import importlib.util
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import unittest

import pytest

rclpy = pytest.importorskip('rclpy')  # tracking: #90


def _pkg_prefix_available(pkg):
    if shutil.which('ros2') is None:
        return False
    try:
        result = subprocess.run(
            ['ros2', 'pkg', 'prefix', pkg],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10.0)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0


def _chain_available():
    return all(
        _pkg_prefix_available(pkg)
        for pkg in ('twist_mux', 'nav2_velocity_smoother', 'nav2_collision_monitor', 'tf2_ros'))


pytestmark = pytest.mark.skipif(
    not _chain_available(),
    reason='needs twist_mux, nav2_velocity_smoother, nav2_collision_monitor and '
           'tf2_ros installed [#90]')

from action_msgs.msg import GoalStatusArray  # noqa: E402,I100
from action_msgs.srv import CancelGoal  # noqa: E402
from geometry_msgs.msg import Point32, PolygonStamped, Twist  # noqa: E402
from openamrobot_nav2.safety_stop_gate_node import (  # noqa: E402
    _CANCEL_ACTIONS, _LEDGER_PRODUCERS, _real_action_server_name, T_READY)
from rclpy.qos import qos_profile_action_status_default  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

_PKG_ROOT = Path(__file__).resolve().parents[1]
_TWIST_MUX_PARAMS = _PKG_ROOT / 'config' / 'twist_mux.yaml'
_NAV2_PARAMS = _PKG_ROOT / 'config' / 'nav2_params.yaml'

# The bound Alex's review states, verbatim.
_KILL_TO_LAST_NONZERO_BOUND_S = 0.5

# tools/issue35_arbitration/stop_lock_invariants.py is scratch review
# tooling, not a repository package (no __init__.py anywhere above it, not
# on any install path) - loaded directly from its file path, same reason
# this file cannot just `import stop_lock_invariants`.
_REPO_ROOT = Path(__file__).resolve().parents[4]
_STOP_LOCK_INVARIANTS_PATH = (
    _REPO_ROOT / 'tools' / 'issue35_arbitration' / 'stop_lock_invariants.py')
_spec = importlib.util.spec_from_file_location(
    'stop_lock_invariants', _STOP_LOCK_INVARIANTS_PATH)
stop_lock_invariants = importlib.util.module_from_spec(_spec)
if _STOP_LOCK_INVARIANTS_PATH.exists():
    _spec.loader.exec_module(stop_lock_invariants)


def _start_new_session(args):
    return subprocess.Popen(
        args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)


def _stop_group(proc, timeout_s=3.0):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=timeout_s)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            pass


def _lifecycle_transition(node_name, label, timeout_s=30.0, proc=None):
    """Run `ros2 lifecycle set <node> <label>`, retrying while not yet discoverable.

    Same retry shape as test_velocity_smoother_publisher_loss_distance.py's
    _lifecycle_transition - a hanging CLI call is "not ready yet", not a
    failure, while the node's lifecycle service is undiscovered.
    """
    deadline = time.monotonic() + timeout_s
    last_detail = '(never ran)'
    while time.monotonic() < deadline:
        if proc is not None and proc.poll() is not None:
            break
        try:
            result = subprocess.run(
                ['ros2', 'lifecycle', 'set', f'/{node_name}', label],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5.0)
        except subprocess.TimeoutExpired:
            last_detail = 'the CLI call itself blocked for 5s (service not discovered yet)'
            continue
        if result.returncode == 0:
            return
        last_detail = (result.stderr or b'').decode(errors='replace').strip() or '(no stderr)'
        time.sleep(0.3)
    state = '(never started)' if proc is None else (
        f'still running (pid {proc.pid})' if proc.poll() is None
        else f'EXITED with code {proc.returncode}')
    raise AssertionError(
        f'ros2 lifecycle set /{node_name} {label} never succeeded within '
        f'{timeout_s}s - last attempt: {last_detail}; process state: {state}')


class TestGateCrashCmdVelBound(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.probe = rclpy.create_node('gate_crash_probe')

        # --- TF: collision_monitor refuses to forward ANY velocity without a
        # resolvable base_frame_id<->odom_frame_id transform (confirmed by
        # direct experiment - see module docstring). Static is enough: this
        # test never moves the robot, it only needs the lookup to succeed.
        self._tf_proc = _start_new_session(
            ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
             '0', '0', '0', '0', '0', '0', 'odom', 'base_link'])

        # --- twist_mux: real, same invocation as test_stop_contract_full_chain.py.
        self._mux_proc = _start_new_session(
            ['ros2', 'run', 'twist_mux', 'twist_mux',
             '--ros-args', '--params-file', str(_TWIST_MUX_PARAMS),
             '-r', 'cmd_vel_out:=cmd_vel_mux'])

        # --- the gate itself: a REAL subprocess, so it can be really killed.
        self._gate_proc = _start_new_session(
            ['ros2', 'run', 'openamrobot_nav2', 'safety_stop_gate_node'])
        self._gate_started_at = time.monotonic()

        # --- velocity_smoother: real, lifecycle-managed, same shape as
        # test_velocity_smoother_publisher_loss_distance.py.
        self._smoother_proc = _start_new_session(
            ['ros2', 'run', 'nav2_velocity_smoother', 'velocity_smoother',
             '--ros-args', '--params-file', str(_NAV2_PARAMS),
             '-r', 'cmd_vel:=cmd_vel_gated',
             '-r', 'odom:=gate_crash_test_unused_odom'])

        # --- collision_monitor: real, lifecycle-managed. No remaps - the
        # shipped config already names cmd_vel_in_topic=cmd_vel_smoothed,
        # cmd_vel_out_topic=cmd_vel (the literal topic under test).
        self._cm_proc = _start_new_session(
            ['ros2', 'run', 'nav2_collision_monitor', 'collision_monitor',
             '--ros-args', '--params-file', str(_NAV2_PARAMS)])

        # --- probe-side publishers/subscriptions.
        self._nav_pub = self.probe.create_publisher(Twist, 'cmd_vel_nav', 10)
        self._scan_pub = self.probe.create_publisher(LaserScan, '/scan_filtered', 10)
        self._fp_pub = self.probe.create_publisher(
            PolygonStamped, '/local_costmap/published_footprint', 10)
        self._final_cmd_vel = []  # (monotonic_receipt_time, linear_x, angular_z)
        self.probe.create_subscription(
            Twist, 'cmd_vel',
            lambda m: self._final_cmd_vel.append((time.monotonic(), m.linear.x, m.angular.z)),
            10)

        # --- fake action/dock/ledger servers (same shapes as
        # test_stop_contract_full_chain.py), only to get the gate released once.
        self._goal_status_pubs = {}
        self._cancel_services = {}
        for action in _CANCEL_ACTIONS:
            self._goal_status_pubs[action] = self.probe.create_publisher(
                GoalStatusArray, f'{_real_action_server_name(action)}/_action/status',
                qos_profile_action_status_default)
            self._cancel_services[action] = self.probe.create_service(
                CancelGoal, f'{_real_action_server_name(action)}/_action/cancel_goal',
                self._accept_cancel)
        self._dock_status_pub = self.probe.create_publisher(String, 'dock_trigger_status', 10)
        self.probe.create_service(Trigger, '/dock_trigger/abort', self._accept_abort)
        self._goal_authority_pub = self.probe.create_publisher(
            String, 'goal_authority_events', 10)

    def tearDown(self):
        _stop_group(self._gate_proc)
        _stop_group(self._mux_proc)
        _stop_group(self._smoother_proc)
        _stop_group(self._cm_proc)
        _stop_group(self._tf_proc)
        self.probe.destroy_node()

    @staticmethod
    def _accept_cancel(request, response):
        del request
        response.return_code = CancelGoal.Response.ERROR_NONE
        return response

    @staticmethod
    def _accept_abort(request, response):
        del request
        response.success = True
        response.message = ''
        return response

    def _spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.probe, timeout_sec=0.01)

    def _publish_fakes_once(self):
        """One round of every status this test needs kept fresh/idle."""
        for action in _CANCEL_ACTIONS:
            self._goal_status_pubs[action].publish(GoalStatusArray())  # empty: no active goal
        self._dock_status_pub.publish(String(data='idle'))
        for producer in _LEDGER_PRODUCERS:
            self._goal_authority_pub.publish(String(data=f'{producer}|heartbeat|gate-crash-test|0'))

        now = self.probe.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = 'base_link'
        scan.angle_min = -3.14
        scan.angle_max = 3.14
        scan.angle_increment = 0.1
        n = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = [10.0] * n  # nothing nearby - this test is about the
        # gate crashing, not about collision_monitor's own obstacle logic.
        scan.range_min = 0.1
        scan.range_max = 20.0
        self._scan_pub.publish(scan)

        fp = PolygonStamped()
        fp.header.stamp = now
        fp.header.frame_id = 'odom'
        fp.polygon.points = [
            Point32(x=0.3, y=0.2, z=0.0), Point32(x=0.3, y=-0.2, z=0.0),
            Point32(x=-0.3, y=-0.2, z=0.0), Point32(x=-0.3, y=0.2, z=0.0)]
        self._fp_pub.publish(fp)

    def _spin_keeping_fakes_fresh(self, seconds, nav_cmd=None):
        end = time.monotonic() + seconds
        last_fake = 0.0
        while time.monotonic() < end:
            if time.monotonic() - last_fake > 0.05:
                self._publish_fakes_once()
                last_fake = time.monotonic()
            if nav_cmd is not None:
                self._nav_pub.publish(nav_cmd)
            rclpy.spin_once(self.probe, timeout_sec=0.02)

    def _release(self):
        self._publish_fakes_once()
        self._spin_for(0.2)
        client = self.probe.create_client(Trigger, '/safety/stop_lock_release')
        self.assertTrue(
            client.wait_for_service(timeout_sec=10.0), 'the gate never came up')
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < end:
            self._spin_for(0.02)
        self.assertTrue(future.done(), 'release call timed out')
        client.destroy()
        return future.result()

    def _bring_up_and_release(self):
        # T_READY: the gate refuses release until it has been up this long.
        elapsed = time.monotonic() - self._gate_started_at
        if elapsed < T_READY + 0.2:
            self._spin_keeping_fakes_fresh(T_READY + 0.2 - elapsed)

        self.assertIsNone(self._gate_proc.poll(), 'the gate exited before release')
        self.assertIsNone(self._smoother_proc.poll(), 'velocity_smoother exited before use')
        self.assertIsNone(self._cm_proc.poll(), 'collision_monitor exited before use')

        _lifecycle_transition('velocity_smoother', 'configure', proc=self._smoother_proc)
        _lifecycle_transition('velocity_smoother', 'activate', proc=self._smoother_proc)
        _lifecycle_transition('collision_monitor', 'configure', proc=self._cm_proc)
        _lifecycle_transition('collision_monitor', 'activate', proc=self._cm_proc)

        response = self._release()
        self.assertTrue(response.success, f'release was refused: {response.message}')

    def _assert_kill_to_last_nonzero_within_bound(self, nav_cmd, field_index, field_name):
        """Drive nav_cmd through the full chain, kill the gate, measure the bound.

        field_index is 1 for linear.x, 2 for angular.z (the tuple shape
        stashed by the 'cmd_vel' subscription in setUp) - shared by both
        the linear and rotation cases below, which differ only in which
        command/field they drive and check.
        """
        # Drive a steady nonzero command through the full chain and confirm
        # it actually reaches the final /cmd_vel before crashing anything -
        # proves the chain is really wired end to end, not just "it didn't
        # error", before the kill measurement means anything.
        self._final_cmd_vel.clear()
        self._spin_keeping_fakes_fresh(2.0, nav_cmd=nav_cmd)
        self.assertTrue(
            self._final_cmd_vel,
            'no /cmd_vel at all reached the probe before the crash - the '
            'chain is not wired end to end (check TF/scan/footprint fakes '
            'and lifecycle activation above)')
        self.assertGreater(
            abs(self._final_cmd_vel[-1][field_index]), 0.01,
            f'/cmd_vel.{field_name} was not nonzero before the crash '
            f'(last={self._final_cmd_vel[-1][field_index]}) - cannot measure a '
            'kill-to-last-nonzero bound from an already-zero chain')

        # The actual event under test: kill -9 the gate process while the
        # nonzero command keeps flowing from upstream (cmd_vel_nav keeps
        # publishing - only the gate dies, nothing upstream of it does).
        self._final_cmd_vel.clear()
        kill_time = time.monotonic()
        os.killpg(self._gate_proc.pid, signal.SIGKILL)
        self._gate_proc.wait(timeout=5.0)

        # Generous settle window: velocity_smoother's own velocity_timeout
        # (<=0.3s, test_velocity_smoother_publisher_loss_distance.py) plus a
        # decel ramp from the test's own commanded value, with slack.
        self._spin_keeping_fakes_fresh(2.0, nav_cmd=nav_cmd)

        self.assertTrue(
            self._final_cmd_vel,
            'no /cmd_vel at all was received after the gate was killed - cannot '
            'find a "last nonzero" to measure (if the chain was already '
            'zero/stopped, that is a DIFFERENT failure than a slow stop and '
            'should not read as passing this bound)')

        last_nonzero_t = None
        for sample in self._final_cmd_vel:
            if abs(sample[field_index]) > 1e-6:
                last_nonzero_t = sample[0]
        self.assertIsNotNone(
            last_nonzero_t,
            f'every /cmd_vel.{field_name} sample after the gate was killed was already '
            'zero - cannot measure a kill-to-last-nonzero bound (the chain may have '
            'already been at zero, which is a different condition than '
            '"stopped within the bound")')

        bound_s = last_nonzero_t - kill_time
        self.assertLessEqual(
            bound_s, _KILL_TO_LAST_NONZERO_BOUND_S,
            f'the last nonzero /cmd_vel.{field_name} message arrived {bound_s:.3f}s after '
            f'the gate process was killed - exceeds the {_KILL_TO_LAST_NONZERO_BOUND_S}s '
            "bound (platform-sw#90, Alex's 13 Nov review, item 6)")

    def test_last_nonzero_cmd_vel_after_a_gate_crash_is_within_half_a_second(self):
        self._bring_up_and_release()
        nav_cmd = Twist()
        nav_cmd.linear.x = 0.2
        self._assert_kill_to_last_nonzero_within_bound(nav_cmd, 1, 'linear.x')

    def test_last_nonzero_angular_z_after_a_gate_crash_is_within_half_a_second(self):
        # panthera-momagdii's #90 review on d29dfc0: the linear case above
        # only records linear.x, and 0.20 m/s is the shipped MAXIMUM linear
        # velocity (velocity_smoother.max_velocity[0], nav2_params.yaml) -
        # not "low" relative to what the chain can actually command, so
        # that case is already the tightest linear one available. Rotation
        # is the genuinely tighter case: from the shipped values
        # (max_velocity[2]=0.5 rad/s, max_decel[2]=-3.0 rad/s², 0.3s
        # velocity_timeout), the decel-ramp time alone
        # (0.5 / 3.0 ≈ 0.167s) stacks on top of the timeout for a
        # calculated ~0.467s, closer to the 0.5s bound than the linear
        # case's ~0.3s. Pure rotation (linear.x left at 0.0) isolates it.
        self._bring_up_and_release()
        nav_cmd = Twist()
        nav_cmd.angular.z = 0.5
        self._assert_kill_to_last_nonzero_within_bound(nav_cmd, 2, 'angular.z')

    def test_stop_lock_invariants_cmd_vel_check_passes_against_the_real_chain(self):
        # panthera-momagdii's #90 review on d29dfc0: the /cmd_vel/
        # collision_monitor singleton check added to stop_lock_invariants.py
        # (items 3/4 push) was never actually RUN with the check turned on -
        # the hosted invariants step always passes --no-check-cmd-vel (it
        # only starts the gate standalone, no collision_monitor), and no
        # test called check(check_cmd_vel=True) either. This file is the
        # only place in the suite that brings up the real collision_monitor
        # at all, so it is where that gap gets closed.
        self._bring_up_and_release()
        self._spin_for(1.0)  # let discovery catch up with collision_monitor's publisher
        problems = stop_lock_invariants.check(
            self.probe, 'safety_stop_gate_node', check_cmd_vel=True)
        self.assertEqual(
            problems, [],
            'stop_lock_invariants.check() with check_cmd_vel=True reported a problem '
            f'against the real, correctly-wired chain: {problems}')

    def test_stop_lock_invariants_cmd_vel_check_fails_on_a_second_publisher(self):
        # Same setup as above, but with a second, rogue /cmd_vel publisher -
        # the exact graph violation this check exists to catch (panthera's
        # review: "add a case with a second /cmd_vel publisher that must
        # fail").
        self._bring_up_and_release()
        rogue_node = rclpy.create_node('gate_crash_rogue_cmd_vel_publisher')
        rogue_pub = rogue_node.create_publisher(Twist, 'cmd_vel', 10)
        try:
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                rogue_pub.publish(Twist())
                rclpy.spin_once(self.probe, timeout_sec=0.02)
                rclpy.spin_once(rogue_node, timeout_sec=0.02)
            problems = stop_lock_invariants.check(
                self.probe, 'safety_stop_gate_node', check_cmd_vel=True)
            cmd_vel_problems = [p for p in problems if p.startswith('/cmd_vel:')]
            self.assertTrue(
                cmd_vel_problems,
                'stop_lock_invariants.check() did not report the rogue second /cmd_vel '
                f'publisher - all problems reported: {problems}')
        finally:
            rogue_node.destroy_node()


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v', '-s']))
