"""
Node-level tests with fake publishers and a fake clock.

openamr_base_msgs does not exist yet (openamrobot-interfaces #30), so the status callbacks
are fed with plain objects carrying the I8 fields. Everything on the ROS side is real:
the lock publisher, /diagnostics, the resume service and its clients.
"""
import time
from types import SimpleNamespace as NS

from diagnostic_msgs.msg import DiagnosticArray
from openamrobot_drivers.base_status_monitor import BaseStatusMonitor
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

REGISTRY_TEXT = 'Fault registry text for the test.'


class FakeClock:

    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def status(seq, ctl='READY', safety='RUN', faults=(), recovery=False, boot=(1, 2, 3, 4)):
    return NS(contract_version=1, boot_id=list(boot), sequence=seq, controller_state=ctl,
              safety_state=safety, recovery_required=recovery, faults=list(faults))


def aux(seq, boot=(1, 2, 3, 4)):
    return NS(contract_version=1, boot_id=list(boot), sequence=seq)


@pytest.fixture
def rig(tmp_path):
    registry = tmp_path / 'registry.yaml'
    registry.write_text('faults:\n  18001:\n    name: CMD_TIMEOUT\n    text: "%s"\n'
                        % REGISTRY_TEXT)
    rclpy.init()
    clock = FakeClock()
    node = BaseStatusMonitor(
        clock_fn=clock,
        parameter_overrides=[
            Parameter('fault_registry_file', value=str(registry)),
            Parameter('status_type', value='no_such_pkg/msg/Nothing')])
    probe = rclpy.create_node('base_gate_probe')
    locks, diags, texts = [], [], []
    probe.create_subscription(Bool, '/base/command_lock', lambda m: locks.append(m.data), 10)
    probe.create_subscription(DiagnosticArray, '/diagnostics', diags.append, 10)
    probe.create_subscription(String, '/base/monitor_text', lambda m: texts.append(m.data), 10)
    client = probe.create_client(Trigger, '/base/resume')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(probe)

    class Rig:
        pass

    r = Rig()
    r.node, r.clock, r.locks, r.diags, r.texts = node, clock, locks, diags, texts
    r.client, r.executor = client, executor

    def spin(seconds=0.25):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)

    def feed_ok(seconds=1.0, **kw):
        for i in range(int(seconds * 10)):
            clock.t += 0.1
            r.seq = getattr(r, 'seq', 0) + 1
            node._on_status_msg(status(r.seq, **kw))
            if r.seq % 2 == 0:
                node._on_aux_msg(aux(r.seq))

    def resume():
        assert client.wait_for_service(timeout_sec=2.0)
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
        return future.result()

    r.spin, r.feed_ok, r.resume = spin, feed_ok, resume
    yield r
    executor.shutdown()
    node.destroy_node()
    probe.destroy_node()
    rclpy.try_shutdown()


def last_lock(rig):
    rig.locks.clear()
    rig.spin(0.2)
    assert rig.locks, 'no lock message published'
    return rig.locks[-1]


def test_missing_message_type_fails_closed_and_says_so(rig):
    # status_type is overridden to a package that does not exist
    rig.spin(0.3)
    assert last_lock(rig) is True
    assert any('not installed' in t for t in rig.texts)
    result = rig.resume()
    assert not result.success


def test_fresh_run_stays_locked_until_resume(rig):
    rig.node.gate.set_type_unavailable(False)     # the fake publishers stand in for the type
    rig.feed_ok()
    assert last_lock(rig) is True, 'nothing resumes by itself'
    result = rig.resume()
    assert result.success
    assert last_lock(rig) is False


def test_stale_status_locks(rig):
    rig.node.gate.set_type_unavailable(False)
    rig.feed_ok()
    assert rig.resume().success
    assert last_lock(rig) is False
    rig.clock.t += 0.51
    assert last_lock(rig) is True
    rig.spin(0.7)
    levels = [s.level for d in rig.diags for s in d.status]
    assert levels and ord(levels[-1]) == 2          # ERROR
    text = ' '.join(v.value for d in rig.diags[-1:] for s in d.status for v in s.values)
    assert 'STATUS_STALE' in text


def test_locked_state_blocks_resume_with_operator_text(rig):
    rig.node.gate.set_type_unavailable(False)
    rig.feed_ok(safety='LOCKED')
    assert last_lock(rig) is True
    result = rig.resume()
    assert not result.success
    assert 'LOCKED' in result.message
    assert last_lock(rig) is True


def test_fault_locks_and_publishes_registry_text(rig):
    rig.node.gate.set_type_unavailable(False)
    fault = NS(code=18001, severity='ERROR', motion_inhibit=True, active=True)
    rig.feed_ok(ctl='FAULTED', safety='LOCKED', faults=[fault])
    assert last_lock(rig) is True
    rig.spin(0.7)
    text = ' '.join(v.value for s in rig.diags[-1].status for v in s.values)
    assert REGISTRY_TEXT in text and '18001' in text
    assert not rig.resume().success


def test_run_after_fault_needs_explicit_resume(rig):
    rig.node.gate.set_type_unavailable(False)
    fault = NS(code=18001, severity='ERROR', motion_inhibit=True, active=True)
    rig.feed_ok(0.5, ctl='FAULTED', safety='LOCKED', faults=[fault])
    rig.feed_ok(1.0)
    assert last_lock(rig) is True
    assert rig.resume().success
    assert last_lock(rig) is False


def test_malformed_message_locks(rig):
    rig.node.gate.set_type_unavailable(False)
    rig.feed_ok()
    assert rig.resume().success
    bad = status(99)
    del bad.sequence
    rig.clock.t += 0.1
    rig.node._on_status_msg(bad)
    assert last_lock(rig) is True


def test_node_publishes_no_velocity():
    # the monitor has no Twist publisher: it only inhibits, it never commands motion
    import inspect
    import openamrobot_drivers.base_status_monitor as module
    source = inspect.getsource(module)
    assert 'Twist' not in source.replace('twist_mux', '')
