"""
Base status monitor: holds the command mux lock until the base is RUN and fresh.

Subscribes /base/status and /base/aux_status (I8 rev 3), publishes

* ``/base/command_lock`` (std_msgs/Bool, true = locked) at 20 Hz, for the twist_mux
  ``locks`` entry. The twist_mux lock timeout makes a dead monitor fail closed.
* ``/diagnostics`` (diagnostic_msgs/DiagnosticArray) at 2 Hz with the operator text.
* ``/base/monitor_text`` (std_msgs/String, transient local) with the current operator line.

Serves ``/base/resume`` (std_srvs/Trigger). Resume clears the inhibit only when every
blocker is gone; nothing resumes by itself and no velocity is ever published here.
This is a functional inhibit. The hardwired E-stop chain is the safety function.
"""
import os

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from openamrobot_drivers.base_status_adapter import (
    aux_snapshot, base_snapshot, FieldMap)
from openamrobot_drivers.base_status_core import (
    AUX_STALE_S, BaseGate, Blocker, FaultRegistry, STATUS_STALE_S)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import yaml

# I8 decision 9: reliable, transient local, depth 1.
STATUS_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)

LOCK_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)


def load_registry(path):
    """Read a fault registry YAML (format TODO(Evans), I8 decision 5). Empty path: none."""
    if not path:
        return FaultRegistry()
    if not os.path.exists(path):
        raise FileNotFoundError('fault registry not found: ' + path)
    with open(path) as handle:
        data = yaml.safe_load(handle) or {}
    return FaultRegistry(data.get('faults', data))


class BaseStatusMonitor(Node):
    """ROS wrapper around BaseGate."""

    def __init__(self, clock_fn=None, **kwargs):
        super().__init__('base_status_monitor', **kwargs)
        self.declare_parameter('status_topic', '/base/status')
        self.declare_parameter('aux_topic', '/base/aux_status')
        self.declare_parameter('status_type', 'openamr_base_msgs/msg/BaseStatus')
        self.declare_parameter('aux_type', 'openamr_base_msgs/msg/BaseAuxStatus')
        self.declare_parameter('lock_topic', '/base/command_lock')
        self.declare_parameter('resume_service', '/base/resume')
        self.declare_parameter('status_stale_s', STATUS_STALE_S)
        self.declare_parameter('aux_stale_s', AUX_STALE_S)
        self.declare_parameter('require_aux', True)
        self.declare_parameter('supported_contract_versions', [1])
        self.declare_parameter('lock_rate_hz', 20.0)
        self.declare_parameter('fault_registry_file', '')
        # TODO(Evans): field names and enum tables from the #30 draft PR.
        self.declare_parameter('field_names', ['__none__'])
        self.declare_parameter('controller_state_enum', ['__none__'])
        self.declare_parameter('safety_state_enum', ['__none__'])

        self._now = clock_fn or (lambda: self.get_clock().now().nanoseconds * 1e-9)
        self._fmap = FieldMap(self._pairs('field_names'), {
            'controller_state': self._pairs('controller_state_enum'),
            'safety_state': self._pairs('safety_state_enum')})
        self.gate = BaseGate(
            registry=load_registry(self.get_parameter('fault_registry_file').value),
            require_aux=self.get_parameter('require_aux').value,
            supported_versions=tuple(self.get_parameter('supported_contract_versions').value),
            status_stale_s=self.get_parameter('status_stale_s').value,
            aux_stale_s=self.get_parameter('aux_stale_s').value)

        self._lock_pub = self.create_publisher(
            Bool, self.get_parameter('lock_topic').value, LOCK_QOS)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self._text_pub = self.create_publisher(
            String, '/base/monitor_text',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(
            Trigger, self.get_parameter('resume_service').value, self._on_resume)

        self._subs = []
        self._subscribe('status_type', 'status_topic', self._on_status_msg)
        self._subscribe('aux_type', 'aux_topic', self._on_aux_msg)

        period = 1.0 / float(self.get_parameter('lock_rate_hz').value)
        self.create_timer(period, self._publish_lock)
        self.create_timer(0.5, self._publish_diagnostics)
        self._last_text = None

    # -- parameter helpers ---------------------------------------------------
    def _pairs(self, name):
        """Read a list of 'key=value' strings as a dict (empty marker means none)."""
        out = {}
        for item in self.get_parameter(name).value:
            if item == '__none__' or '=' not in item:
                continue
            key, value = item.split('=', 1)
            out[key.strip()] = value.strip()
        return out

    def _subscribe(self, type_param, topic_param, callback):
        try:
            msg_type = get_message(self.get_parameter(type_param).value)
        except (AttributeError, ImportError, ModuleNotFoundError, ValueError) as err:
            self.get_logger().error(
                'message type %s unavailable (%s); the lock stays engaged.'
                % (self.get_parameter(type_param).value, err))
            self.gate.set_type_unavailable()
            return
        self._subs.append(self.create_subscription(
            msg_type, self.get_parameter(topic_param).value, callback, STATUS_QOS))

    # -- inputs --------------------------------------------------------------
    def _on_status_msg(self, msg):
        self.gate.on_status(base_snapshot(msg, self._fmap), self._now())

    def _on_aux_msg(self, msg):
        self.gate.on_aux(aux_snapshot(msg, self._fmap), self._now())

    def _on_resume(self, request, response):
        del request
        response.success, response.message = self.gate.resume(self._now())
        self.get_logger().info('resume: %s' % response.message)
        self._publish_lock()
        return response

    # -- outputs -------------------------------------------------------------
    def _publish_lock(self):
        state = self.gate.evaluate(self._now())
        self._lock_pub.publish(Bool(data=bool(state.locked)))
        if state.summary != self._last_text:
            self._last_text = state.summary
            self._text_pub.publish(String(data=state.summary))
            self.get_logger().info('base gate: %s' % state.summary)

    def _publish_diagnostics(self):
        now = self._now()
        state = self.gate.evaluate(now)
        status = DiagnosticStatus(name='base_status_monitor', hardware_id='base_controller')
        if not state.locked:
            status.level, status.message = DiagnosticStatus.OK, state.summary
        elif state.blockers:
            stale_or_fault = {Blocker.STATUS_STALE, Blocker.AUX_STALE, Blocker.STATUS_MALFORMED,
                              Blocker.AUX_MALFORMED, Blocker.MESSAGE_TYPE_UNAVAILABLE}
            fault = any(b in stale_or_fault for b in state.blockers) or (
                state.base is not None and any(f.active for f in state.base.faults))
            status.level = DiagnosticStatus.ERROR if fault else DiagnosticStatus.WARN
            status.message = state.summary
        else:
            status.level, status.message = DiagnosticStatus.WARN, state.summary
        values = {
            'locked': str(state.locked).lower(),
            'status_state': state.status_state,
            'aux_state': state.aux_state,
            'status_age_s': '%.3f' % self.gate.status.age(now),
            'aux_age_s': '%.3f' % self.gate.aux.age(now),
            'blockers': ','.join(b.value for b in state.blockers),
            'publisher_restarts': str(self.gate.restarts),
            'sequence_gaps': str(self.gate.sequence_gaps),
        }
        for index, line in enumerate(state.operator_lines):
            values['text_%d' % index] = line
        status.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diag_pub.publish(array)


def main(args=None):
    """Run the node."""
    rclpy.init(args=args)
    node = BaseStatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
