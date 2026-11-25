"""ROS 2 node: capture vcan_drive for the whole run and audit the SDO traffic to the drive.

Listen only: it sends nothing. Every frame goes to a candump -L file (parameter capture_file)
with the kernel receive time. The SDO frames of the drive node go through renode_fw.SdoAudit,
whose counters are published as JSON on /sil/drive_bus/audit at 5 Hz. sil.launch.py starts it
before Renode and the host test client, and both wait for its first message, so the capture
covers every frame they send. Test fixture: it judges no motion and stops nothing.
"""
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from openamrobot_sil import config, renode_fw
from openamrobot_sil.canbus import open_bus
from openamrobot_sil.nodes.common import outside_gazebo_timer, publish_json


class DriveBusAuditNode(Node):
    def __init__(self):
        super().__init__('sil_drive_bus_audit')
        cfg = config.load(self.declare_parameter('config_file', '').value)
        self.capture_file = self.declare_parameter(
            'capture_file', '/tmp/openamrobot_sil_vcan_drive.log').value
        drive = cfg['buses']['drive']
        self.interface = drive['interface']
        self.audit = renode_fw.SdoAudit(drive['zlac_node_id'])
        self.frames = 0
        self.lock = threading.Lock()
        self.bus = open_bus(self.interface)
        self.out = open(self.capture_file, 'w', encoding='utf-8')
        self.pub = self.create_publisher(String, '/sil/drive_bus/audit', 10)
        self._running = True
        self._thread = threading.Thread(target=self._rx, daemon=True)
        self._thread.start()
        outside_gazebo_timer(self, 0.2, self.publish)
        self.get_logger().info('capturing %s to %s' % (self.interface, self.capture_file))

    def _rx(self):
        while self._running:
            msg = self.bus.recv(timeout=0.1)
            if msg is None:
                continue
            t = msg.timestamp or time.time()
            with self.lock:
                self.frames += 1
                self.out.write(renode_fw.candump_line(t, self.interface, msg.arbitration_id,
                                                      msg.data, msg.is_extended_id) + '\n')
                self.audit.feed(msg.arbitration_id, msg.data, t, msg.is_extended_id)

    def publish(self):
        with self.lock:
            self.out.flush()
            data = self.audit.summary()
            data.update(frames=self.frames, interface=self.interface,
                        capture_file=self.capture_file)
        publish_json(self.pub, data)

    def shutdown(self):
        self._running = False
        self._thread.join(timeout=1)
        with self.lock:
            self.out.close()
        self.bus.shutdown()


def main():
    rclpy.init()
    node = DriveBusAuditNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
