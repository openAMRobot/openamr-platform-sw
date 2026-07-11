#!/usr/bin/env python3
"""Split /rosout into per-node topics under /logs/<node>.

Every ROS 2 node already publishes its logs to /rosout. This node fans that
firehose out into ONE topic per source, so you can watch a single subsystem
instead of scrolling the whole bring-up console:

    ros2 topic echo /logs/dock_trigger --field data     # docking
    ros2 topic echo /logs/camera       --field data     # camera
    ros2 topic echo /logs/controller_server --field data
    ros2 topic echo /logs/all          --field data     # everything, node-prefixed

Run it ON THE PI (it subscribes to /rosout locally — no Wi-Fi cost; you only
echo the one topic you care about from the PC). Levels are prefixed in each line
([INFO]/[WARN]/[ERROR]/…). Ctrl-C to stop.
"""
import re
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from std_msgs.msg import String

LEVELS = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}


def sanitize(name: str) -> str:
    """Node/logger name -> valid topic suffix, keeping namespace hierarchy."""
    n = (name or 'unknown').strip('/')
    n = re.sub(r'[^A-Za-z0-9_/]', '_', n)   # dots (sub-loggers) -> _
    return n or 'unknown'


class LogSplitter(Node):
    def __init__(self):
        super().__init__('log_splitter')
        self._pubs = {}
        self._all = self.create_publisher(String, '/logs/all', 50)
        self.create_subscription(Log, '/rosout', self._cb, 200)
        self.get_logger().info(
            'log_splitter up: /rosout -> /logs/<node>. '
            'e.g. ros2 topic echo /logs/dock_trigger --field data')

    def _pub_for(self, key: str):
        p = self._pubs.get(key)
        if p is None:
            p = self.create_publisher(String, f'/logs/{key}', 50)
            self._pubs[key] = p
        return p

    def _cb(self, msg: Log):
        if msg.name == 'log_splitter':
            return                                   # don't echo ourselves
        lvl = LEVELS.get(msg.level, str(msg.level))
        line = f'[{lvl}] {msg.msg}'
        self._pub_for(sanitize(msg.name)).publish(String(data=line))
        self._all.publish(String(data=f'[{msg.name}] {line}'))


def main():
    rclpy.init()
    node = LogSplitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
