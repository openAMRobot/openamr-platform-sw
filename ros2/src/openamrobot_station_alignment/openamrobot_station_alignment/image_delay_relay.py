# SPDX-License-Identifier: MIT
"""image_delay_relay (test only): republishes images `delay_s` of simulation time after their stamp.

Reproduces in the station alignment world what Baseline 1 showed at a real-time factor of 0.2: station poses whose
image stamps are several seconds of simulation time old when they reach the alignment. Not for the robot.
"""
import collections

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image


class ImageDelayRelay(Node):
    def __init__(self):
        super().__init__("image_delay_relay")
        self.declare_parameter("input", "/base_camera/color/image_raw")
        self.declare_parameter("output", "/base_camera/color/image_delayed")
        self.declare_parameter("delay_s", 7.0)
        self.queue = collections.deque()
        self.pub = self.create_publisher(Image, self.get_parameter("output").value, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter("input").value, self.queue.append, qos_profile_sensor_data)
        self.create_timer(0.02, self.release)

    def release(self):
        delay = int(float(self.get_parameter("delay_s").value) * 1e9)
        now = self.get_clock().now().nanoseconds
        while self.queue and Time.from_msg(self.queue[0].header.stamp).nanoseconds + delay <= now:
            self.pub.publish(self.queue.popleft())


def main(args=None):
    rclpy.init(args=args)
    node = ImageDelayRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
