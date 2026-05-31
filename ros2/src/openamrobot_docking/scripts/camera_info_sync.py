#!/usr/bin/env python3
"""Republish CameraInfo with the latest image timestamp for exact sync users."""

import copy
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


class CameraInfoSync(Node):
    def __init__(self):
        super().__init__('camera_info_sync')

        self.declare_parameter('image_topic', '/rgb_image')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('synced_camera_info_topic', '/camera_info_synced')

        self._latest_info = None
        self._lock = threading.Lock()

        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        synced_topic = self.get_parameter('synced_camera_info_topic').value

        self._info_pub = self.create_publisher(CameraInfo, synced_topic, QoSProfile(depth=10))
        self.create_subscription(CameraInfo, camera_info_topic, self._on_camera_info, QoSProfile(depth=10))
        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)

        self.get_logger().info(
            f'camera_info_sync: {camera_info_topic} + {image_topic} -> {synced_topic}'
        )

    def _on_camera_info(self, msg):
        with self._lock:
            self._latest_info = msg

    def _on_image(self, msg):
        with self._lock:
            if self._latest_info is None:
                return
            info = copy.deepcopy(self._latest_info)

        info.header.stamp = msg.header.stamp
        info.header.frame_id = msg.header.frame_id
        self._info_pub.publish(info)


def main():
    rclpy.init()
    node = CameraInfoSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
