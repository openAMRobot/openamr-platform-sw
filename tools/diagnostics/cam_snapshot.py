#!/usr/bin/env python3
"""Capture instantanee de la camera cote robot (Pi headless).
Le topic /camera/image_raw/compressed est deja du JPEG -> on ecrit les octets tels quels.
Usage : source ROS + camera_ws, puis : python3 ~/cam_snapshot.py [chemin.jpg]
"""
import sys, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data

OUT = sys.argv[1] if len(sys.argv) > 1 else "/home/botshare/cam_snapshot.jpg"

class Snap(Node):
    def __init__(self):
        super().__init__("cam_snapshot")
        self.done = False
        self.create_subscription(CompressedImage, "/camera/image_raw/compressed",
                                 self.cb, qos_profile_sensor_data)
    def cb(self, m):
        if self.done:
            return
        with open(OUT, "wb") as f:
            f.write(bytes(m.data))
        print(f"snapshot ecrit: {OUT} ({len(m.data)} octets, format {m.format})")
        self.done = True

def main():
    rclpy.init()
    n = Snap()
    t0 = time.time()
    while rclpy.ok() and not n.done and time.time() - t0 < 8:
        rclpy.spin_once(n, timeout_sec=0.3)
    if not n.done:
        print("AUCUNE image recue (camera_node lance ? camera_ws source ?)")

if __name__ == "__main__":
    main()
