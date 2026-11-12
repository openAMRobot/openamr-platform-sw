#!/usr/bin/env python3
# Mesure la rotation TOTALE vue par l'odom EKF (yaw cumule, deroule).
# Tourne le robot exactement 1 tour complet (360 deg) a la main -> compare a l'affichage.
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry

class YawTest(Node):
    def __init__(self):
        super().__init__('yaw_test')
        self.create_subscription(Odometry, '/odom', self.cb, 10)
        self.last = None
        self.total = 0.0
        print("Tourne le robot LENTEMENT d'exactement 1 tour (360 deg) a la main.")
        print("Le 'total' doit afficher ~360 (ou -360). (Ctrl-C pour finir)\n")

    def cb(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        if self.last is not None:
            d = yaw - self.last
            if d > math.pi: d -= 2*math.pi      # deroulement
            if d < -math.pi: d += 2*math.pi
            self.total += d
        self.last = yaw
        print(f'  yaw instant = {math.degrees(yaw):7.1f} deg   |   ROTATION TOTALE = {math.degrees(self.total):8.1f} deg', end='\r')

def main():
    rclpy.init()
    n = YawTest()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        print(f'\n\n>>> ROTATION TOTALE MESUREE = {math.degrees(n.total):.1f} deg  (cible: 360 si tu as fait 1 tour)')
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
