#!/usr/bin/env python3
# Lecteur d'angle encodeur en direct (tourne la roue a la main et regarde).
# /debug/left et /debug/right : Vector3, champ z = counts cumules. 1024 counts/tour.
# Usage (Ubuntu, Cyclone domain 0) : python3 encread.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Vector3

CPR = 1024.0  # counts par tour (AS5040, quadrature)

class EncRead(Node):
    def __init__(self):
        super().__init__('enc_read')
        q = QoSProfile(depth=10)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        q.history = HistoryPolicy.KEEP_LAST
        self.last = {'GAUCHE': None, 'DROITE': None}
        self.create_subscription(Vector3, '/debug/left',  lambda m: self.cb('GAUCHE', m), q)
        self.create_subscription(Vector3, '/debug/right', lambda m: self.cb('DROITE', m), q)
        print("Tourne une roue a la main. (Ctrl-C pour quitter)\n")

    def cb(self, wheel, msg):
        cnt = msg.z
        prev = self.last[wheel]
        if prev is not None and cnt == prev:
            return
        d = '' if prev is None else f'  delta={cnt-prev:+.0f}'
        self.last[wheel] = cnt
        ang = cnt / CPR * 360.0
        print(f'{wheel:6s} | count={cnt:10.0f} | angle={ang:9.1f} deg | tour={ang%360.0:6.1f} deg{d}')

def main():
    rclpy.init()
    n = EncRead()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
