#!/usr/bin/env python3
# Affichage live PID par roue : cible / mesure / ERREUR (cible-mesure) / counts.
# /debug/left,/debug/right : Vector3  x=cible_rpm  y=mesure_rpm  z=counts.
# Usage (Ubuntu, Cyclone domain 0) : python3 encpid.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Vector3

class EncPid(Node):
    def __init__(self):
        super().__init__('enc_pid')
        q = QoSProfile(depth=10)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        q.history = HistoryPolicy.KEEP_LAST
        self.L = None
        self.R = None
        self.maxerr = {'GAUCHE': 0.0, 'DROITE': 0.0}
        self.create_subscription(Vector3, '/debug/left',  lambda m: setattr(self, 'L', m), q)
        self.create_subscription(Vector3, '/debug/right', lambda m: setattr(self, 'R', m), q)
        self.create_timer(0.2, self.tick)  # 5 Hz
        print(f"\n{'roue':6s} | {'cible':>7s} | {'mesure':>7s} | {'ERREUR':>8s} | {'|err|max':>8s} | {'count':>9s}")
        print('-'*62)

    def line(self, name, m):
        if m is None:
            print(f'{name:6s} |   (pas de donnee)')
            return
        tgt, meas = m.x, m.y
        err = tgt - meas
        if abs(err) > self.maxerr[name]:
            self.maxerr[name] = abs(err)
        print(f'{name:6s} | {tgt:7.2f} | {meas:7.2f} | {err:+8.2f} | {self.maxerr[name]:8.2f} | {m.z:9.0f}')

    def tick(self):
        self.line('GAUCHE', self.L)
        self.line('DROITE', self.R)
        print('-'*62)

def main():
    rclpy.init()
    n = EncPid()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
