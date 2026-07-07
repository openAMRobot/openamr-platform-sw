#!/usr/bin/env python3
"""Vue ASCII du scan + estimation de l'AVANT par l'arc ouvert.

Binne le scan PAR ANGLE (robuste au nb de points variable). Pour chaque secteur
de 10 deg, affiche la distance min. Le corps du robot bloque une partie (retours
tres proches / absents) -> l'arc OUVERT (vers la piece) a son centre = l'avant.

Lance :  python3 ~/lidar_view.py
Astuce : place ton objet droit devant, il apparaitra comme un creux net de
distance dans un secteur -> ca confirme ou est l'avant.
"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST, depth=10)
SECT = 10            # degres par secteur
BLOCK = 0.35         # m : en dessous = bloque (corps du robot)


class V(Node):
    def __init__(self):
        super().__init__('lidar_view')
        self.create_subscription(LaserScan, '/scan', self.cb, QOS)
        self.acc = {}     # secteur (deg, multiple de SECT, -180..170) -> min dist

    def cb(self, msg):
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            ang = math.degrees(msg.angle_min + i * msg.angle_increment)
            s = int(math.floor(ang / SECT) * SECT)
            if s not in self.acc or r < self.acc[s]:
                self.acc[s] = r


def main():
    rclpy.init()
    node = V()
    t0 = time.time()
    while time.time() - t0 < 2.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    sectors = list(range(-180, 180, SECT))
    print("\nSCAN — distance min par secteur (repere LIDAR, 0deg = avant du capteur)")
    print(" angle |  dist  | (mur/objet)            etat")
    print("-------+--------+--------------------------------")
    open_secs = []
    for s in sectors:
        d = node.acc.get(s)
        if d is None:
            print(f" {s:+4d}  |   --   |                          (rien)")
        elif d < BLOCK:
            print(f" {s:+4d}  | {d:5.2f}  | {'#'*min(int(d*10),24):24s} BLOQUE")
        else:
            print(f" {s:+4d}  | {d:5.2f}  | {'#'*min(int(d*10),24):24s} ouvert")
            open_secs.append(s)

    # centre de l'arc ouvert (gestion du wrap +-180)
    if open_secs:
        xs = [math.cos(math.radians(s + SECT / 2)) for s in open_secs]
        ys = [math.sin(math.radians(s + SECT / 2)) for s in open_secs]
        center = math.degrees(math.atan2(sum(ys), sum(xs)))
        print("-------+--------+--------------------------------")
        print(f"Centre de l'arc OUVERT ~ {center:+.0f} deg  (= avant probable du robot)")
        print(f" -> yaw TF base_link->lidar_link ~ {-center:+.0f} deg = {math.radians(-center):+.3f} rad")
        print("(a confirmer : place l'objet droit devant, il doit tomber pres de ce centre)")
    node.destroy_node()
    rclpy.shutdown()


main()
