#!/usr/bin/env python3
"""Test guide des encodeurs (roues en l'air, a la main, SANS alim 24V).

Le script te dit quoi faire et quand, avec compte a rebours, detecte le
mouvement en direct, et affiche une conclusion. Lance-le dans TON terminal :

    python3 ~/guided_encoder_test.py

Vitesses reconstruites depuis /odom/unfiltered :
    v_droite = lin.x + ang.z * (LR/2)
    v_gauche = lin.x - ang.z * (LR/2)
"""
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

LR = 0.45
HALF = LR / 2.0
SEUIL = 0.01  # m/s : au-dessus = mouvement detecte


class Mon(Node):
    def __init__(self):
        super().__init__('guided_encoder_test')
        self.create_subscription(Odometry, '/odom/unfiltered', self.cb, 10)
        self.vl = 0.0
        self.vr = 0.0

    def cb(self, msg):
        lin = msg.twist.twist.linear.x
        ang = msg.twist.twist.angular.z
        self.vr = lin + ang * HALF
        self.vl = lin - ang * HALF


def phase(node, titre, secondes, mesurer=True):
    print("\n" + "=" * 50)
    print(titre)
    print("=" * 50, flush=True)
    max_l = 0.0
    max_r = 0.0
    t0 = time.time()
    last_print = -1
    while time.time() - t0 < secondes:
        rclpy.spin_once(node, timeout_sec=0.05)
        if mesurer:
            max_l = max(max_l, abs(node.vl))
            max_r = max(max_r, abs(node.vr))
        restant = int(secondes - (time.time() - t0)) + 1
        if restant != last_print:
            last_print = restant
            live = ""
            if mesurer:
                gl = "GAUCHE✓" if abs(node.vl) > SEUIL else "gauche·"
                dr = "DROITE✓" if abs(node.vr) > SEUIL else "droite·"
                live = f"   [{gl}  {dr}]  vg={node.vl:+.3f} vd={node.vr:+.3f}"
            print(f"   {restant:2d}s...{live}", flush=True)
    return max_l, max_r


def main():
    rclpy.init()
    node = Mon()
    # purge du pic transitoire de demarrage
    phase(node, "INIT - ne touche a rien (purge demarrage)", 3, mesurer=False)
    gl, gr = phase(node, ">>> TOURNE LA ROUE  G-A-U-C-H-E  (MOTOR1) <<<", 10)
    phase(node, "PAUSE - arrete tout", 3, mesurer=False)
    dl, dr = phase(node, ">>> TOURNE LA ROUE  D-R-O-I-T-E  (MOTOR2) <<<", 10)

    print("\n" + "#" * 50)
    print("RESULTAT")
    print("#" * 50)
    print(f"Phase GAUCHE tournee : max v_gauche={gl:.3f}  max v_droite={gr:.3f}")
    print(f"Phase DROITE tournee : max v_gauche={dl:.3f}  max v_droite={dr:.3f}")
    print("-" * 50)
    enc_g = gl > SEUIL or dl > SEUIL  # un signal pendant qu'on tourne gauche
    enc_d = gr > SEUIL or dr > SEUIL  # un signal pendant qu'on tourne droite

    def verdict(nom, max_attendu, max_autre):
        if max_attendu > SEUIL and max_autre <= SEUIL:
            return f"{nom}: OK (encodeur compte, bonne voie)"
        if max_attendu <= SEUIL and max_autre > SEUIL:
            return f"{nom}: signal sur la MAUVAISE voie (cablage/assignation inversee ?)"
        if max_attendu <= SEUIL and max_autre <= SEUIL:
            return f"{nom}: AUCUN signal -> encodeur muet (HS/debranche ?)"
        return f"{nom}: signal sur les DEUX voies (etrange)"

    print(verdict("Roue GAUCHE", gl, gr))
    print(verdict("Roue DROITE", dr, dl))
    print("#" * 50, flush=True)

    node.destroy_node()
    rclpy.shutdown()


main()
