#!/usr/bin/env python3
"""Test BOUCLE OUVERTE : meme PWM fixe sur les 2 moteurs (PID ignore).
ROUES EN L'AIR. 24V. Main sur la coupure. Pots IDENTIQUES (3,5 / 3,5).

Publie /debug/openloop (x=PWM) a 20 Hz pendant DUREE, logue /debug/left et
/debug/right (counts bruts + rpm mesure). On compare : a PWM identique et
pots identiques, les deux roues DOIVENT reagir de facon similaire.
  - similaire et lisse        -> les 2 canaux sont sains
  - droite erratique/cale/saute -> defaut canal droit (moteur/Hall ou encodeur)

Usage : python3 ~/openloop_test.py [pwm] [duree]
  defaut : pwm=120, duree=10
Securite : abort si |rpm| > 60 ; envoie 0 a la fin / abort / Ctrl-C ;
le firmware coupe aussi si plus de commande openloop pendant 300 ms.
"""
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Vector3

PWM = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
RPM_ABORT = 60.0

BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=10)


class OL(Node):
    def __init__(self):
        super().__init__('openloop_test')
        self.pub = self.create_publisher(Vector3, '/debug/openloop', 10)
        self.create_subscription(Vector3, '/debug/left', self.cl, BE)
        self.create_subscription(Vector3, '/debug/right', self.cr, BE)
        self.l = (0.0, 0.0, 0.0)   # cible, mesure, counts
        self.r = (0.0, 0.0, 0.0)

    def cl(self, m):
        self.l = (m.x, m.y, m.z)

    def cr(self, m):
        self.r = (m.x, m.y, m.z)

    def send(self, pwm):
        v = Vector3()
        v.x = float(pwm)
        self.pub.publish(v)

    def stop(self, n=15):
        for _ in range(n):
            self.send(0.0)
            time.sleep(0.02)


def main():
    rclpy.init()
    node = OL()
    t0 = time.time()
    while time.time() - t0 < 0.6:
        rclpy.spin_once(node, timeout_sec=0.05)

    print(f"BOUCLE OUVERTE : PWM={PWM:+.0f} sur les 2 moteurs, {DUR:.0f}s "
          f"(abort si |rpm|>{RPM_ABORT})")
    print(" t   | GAUCHE rpm  cnt     d | DROITE rpm  cnt     d")
    print("-----+----------------------+----------------------", flush=True)

    rl, rr = [], []          # mesures rpm pour stats
    aborted = None
    prevL = None
    prevR = None
    sumL = 0
    sumR = 0
    t0 = time.time()
    last = -1.0
    try:
        while time.time() - t0 < DUR:
            node.send(PWM)
            rclpy.spin_once(node, timeout_sec=0.03)
            if abs(node.l[1]) > RPM_ABORT or abs(node.r[1]) > RPM_ABORT:
                aborted = f"rpm>{RPM_ABORT} (G={node.l[1]:.0f} D={node.r[1]:.0f})"
                break
            now = time.time()
            if now - last >= 0.3:
                last = now
                lz, rz = node.l[2], node.r[2]
                if prevL is not None:
                    dL, dR = int(lz - prevL), int(rz - prevR)
                    if abs(dL) < 2000 and abs(dR) < 2000:   # ignore artefact de capture
                        sumL += dL
                        sumR += dR
                        rl.append(node.l[1])
                        rr.append(node.r[1])
                        print(f"{now - t0:4.1f} | {node.l[1]:+6.1f} {int(lz):+8d} {dL:+5d} | "
                              f"{node.r[1]:+6.1f} {int(rz):+8d} {dR:+5d}", flush=True)
                prevL, prevR = lz, rz
    except KeyboardInterrupt:
        aborted = "Ctrl-C"
    finally:
        node.stop()

    def stats(name, xs):
        if not xs:
            return f"{name}: (pas de donnees)"
        n = len(xs)
        mean = sum(xs) / n
        var = sum((x - mean) ** 2 for x in xs) / n
        return (f"{name}: moy={mean:+5.1f} rpm  min={min(xs):+5.1f}  "
                f"max={max(xs):+5.1f}  ecart-type={var**0.5:4.1f}")

    tL = sumL
    tR = sumR
    ratio = (tR / tL) if tL else 0.0
    print("\n----- RESULTAT -----")
    print(f"abort = {aborted if aborted else 'non (duree ecoulee)'}")
    print(stats("GAUCHE", rl))
    print(stats("DROITE", rr))
    print(f"COUNTS TOTAUX : GAUCHE={int(tL)}  DROITE={int(tR)}  ratio D/G = {ratio:.3f}")
    print("Objectif ratio D/G = 1.000 (vitesses egales a PWM identique).")
    print("  ratio < 1  -> droite trop LENTE  -> MONTER un peu le pot droit")
    print("  ratio > 1  -> droite trop RAPIDE -> BAISSER un peu le pot droit")
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


main()
