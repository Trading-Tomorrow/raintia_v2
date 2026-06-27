#!/usr/bin/env python3
"""
Monitor ao vivo do skew das paredes durante o mapeamento manual.
Imprime uma linha a cada ~4s. Alerta se as paredes comecarem a entortar.

Referencia: ang_paredes ~22 graus (offset inicial do robot, NORMAL e constante).
ALERTA se: o angulo se afastar muito da banda estabelecida, ou o spread disparar.
"""
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class Monitor(Node):
    def __init__(self):
        super().__init__('monitor_vivo')
        self.ultimo = None
        self.create_subscription(OccupancyGrid, '/map',
                                 lambda m: setattr(self, 'ultimo', m), 1)
        self.base = None          # angulo de referencia (primeira leitura estavel)
        self.hist = []

    def medir(self, m):
        w, h = m.info.width, m.info.height
        arr = np.array(m.data, dtype=np.int16).reshape(h, w)
        occ = int(np.count_nonzero(arr > 50))
        img = np.zeros((h, w), np.uint8)
        img[arr > 50] = 255
        lines = cv2.HoughLinesP(img, 1, np.pi/180, threshold=10,
                                minLineLength=8, maxLineGap=2)
        segs = []
        if lines is not None:
            for l in lines:
                x1, y1, x2, y2 = l[0]
                comp = np.hypot(x2-x1, y2-y1)
                ang = np.degrees(np.arctan2(y2-y1, x2-x1)) % 90.0
                segs.append((comp, min(ang, 90.0-ang)))
        segs.sort(reverse=True)
        longas = [d for _, d in segs[:6]]
        ang = float(np.median(longas)) if longas else -1.0
        spread = float(np.std(longas)) if len(longas) > 1 else 0.0
        return occ, ang, spread

    def loop(self):
        print("=== MONITOR DE SKEW AO VIVO (Ctrl+C para parar) ===", flush=True)
        print("Conduz o robot no teleop. Vou avisar se as paredes entortarem.\n", flush=True)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.ultimo is None:
                continue
            occ, ang, spread = self.medir(self.ultimo)
            if ang < 0:
                continue
            # estabelecer baseline com as primeiras leituras
            if self.base is None and occ > 200:
                self.base = ang
            estado = "OK  paredes retas"
            if self.base is not None:
                desvio = abs(ang - self.base)
                if desvio > 12 or spread > 12:
                    estado = ">>> ALERTA: possivel SKEW (paredes a entortar) <<<"
                elif desvio > 7 or spread > 8:
                    estado = "  ~ atencao: ligeiro desvio"
            print(f"occ={occ:4d}  ang_paredes={ang:5.1f}deg  spread={spread:4.1f}  "
                  f"base={self.base if self.base else '--'}  -> {estado}", flush=True)
            time.sleep(4)


def main():
    rclpy.init()
    m = Monitor()
    try:
        m.loop()
    except KeyboardInterrupt:
        pass
    finally:
        m.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
