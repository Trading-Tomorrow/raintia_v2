#!/usr/bin/env python3
"""
Mede a 'rectidao' das paredes do mapa SLAM em tempo real.
Deteta deteta lines com Hough e calcula o desvio angular medio em relacao
aos eixos (0/90 graus). Paredes alinhadas -> skew ~0. Mapa torto -> skew sobe.

Imprime uma linha JSON: {dim, occ, skew_deg, n_lines}
"""
import sys, time, json
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

def analisar(grid):
    w, h = grid.info.width, grid.info.height
    arr = np.array(grid.data, dtype=np.int16).reshape(h, w)
    occ = np.count_nonzero(arr > 50)
    # imagem binaria das paredes (ocupado = branco)
    img = np.zeros((h, w), dtype=np.uint8)
    img[arr > 50] = 255
    # detetar segmentos de linha
    lines = cv2.HoughLinesP(img, 1, np.pi/180, threshold=10,
                            minLineLength=8, maxLineGap=2)
    segs = []
    if lines is not None:
        for l in lines:
            x1, y1, x2, y2 = l[0]
            comp = np.hypot(x2 - x1, y2 - y1)
            ang = np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 90.0
            d = min(ang, 90.0 - ang)  # desvio ao eixo mais proximo
            segs.append((comp, d))
    # ordenar por comprimento; as paredes reais sao as linhas mais compridas
    segs.sort(reverse=True)
    longas = [d for _, d in segs[:6]]          # 6 linhas mais compridas (paredes)
    ang_paredes = float(np.median(longas)) if longas else -1.0
    # dispersao: paredes retas e paralelas -> spread baixo. Mapa torto -> spread alto.
    spread = float(np.std(longas)) if len(longas) > 1 else 0.0
    return {'dim': f'{w}x{h}', 'occ': occ,
            'ang_paredes': round(ang_paredes, 2),
            'spread': round(spread, 2),
            'n_lines': len(segs)}

def main():
    rclpy.init()
    n = Node('skew_monitor')
    box = {}
    n.create_subscription(OccupancyGrid, '/map', lambda m: box.__setitem__('m', m), 1)
    t = time.time()
    while time.time() - t < 6 and 'm' not in box:
        rclpy.spin_once(n, timeout_sec=0.3)
    if 'm' in box:
        print(json.dumps(analisar(box['m'])))
    else:
        print(json.dumps({'erro': 'sem mapa'}))
    rclpy.shutdown()

if __name__ == '__main__':
    main()
